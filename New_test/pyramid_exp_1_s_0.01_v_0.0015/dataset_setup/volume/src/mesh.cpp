#include "mesh.h"
#include "platform.h"

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/mls.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/filters/extract_indices.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>
#include <iostream>
#include <fstream>
#include <vector>


// #include <igl/copyleft/tetgen/tetrahedralize.h>
// #include <igl/readPLY.h>
// #include <igl/writePLY.h>


using namespace std;


Mesh::Mesh(const Input& mesh_in)
{
    // position of each particle (x,y,z)
    m_positions = mesh_in.m_positions;
    // estimated normal of each particle (x,y,z)
    m_normals = mesh_in.m_normals;
    // colour of each particle for visualizing
    m_colours = mesh_in.m_colours;
    // indices of particles in each mesh tet.
    m_indices = mesh_in.m_indices;       
}

Mesh::Mesh()
{
}

Mesh::~Mesh()
{
}

void Mesh::DuplicateVertex(uint32_t i)
{
	assert(m_positions.size() > i);	
	m_positions.push_back(m_positions[i]);
	
	if (m_normals.size() > i)
		m_normals.push_back(m_normals[i]);
	
	if (m_colours.size() > i)
		m_colours.push_back(m_colours[i]);
	
}

void Mesh::AddMesh(const Mesh& m)
{
    uint32_t offset = uint32_t(m_positions.size());

    // add new vertices
    m_positions.insert(m_positions.end(), m.m_positions.begin(), m.m_positions.end());
    m_normals.insert(m_normals.end(), m.m_normals.begin(), m.m_normals.end());
    m_colours.insert(m_colours.end(), m.m_colours.begin(), m.m_colours.end());

    // add new indices with offset
    for (uint32_t i=0; i < m.m_indices.size(); ++i)
    {
        m_indices.push_back(m.m_indices[i]+offset);
    }    
}

void Mesh::Normalize(float s)
{
	Vec3 lower, upper;
	GetBounds(lower, upper);
	Vec3 edges = upper-lower;

	Transform(TranslationMatrix(Point3(-lower)));

	float maxEdge = max(edges.x, max(edges.y, edges.z));
	Transform(ScaleMatrix(s/maxEdge));
}

void Mesh::CalculateNormals()
{
	m_normals.resize(0);
	m_normals.resize(m_positions.size());

	int numTris = int(GetNumTets());

	for (int i=0; i < numTris; ++i)
	{
		int a = m_indices[i*3+0];
		int b = m_indices[i*3+1];
		int c = m_indices[i*3+2];
		
		Vec3 n = Cross(m_positions[b]-m_positions[a], m_positions[c]-m_positions[a]);

		m_normals[a] += n;
		m_normals[b] += n;
		m_normals[c] += n;
	}

	int numVertices = int(GetNumVertices());

	for (int i=0; i < numVertices; ++i)
		m_normals[i] = ::Normalize(m_normals[i]);
}

void Mesh::Transform(const Matrix44& m)
{
    for (uint32_t i=0; i < m_positions.size(); ++i)
    {
        m_positions[i] = m*m_positions[i];
        m_normals[i] = m*m_normals[i];
    }
}

void Mesh::GetBounds(Vector3& outMinExtents, Vector3& outMaxExtents) const
{
    Point3 minExtents(FLT_MAX);
    Point3 maxExtents(-FLT_MAX);

    // calculate face bounds
    for (uint32_t i=0; i < m_positions.size(); ++i)
    {
        const Point3& a = m_positions[i];

        minExtents = Min(a, minExtents);
        maxExtents = Max(a, maxExtents);
    }

    outMinExtents = Vector3(minExtents);
    outMaxExtents = Vector3(maxExtents);
}

namespace 
{

    enum PlyFormat
    {
        eAscii,
        eBinaryBigEndian    
    };

    template <typename T>
    T PlyRead(ifstream& s, PlyFormat format)
    {
        T data = eAscii;

        switch (format)
        {
            case eAscii:
            {
                s >> data;
                break;
            }
            case eBinaryBigEndian:
            {
                char c[sizeof(T)];
                s.read(c, sizeof(T));
                reverse(c, c+sizeof(T));
                data = *(T*)c;
                break;
            }      
			default:
				assert(0);
        }

        return data;
    }

} // namespace anonymous

void Mesh::ImportMesh(const char* path)
{
	std::string ext = GetExtension(path);

	Mesh* mesh = NULL;

	if (ext == "bin")
		ImportMeshFromBin(path);
	else if (ext == "pcd")
		ImportMeshFromPcd(path);
	else if (ext == "ply")
		ImportMeshFromPly(path);
}

void Mesh::ImportMeshFromBin(const char* path)
{
	double start = GetSeconds();

	FILE* f = fopen(path, "rb");

	if (f)
	{
		int numVertices;
		int numIndices;

		size_t len;
		len = fread(&numVertices, sizeof(numVertices), 1, f);
		len = fread(&numIndices, sizeof(numIndices), 1, f);

		m_positions.resize(numVertices);
		m_normals.resize(numVertices);
		m_indices.resize(numIndices);

		len = fread(&m_positions[0], sizeof(Vec3)*numVertices, 1, f);
		len = fread(&m_normals[0], sizeof(Vec3)*numVertices, 1, f);
		len = fread(&m_indices[0], sizeof(int)*numIndices, 1, f);

		(void)len;
		
		fclose(f);

		double end = GetSeconds();

		printf("Imported mesh %s in %f ms\n", path, (end-start)*1000.0f);

	}

}


void Mesh::ImportMeshFromPly(const char* path)
{
    ifstream file(path, ios_base::in | ios_base::binary);


    // some scratch memory
    const uint32_t kMaxLineLength = 1024;
    char buffer[kMaxLineLength];

    //double startTime = GetSeconds();

    file >> buffer;

    PlyFormat format = eAscii;


    const uint32_t kMaxProperties = 16;
    uint32_t numProperties = 0; 
    float properties[kMaxProperties];

    bool vertexElement = false;

    while (file)
    {
        file >> buffer;

        if (strcmp(buffer, "element") == 0)
        {
            file >> buffer;

            if (strcmp(buffer, "face") == 0)
            {                
                vertexElement = false;
                file >> numFaces;
            }

            else if (strcmp(buffer, "vertex") == 0)
            {
                vertexElement = true;
                file >> numVertices;
            }
        }
        else if (strcmp(buffer, "format") == 0)
        {
            file >> buffer;
            if (strcmp(buffer, "ascii") == 0)
            {
                format = eAscii;
            }
            else if (strcmp(buffer, "binary_big_endian") == 0)
            {
                format = eBinaryBigEndian;
            }
			else
			{
				printf("Ply: unknown format\n");

			}
        }
        else if (strcmp(buffer, "property") == 0)
        {
            if (vertexElement)
                ++numProperties;
        }
        else if (strcmp(buffer, "end_header") == 0)
        {
            break;
        }
    }

    // eat newline
    char nl;
    file.read(&nl, 1);
	
	// debug
#if ENABLE_VERBOSE_OUTPUT
	printf ("Loaded mesh: %s numFaces: %d numVertices: %d format: %d numProperties: %d\n", path, numFaces, numVertices, format, numProperties);
#endif

    m_positions.resize(numVertices);
    m_normals.resize(numVertices);
    m_colours.resize(numVertices, Colour(1.0f, 1.0f, 1.0f, 1.0f));

    m_indices.reserve(numFaces*3);

    // read vertices
    for (uint32_t v=0; v < numVertices; ++v)
    {
        for (uint32_t i=0; i < numProperties; ++i)
        {
            properties[i] = PlyRead<float>(file, format);
        }

        m_positions[v] = Point3(properties[0], properties[1], properties[2]);
        m_normals[v] = Vector3(0.0f, 0.0f, 0.0f);
    }

    // read indices
    for (uint32_t f=0; f < numFaces; ++f)
    {
        uint32_t numIndices = (format == eAscii)?PlyRead<uint32_t>(file, format):PlyRead<uint8_t>(file, format);
		uint32_t indices[4];

		for (uint32_t i=0; i < numIndices; ++i)
		{
			indices[i] = PlyRead<uint32_t>(file, format);
		}

		switch (numIndices)
		{
		case 3:
			m_indices.push_back(indices[0]);
			m_indices.push_back(indices[1]);
			m_indices.push_back(indices[2]);
			break;
		case 4:
			m_indices.push_back(indices[0]);
			m_indices.push_back(indices[1]);
			m_indices.push_back(indices[2]);

			m_indices.push_back(indices[2]);
			m_indices.push_back(indices[3]);
			m_indices.push_back(indices[0]);
			break;

		default:
			assert(!"invalid number of indices, only support tris and quads");
			break;
		};

		// calculate vertex normals as we go
        Point3& v0 = m_positions[indices[0]];
        Point3& v1 = m_positions[indices[1]];
        Point3& v2 = m_positions[indices[2]];

        Vector3 n = SafeNormalize(Cross(v1-v0, v2-v0), Vector3(0.0f, 1.0f, 0.0f));

		for (uint32_t i=0; i < numIndices; ++i)
		{
	        m_normals[indices[i]] += n;
	    }
	}

    for (uint32_t i=0; i < numVertices; ++i)
    {
        m_normals[i] = SafeNormalize(m_normals[i], Vector3(0.0f, 1.0f, 0.0f));
    }

    //cout << "Imported mesh " << path << " in " << (GetSeconds()-startTime)*1000.f << "ms" << endl;


}

void Mesh::CreateVolumeMesh(string in_path, string out_path)
{
    pcl::PolygonMesh vol_mesh;
    pcl::PolygonMesh surf_mesh;
    pcl::PolygonMesh proj_mesh;
    pcl::PointCloud<pcl::PointXYZ> vol_pt;
    pcl::PointCloud<pcl::PointXYZ>::Ptr surf_pt (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2 surf_pt_2;
    pcl::io::loadPLYFile (in_path, surf_mesh);
    pcl::fromPCLPointCloud2 (surf_mesh.cloud, *surf_pt);
    proj_mesh = surf_mesh;
    vol_mesh = surf_mesh;  
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    // Create a set of planar coefficients X,Y,Z,d
    coefficients->values.resize (4);
    coefficients->values[0] = -0.19;
    coefficients->values[1] = 0.62;
    coefficients->values[2] = 0.76;
    coefficients->values[3] = -0.18;

    // Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();

    // // Define a translation
    // transform_1 (1,3) = 0.0028;
    // transform_1 (2,3) = 0.0028;
    // pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud_1 (new pcl::PointCloud<pcl::PointXYZ> ());
    
    // pcl::transformPointCloud (*surf_pt, *transformed_cloud_1, transform_1);

    // vol_pt = *surf_pt + *transformed_cloud_1;


    // Create the filtering object
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (surf_pt);
    proj.setModelCoefficients (coefficients);
    proj.filter (*cloud_projected);

    vol_pt = *surf_pt + *cloud_projected;
    // for (int i = 0; i < vol_pt.width; i++)
    // {
    //     vol_pt.points[i].x = vol_pt.points[i].x * 1000;
    //     vol_pt.points[i].y = vol_pt.points[i].y * 1000; 
    //     vol_pt.points[i].z = vol_pt.points[i].z * 1000; 
    // }    
    pcl::toPCLPointCloud2 (vol_pt, surf_pt_2);
    vol_mesh.cloud = surf_pt_2;

    // Create a Concave Hull representation of the projected inliers
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointIndices	hull_point_indices;
    pcl::ConcaveHull<pcl::PointXYZ> chull;
    chull.setInputCloud (cloud_projected);
    chull.setAlpha (0.004);
    chull.setKeepInformation(true);
    chull.reconstruct (*cloud_hull);
    chull.getHullPointIndices(hull_point_indices);   
    
    // handcraft side surface
    uint32_t layerSize;
    layerSize = cloud_projected->width; 
    std::vector<int> layer_indices;
    layer_indices = hull_point_indices.indices;
    layer_indices.push_back(layer_indices[0]);
    
    string hull_path;
    hull_path = out_path + "/hull_idx.txt";
	std::ofstream file(hull_path);

    if (!file)
        return;
	for (uint32_t i=0; i < layer_indices.size()-1; ++i)
	{		
        file << layer_indices[i]<< std::endl;
	}
	for (uint32_t i=0; i < layer_indices.size()-1; ++i)
	{		
        file << layer_indices[i]+layerSize << std::endl;
	}    
    file.close();
    for (int i = 0; i < surf_mesh.polygons.size(); i++)
    {
        pcl::Vertices newMesh;
        newMesh.vertices.resize (3);
        newMesh.vertices[0] = vol_mesh.polygons[i].vertices[0]+layerSize;
        newMesh.vertices[1] = vol_mesh.polygons[i].vertices[1]+layerSize;
        newMesh.vertices[2] = vol_mesh.polygons[i].vertices[2]+layerSize;
        vol_mesh.polygons.push_back(newMesh);
    }

    for (int i = 0; i < layer_indices.size()-1; i++)
    {
        pcl::Vertices newMesh;
        newMesh.vertices.resize (3);
        newMesh.vertices[0] = layer_indices[i];
        newMesh.vertices[1] = layer_indices[i]+layerSize;
        newMesh.vertices[2] = layer_indices[i+1]+layerSize;
        vol_mesh.polygons.push_back (newMesh);
        pcl::Vertices newMesh2;
        newMesh2.vertices.resize (3);
        newMesh2.vertices[0] = layer_indices[i];
        newMesh2.vertices[1] = layer_indices[i+1];
        newMesh2.vertices[2] = layer_indices[i+1]+layerSize;
        vol_mesh.polygons.push_back (newMesh2);
    }
    string vol_path;
    vol_path = out_path + "/vol_mesh.ply";    
    pcl::io::savePLYFile (vol_path, vol_mesh);
}
void Mesh::ImportMeshFromPcd(const char* path)
{
   
    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());

    // Fill in the cloud data
    pcl::PCDReader reader;
    // Replace the path below with the path where you saved your file
    reader.read (path, *cloud); // Remember to download the file first!

    std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
        << " data points (" << pcl::getFieldsList (*cloud) << ").";

    
    pcl::RadiusOutlierRemoval<pcl::PCLPointCloud2> outrem;
    pcl::PCLPointCloud2::Ptr cloud_r (new pcl::PCLPointCloud2 ());
    // build the filter
    outrem.setInputCloud(cloud);
    outrem.setRadiusSearch(0.8);
    outrem.setMinNeighborsInRadius (2);
    // apply filter
    outrem.filter (*cloud_r);

    // Create the filtering object
    pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloud_r);
    sor.setLeafSize (2,2,2);
    sor.filter (*cloud_filtered);
    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
        << " data points (" << pcl::getFieldsList (*cloud_filtered) << ").";

    
    // Load input file into a PointCloud<T> with an appropriate type
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_new (new pcl::PointCloud<pcl::PointXYZ>);
    
    // pcl::PCLPointCloud2 cloud_blob;
    // pcl::io::loadPCDFile ("./test_downsampled.pcd", cloud_blob);

    pcl::fromPCLPointCloud2 (*cloud_filtered, *cloud_new);

    // manully delete some outliers
    pcl::PointIndices::Ptr mainTissue(new pcl::PointIndices());
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    for (int i = 0; i < (*cloud_new).size(); i++)
    {
        pcl::PointXYZ pt(cloud_new->points[i].x, cloud_new->points[i].y, cloud_new->points[i].z);
        float THRESHOLD = 50;
        if (abs(pt.x - pt.y) > THRESHOLD) // e.g. remove all pts above 
        {
        mainTissue->indices.push_back(i);
        }
    }
    extract.setInputCloud(cloud_new);
    extract.setIndices(mainTissue);
    extract.setNegative(true);
    extract.filter(*cloud_new);

    std::cout << "Successfully delete outliers."<< std::endl;

    /* 
    point cloud projection
    */
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_3d (new pcl::PointCloud<pcl::PointXYZ>); 
    *cloud_3d += *cloud_new;
    
    
    // for(int i_proj = 0;i_proj<1;i_proj++ )
    // {
    int i_proj = 0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    // Create a set of planar coefficients X,Y,Z,d
    coefficients->values.resize (4);
    coefficients->values[0] = 0;
    coefficients->values[1] = 0;
    coefficients->values[2] = -1;
    coefficients->values[3] = 120+i_proj;

    // Create the filtering object
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (cloud_new);
    proj.setModelCoefficients (coefficients);
    proj.filter (*cloud_projected);
    
    
    // Create a Concave Hull representation of the projected inliers
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointIndices	hull_point_indices;
    pcl::ConcaveHull<pcl::PointXYZ> chull;
    chull.setInputCloud (cloud_projected);
    chull.setAlpha (5);
    chull.setKeepInformation(true);
    chull.reconstruct (*cloud_hull);
    chull.getHullPointIndices(hull_point_indices);

    *cloud_3d += *cloud_projected;

    std::cout << "Successfully projected."<< std::endl;

    // Normal estimation*
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud_3d);
    n.setInputCloud (cloud_3d);
    n.setSearchMethod (tree);
    n.setKSearch (20);
    n.compute (*normals);
    //* normals should not contain the point normals + surface curvatures

    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (*cloud_3d, *normals, *cloud_with_normals);
    //* cloud_with_normals = cloud + normals

    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud (cloud_with_normals);

    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangles;

    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius (15);

    // Set typical values for the parameters
    gp3.setMu (5);
    gp3.setMaximumNearestNeighbors (500);
    gp3.setMaximumSurfaceAngle(M_PI/3); // 45 degrees
    gp3.setMinimumAngle(M_PI/18); // 10 degrees
    gp3.setMaximumAngle(M_PI/1.2); // 120 degrees
    gp3.setNormalConsistency(false);

    // Get result
    gp3.setInputCloud (cloud_with_normals);
    gp3.setSearchMethod (tree2);
    gp3.reconstruct (triangles);

    // Additional vertex information
    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();

    std::cout << "Successfully create mesh."<< std::endl;


    // handcraft side surface
    uint32_t layerSize;
    layerSize = cloud_projected->width; 
    std::vector<int,std::allocator<int>> layer_indices;
    layer_indices = hull_point_indices.indices;

    
    for (int i = 0; i < layer_indices.size()-1; i++)
    {
        pcl::Vertices newMesh;
        newMesh.vertices.resize (3);
        newMesh.vertices[0] = layer_indices[i];
        newMesh.vertices[1] = layer_indices[i]+layerSize;
        newMesh.vertices[2] = layer_indices[i+1]+layerSize;
        triangles.polygons.push_back (newMesh);
        pcl::Vertices newMesh2;
        newMesh2.vertices.resize (3);
        newMesh2.vertices[0] = layer_indices[i];
        newMesh2.vertices[1] = layer_indices[i+1];
        newMesh2.vertices[2] = layer_indices[i+1]+layerSize;
        triangles.polygons.push_back (newMesh2);
    }

    std::cout << "Successfully handcraft mesh."<< std::endl;

    // // Save

    // pcl::io::saveVTKFile ("mesh2.vtk", triangles);
    
    // // Finish
    // return (0);


    const uint32_t kMaxProperties = 16;
    uint32_t numProperties = 0; 
    float properties[kMaxProperties];

    std::cout << "Initial"<< std::endl;


    // debug
    #if ENABLE_VERBOSE_OUTPUT
    printf ("Loaded mesh: %s numFaces: %d numVertices: %d format: %d numProperties: %d\n", path, numFaces, numVertices, format, numProperties);
    #endif

    numVertices = triangles.cloud.height * triangles.cloud.width;
    std::cout << "numVertices"<< std::endl;
    numFaces = triangles.polygons.size();
    std::cout << "numFaces"<< std::endl;
    unsigned int point_size = static_cast<unsigned int> (triangles.cloud.data.size () / numVertices);

    printf ("Loaded mesh: numFaces: %d numVertices: %d\n", numFaces, numVertices);

    m_positions.resize(numVertices);
    m_normals.resize(numVertices);
    m_colours.resize(numVertices, Colour(1.0f, 1.0f, 1.0f, 1.0f));

    m_indices.reserve(numFaces*3);

    

    // read vertices

    for (unsigned int i = 0; i < numVertices; ++i)
    {
    int xyz = 0;
    for (std::size_t d = 0; d < triangles.cloud.fields.size (); ++d)
    {
        if ((triangles.cloud.fields[d].datatype == pcl::PCLPointField::FLOAT32) && (
            triangles.cloud.fields[d].name == "x" || 
            triangles.cloud.fields[d].name == "y" || 
            triangles.cloud.fields[d].name == "z"))
        {
        float value;
        memcpy (&value, &triangles.cloud.data[i * point_size + triangles.cloud.fields[d].offset], sizeof (float));
        properties[xyz] = value;
        
        if (++xyz == 3)
            break;
        }
    }
    if (xyz != 3)
    {
        PCL_ERROR ("[pcl::io::saveVTKFile] Input point cloud has no XYZ data!\n");
        return ;
    }
    m_positions[i] = Point3(properties[0], properties[1]+10, -properties[2]);
    m_normals[i] = Vector3(0.0f, 0.0f, 0.0f);
    }
    std::cout << "Successfully write vertices."<< std::endl;
    // for (uint32_t v=0; v < numVertices; ++v)
    // {
    //     for (uint32_t i=0; i < numProperties; ++i)
    //     {
    //         properties[i] = PlyRead<float>(file, format);
    //     }

    //     mesh->m_positions[v] = Point3(properties[0], properties[1], properties[2]);
    //     mesh->m_normals[v] = Vector3(0.0f, 0.0f, 0.0f);
    // }

    // read indices
    for (uint32_t f=0; f < numFaces; ++f)
    {
        uint32_t numIndices = 3;
        uint32_t indices[3];

        for (uint32_t i=0; i < numIndices; ++i)
        {
            indices[i] = triangles.polygons[f].vertices[i];
        }
        
        m_indices.push_back(indices[0]);
        m_indices.push_back(indices[1]);
        m_indices.push_back(indices[2]);

        
        // calculate vertex normals 
        Point3& v0 = m_positions[indices[0]];
        Point3& v1 = m_positions[indices[1]];
        Point3& v2 = m_positions[indices[2]];

        Vector3 n = SafeNormalize(Cross(v1-v0, v2-v0), Vector3(0.0f, 1.0f, 0.0f));

        for (uint32_t i=0; i < numIndices; ++i)
        {
            m_normals[indices[i]] += n;
        }
    }
    std::cout << "Successfully write indices."<< std::endl;

    for (uint32_t i=0; i < numVertices; ++i)
    {
        m_normals[i] = SafeNormalize(m_normals[i], Vector3(0.0f, 1.0f, 0.0f));
    }
    std::cout << "Successfully write normals."<< std::endl;

    //cout << "Imported mesh " << path << " in " << (GetSeconds()-startTime)*1000.f << "ms" << endl;



}


void Mesh::ExportToOff(const char* path)
{
	ofstream file(path);

    if (!file)
        return;

	file << "OFF" << endl;
    file << numParticles << " " << numTet << " " << "0" << endl;

	for (uint32_t i=1; i < numParticles; ++i)
	{
		
        Point3 v = m_positions[i];
		file  << v.x << " " << v.y << " " << v.z << endl;
	}

	for (uint32_t i=0; i < numTet; ++i)
	{
		
		file << "4" << " " << m_indices[i*4] << " " << m_indices[i*4+1] << " " << m_indices[i*4+2] << " " << m_indices[i*4+3] << endl;
	}

}

void Mesh::ExportToPly(const char* path)
{
	ofstream file(path);

    if (!file)
        return;

	file << "ply" << endl;
    file << "format ascii 1.0" << endl;
    file << "element vertex " << numVertices << endl;
    file << "property float x" << endl << "property float y" << endl << "property float z" << endl;
    file << "element face " << numFaces << endl;
    file << "property list char uint32 vertex_indices" << endl;
    file << "end_header" << endl;

	for (uint32_t i=1; i < numVertices; ++i)
	{
		
        Point3 v = m_positions[i];
		file  << v.x << " " << v.y << " " << v.z << endl;
	}

	for (uint32_t i=0; i < numFaces; ++i)
	{
		file << char('3') << " " << uint32_t(m_indices[i*3]) << " " << uint32_t(m_indices[i*3+1]) << " " << uint32_t(m_indices[i*3+2]) << endl;
	}

}

void Mesh::ExportLabeledSurf(const char* path)
{
	ofstream file(path);

    if (!file)
        return;

    file << "OFF "  << endl;
    file << numVertices << " " << numFaces << " " << "0" << endl;

	for (uint32_t i=0; i < numVertices; ++i)
	{
		
        Point3 v = m_positions[i];
		file  << v.x << " " << v.y << " " << v.z << endl;
	}

	for (uint32_t i=0; i < numFaces; ++i)
	{
		file << char('3') << " " << uint32_t(m_indices[i*3]) << " " << uint32_t(m_indices[i*3+1]) << " " << uint32_t(m_indices[i*3+2]) << endl;
	}

}





typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Polyhedron_3<Kernel>     Polyhedron;
typedef Polyhedron::Halfedge_handle    Halfedge_handle;
typedef Polyhedron::Facet_handle       Facet_handle;
typedef Polyhedron::Vertex_handle      Vertex_handle;

int Mesh::HoleFilling()
{
  const char* filename = "./holes.off";
  std::ifstream input(filename);
  Polyhedron poly;
  if ( !input || !(input >> poly) || poly.empty() ) {
    std::cerr << "Not a valid off file." << std::endl;
    return 1;
  }
  // Incrementally fill the holes
  unsigned int nb_holes = 0;
  for(Halfedge_handle h : halfedges(poly))
  {
    if(h->is_border())
    {
      std::vector<Facet_handle>  patch_facets;
      std::vector<Vertex_handle> patch_vertices;
      bool success = std::get<0>(
        CGAL::Polygon_mesh_processing::triangulate_refine_and_fair_hole(
                  poly,
                  h,
                  std::back_inserter(patch_facets),
                  std::back_inserter(patch_vertices),
     CGAL::Polygon_mesh_processing::parameters::vertex_point_map(get(CGAL::vertex_point, poly)).
                  geom_traits(Kernel())) );
      std::cout << " Number of facets in constructed patch: " << patch_facets.size() << std::endl;
      std::cout << " Number of vertices in constructed patch: " << patch_vertices.size() << std::endl;
      std::cout << " Fairing : " << (success ? "succeeded" : "failed") << std::endl;
      ++nb_holes;
    }
  }
  std::cout << std::endl;
  std::cout << nb_holes << " holes have been filled" << std::endl;
  
  std::ofstream out("filled.off");
  out.precision(17);
  out << poly << std::endl;

  return 0;

}



// transfer a surface mesh into tetgen object
tetgenio Mesh::ExportMeshToTetGen()
{
    // tetgenio in;
    // tetgenio out;
    // tetgenio::facet *f;
    // tetgenio::polygon *p;
    // in.load_ply("./lung1.ply");
    // printf ("good1");
    // numVertices = in.numberofpoints;
    // numFaces = in.numberoffacets;
    // for (int i=0; i<in.numberofpoints; i++){

    //     m_positions[i][0] = in.pointlist[i * 3];
 
    //     m_positions[i][1] = in.pointlist[i * 3 + 1];

    //     m_positions[i][2] = in.pointlist[i * 3 + 2];
    // } 
    // printf ("good2");
    // for (int i=0; i<in.numberoffacets; i++){
    // // Facet 1. The leftmost facet.
    //     f = &in.facetlist[i];
    //     p = &f->polygonlist[0];
    //     m_indices[i * 3] = p->vertexlist[0];
    //     m_indices[i * 3 + 1] = p->vertexlist[1];
    //     m_indices[i * 3 + 2] = p->vertexlist[2];

    // }    
    // printf ("good3");
    // ExportMeshToBin("./filledsave.bin");       
    // tetrahedralize("pq", &in, &out);
    tetgenio in;
    tetgenio out;
    // tetgenio out2;
    tetgenio::facet *f;
    tetgenio::polygon *p;
    int i;
    numVertices = m_positions.size();
    numFaces = m_indices.size()/3;
    // All indices start from 1.
    in.firstnumber = 1;
    printf ("Loaded mesh: numFaces: %d numVertices: %d\n", numFaces, numVertices);
    in.numberofpoints = numVertices;;
    in.pointlist = new REAL[in.numberofpoints * 3];
    for (i=0; i<in.numberofpoints; i++){

        in.pointlist[i * 3]     = m_positions[i][0];
 
        in.pointlist[i * 3 + 1] = m_positions[i][1];

        in.pointlist[i * 3 + 2] = m_positions[i][2];
    }
    printf ("good1");
    
    in.numberoffacets = numFaces;
    in.facetlist = new tetgenio::facet[in.numberoffacets];
    in.facetmarkerlist = new int[in.numberoffacets];
    printf ("good2");
    for (i=0; i<in.numberoffacets; i++){
    // Facet 1. The leftmost facet.
        f = &in.facetlist[i];
        f->numberofpolygons = 1;
        f->polygonlist = new tetgenio::polygon[1];
        f->numberofholes = 0;
        f->holelist = NULL;
        p = &f->polygonlist[0];
        p->numberofvertices = 3;
        p->vertexlist = new int[p->numberofvertices];
        p->vertexlist[0] = m_indices[i * 3] + 1;
        p->vertexlist[1] = m_indices[i * 3 + 1] + 1;
        p->vertexlist[2] = m_indices[i * 3 + 2] + 1;

    }
    for (i=0; i<in.numberoffacets; i++){
        in.facetmarkerlist[i] = 0;  
    }
    // in.load_off("filled.off");
    // Output the PLC to files 'barin.node' and 'barin.poly'.
    in.save_nodes("barin");
    in.save_poly("barin");

    // Tetrahedralize the PLC. Switches are chosen to read a PLC (p),
    //   do quality mesh generation (q) with a specified quality bound
    //   (1.414), and apply a maximum volume constraint (a0.1).

    // tetrahedralize("w", &in, &out);
    tetrahedralize("pq", &in, &out);
    printf ("Vertices: ");
    
    m_positions.resize(out.numberofpoints);
    m_indices.reserve(out.numberoftetrahedra * 4);
    numParticles = out.numberofpoints;
    numTet = out.numberoftetrahedra;
    for (i=0; i < out.numberofpoints; i++){

        m_positions[i][0] = out.pointlist[i * 3];
 
        m_positions[i][1] = out.pointlist[i * 3 + 1];

        m_positions[i][2] = out.pointlist[i * 3 + 2];

    }
    for (i = 0; i < out.numberoftetrahedra; i++) {
        m_indices[i * 4] = out.tetrahedronlist[i * 4];
        m_indices[i * 4 + 1] = out.tetrahedronlist[i * 4 + 1];
        m_indices[i * 4 + 2] = out.tetrahedronlist[i * 4 + 2];
        m_indices[i * 4 + 3] = out.tetrahedronlist[i * 4 + 3];
    }
    // Output mesh to files 'barout.node', 'barout.ele' and 'barout.face'.
    // out.save_nodes("barout");
    // out.save_elements("barout");
    // out.save_faces("barout");

    return out;

    // tetgenio in, out;
    // tetgenio::facet *f;
    // tetgenio::polygon *p;
    // int i;
    // // All indices start from 1.
    // in.firstnumber = 1;
    // in.numberofpoints = 8;
    // in.pointlist = new REAL[in.numberofpoints * 3];
    // in.pointlist[0] = 0; // node 1.
    // in.pointlist[1] = 0;
    // in.pointlist[2] = 0;
    // in.pointlist[3] = 2; // node 2.
    // in.pointlist[4] = 0;
    // in.pointlist[5] = 0;
    // in.pointlist[6] = 2; // node 3.
    // in.pointlist[7] = 2;
    // in.pointlist[8] = 0;
    // in.pointlist[9] = 0; // node 4.
    // in.pointlist[10] = 2;
    // in.pointlist[11] = 0;
    // // Set node 5, 6, 7, 8.
    // for (i = 4; i < 8; i++) {
    // in.pointlist[i * 3] = in.pointlist[(i - 4) * 3];
    // in.pointlist[i * 3 + 1] = in.pointlist[(i - 4) * 3 + 1];
    // in.pointlist[i * 3 + 2] = 12;
    // }
    // in.numberoffacets = 6;
    // in.facetlist = new tetgenio::facet[in.numberoffacets];
    // in.facetmarkerlist = new int[in.numberoffacets];
    // // Facet 1. The leftmost facet.
    // f = &in.facetlist[0];
    // f->numberofpolygons = 1;
    // f->polygonlist = new tetgenio::polygon[f->numberofpolygons];
    // f->numberofholes = 0;
    // f->holelist = NULL;
    // p = &f->polygonlist[0];
    // p->numberofvertices = 4;
    // p->vertexlist = new int[p->numberofvertices];
    // p->vertexlist[0] = 1;
    // p->vertexlist[1] = 2;
    // p->vertexlist[2] = 3;
    // p->vertexlist[3] = 4;
    // // Facet 2. The rightmost facet.
    // f = &in.facetlist[1];
    // f->numberofpolygons = 1;
    // f->polygonlist = new tetgenio::polygon[f->numberofpolygons];
    // f->numberofholes = 0;
    // f->holelist = NULL;
    // p = &f->polygonlist[0];
    // p->numberofvertices = 4;
    // p->vertexlist = new int[p->numberofvertices];
    // p->vertexlist[0] = 5;
    // p->vertexlist[1] = 6;
    // p->vertexlist[2] = 7;
    // p->vertexlist[3] = 8;
    // // Facet 3. The bottom facet.
    // f = &in.facetlist[2];
    // f->numberofpolygons = 1;
    // f->polygonlist = new tetgenio::polygon[f->numberofpolygons];
    // f->numberofholes = 0;
    // f->holelist = NULL;
    // p = &f->polygonlist[0];
    // p->numberofvertices = 4;
    // p->vertexlist = new int[p->numberofvertices];
    // p->vertexlist[0] = 1;
    // p->vertexlist[1] = 5;
    // p->vertexlist[2] = 6;
    // p->vertexlist[3] = 2;
    // // Facet 4. The back facet.
    // f = &in.facetlist[3];
    // f->numberofpolygons = 1;
    // f->polygonlist = new tetgenio::polygon[f->numberofpolygons];
    // f->numberofholes = 0;
    // f->holelist = NULL;
    // p = &f->polygonlist[0];
    // p->numberofvertices = 4;
    // p->vertexlist = new int[p->numberofvertices];
    // p->vertexlist[0] = 2;
    // p->vertexlist[1] = 6;
    // p->vertexlist[2] = 7;
    // p->vertexlist[3] = 3;
    // // Facet 5. The top facet.
    // f = &in.facetlist[4];
    // f->numberofpolygons = 1;
    // f->polygonlist = new tetgenio::polygon[f->numberofpolygons];
    // f->numberofholes = 0;
    // f->holelist = NULL;
    // p = &f->polygonlist[0];
    // p->numberofvertices = 4;
    // p->vertexlist = new int[p->numberofvertices];
    // p->vertexlist[0] = 3;
    // p->vertexlist[1] = 7;
    // p->vertexlist[2] = 8;
    // p->vertexlist[3] = 4;
    // // Facet 6. The front facet.
    // f = &in.facetlist[5];
    // f->numberofpolygons = 1;
    // f->polygonlist = new tetgenio::polygon[f->numberofpolygons];
    // f->numberofholes = 0;
    // f->holelist = NULL;
    // p = &f->polygonlist[0];
    // p->numberofvertices = 4;
    // p->vertexlist = new int[p->numberofvertices];
    // p->vertexlist[0] = 4;
    // p->vertexlist[1] = 8;
    // p->vertexlist[2] = 5;
    // p->vertexlist[3] = 1;
    // // Set ’in.facetmarkerlist’
    // in.facetmarkerlist[0] = -1;
    // in.facetmarkerlist[1] = -2;
    // in.facetmarkerlist[2] = 0;
    // in.facetmarkerlist[3] = 0;
    // in.facetmarkerlist[4] = 0;
    // in.facetmarkerlist[5] = 0;
    // // Output the PLC to files ’barin.node’ and ’barin.poly’.
    // in.save_nodes("barin");
    // in.save_poly("barin");    
    // // Tetrahedralize the PLC. Switches are chosen to read a PLC (p),
    // // do quality mesh generation (q) with a specified quality bound
    // // (1.414), and apply a maximum volume constraint (a0.1).
    // tetrahedralize("pq", &in, &out);
    // // Output mesh to files ’barout.node’, ’barout.ele’ and ’barout.face’.
    // out.save_nodes("barout");
    // out.save_elements("barout");
} 



// trying to introduce igl for tethedral mesh
void Mesh::CreateTetMesh()
{

};

void Mesh::ExportMeshToBin(const char* path)
{
	FILE* f = fopen(path, "wb");

	if (f)
	{
		int numVertices = int(m_positions.size());
		int numIndices = int(m_indices.size());

		fwrite(&numVertices, sizeof(numVertices), 1, f);
		fwrite(&numIndices, sizeof(numIndices), 1, f);

		// write data blocks
		fwrite(&m_positions[0], sizeof(Vec3)*numVertices, 1, f);
		fwrite(&m_normals[0], sizeof(Vec3)*numVertices, 1, f);		
		fwrite(&m_indices[0], sizeof(int)*numIndices, 1, f);

		fclose(f);
	}
}

