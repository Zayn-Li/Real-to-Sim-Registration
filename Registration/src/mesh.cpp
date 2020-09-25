#include "mesh.h"
#include "platform.h"

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

void Mesh::ImportMeshFromPly(const char* path)
{
    ifstream file(path, ios_base::in | ios_base::binary);


    // some scratch memory
    const uint32_t kMaxLineLength = 1024;
    char buffer[kMaxLineLength];

    //double startTime = GetSeconds();

    file >> buffer;

    PlyFormat format = eAscii;
    uint32_t cam_prop = 0;
    numFaces = 0;


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
            else if (strcmp(buffer, "camera") == 0) 
            {
                vertexElement = false;
                file >> cam_prop;
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
        else if (strcmp(buffer, "property") == 0 )
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
    // file.close();
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
			// printf ("error line: Point1: %d Point2: %d Point3: %d\n", indices[0], indices[1], indices[2]);
            assert(!"invalid number of indices, only support tris and quads");
			break;
		};

		// // calculate vertex normals as we go
        // Point3& v0 = m_positions[indices[0]];
        // Point3& v1 = m_positions[indices[1]];
        // Point3& v2 = m_positions[indices[2]];

        // Vector3 n = SafeNormalize(Cross(v1-v0, v2-v0), Vector3(0.0f, 1.0f, 0.0f));

		// for (uint32_t i=0; i < numIndices; ++i)
		// {
	    //     m_normals[indices[i]] += n;
	    // }
	}

    // for (uint32_t i=0; i < numVertices; ++i)
    // {
    //     m_normals[i] = SafeNormalize(m_normals[i], Vector3(0.0f, 1.0f, 0.0f));
    // }

    //cout << "Imported mesh " << path << " in " << (GetSeconds()-startTime)*1000.f << "ms" << endl;


}


void Mesh::ExportToPly(const char* path)
{
	ofstream file(path);

    if (!file)
        return;

	// numVertices = m_position.size();
    // numFaces = 0;
    file << "ply" << endl;
    file << "format ascii 1.0" << endl;
    file << "element vertex " << numVertices << endl;
    file << "property float x" << endl << "property float y" << endl << "property float z" << endl;
    file << "element face " << numFaces << endl;
    file << "property list char uint32 vertex_indices" << endl;
    file << "end_header" << endl;

	for (uint32_t i=0; i < numVertices; ++i)
	{
		
        Point3 v = m_positions[i];
		file  << v.x << " " << v.y << " " << v.z << endl;
	}

}


