#pragma once

#include <vector>
#include <string>

#include "core.h"
#include "maths.h"
#include "tetgen.h"

class Mesh
{
private:
    

public:

    struct Input
    {
        // position of each particle (x,y,z)
        std::vector<Point3> m_positions;
        // estimated normal of each particle (x,y,z)
        std::vector<Vector3> m_normals;
        // unknown attribute in FLEX data structure
        // std::vector<Vector2> m_texcoords[2];
        // colour of each particle for visualizing
        std::vector<Colour> m_colours;
        // indices of particles in each mesh tet.
        std::vector<uint32_t> m_indices;        
    };
    // position of each particle (x,y,z)
    std::vector<Point3> m_positions;
    // estimated normal of each particle (x,y,z)
    std::vector<Vector3> m_normals;
    // unknown attribute in FLEX data structure
    // std::vector<Vector2> m_texcoords[2];
    // colour of each particle for visualizing
    std::vector<Colour> m_colours;
    // indices of particles in each mesh tet.
    std::vector<uint32_t> m_indices;  
    // number of surface points
    uint32_t  numVertices;
    // number of triangle on surface
    uint32_t  numFaces;      
    // number of points
    uint32_t  numParticles;
    // number of tetrahedral
    uint32_t  numTet; 
    // number of internal vertices
    uint32_t  numInter;

    // float radius = 0.5;
    // float smoothing = 0.0;

    Mesh(const Input& mesh_in);
    Mesh();
    virtual ~Mesh();
    
    void AddMesh(const Mesh& m);
    
    // statistic of mesh
    uint32_t GetNumVertices() { 
        numVertices = m_positions.size(); 
        return numVertices;
    }

    uint32_t GetNumTets() { 
        numFaces = m_indices.size()/ 3;
        return numFaces; 
    }
    
    // operations on mesh

    void CalculateNormals();
    void Transform(const Matrix44& t_matrix);
	void Normalize(float s=1.0f);	// scale so bounds in any dimension equals s and lower bound = (0,0,0)
    // duplicate one particle and put it at the end of all attributes
    void DuplicateVertex(uint32_t i);

    void GetBounds(Vector3& minExtents, Vector3& maxExtents) const;

    // create volume mesh from surface mesh
    void CreateVolumeMesh(std::string in_path, std::string out_path);
    // create mesh from point cloud
    void ImportMeshFromPcd(const char* path);
    // create mesh from saved mesh
    void ImportMeshFromBin(const char* path);
    void ImportMeshFromPly(const char* path);  

    // switch between different file input
    void ImportMesh(const char* path);
    
    // export tet mesh
    void ExportToOff(const char* path);
    // export surface mesh
    void ExportToPly(const char* path);

    void ExportLabeledSurf(const char* path);

    // return 0 if the mesh is successfully filled
    int HoleFilling();

    // transfer a surface mesh into tetgen object
    tetgenio ExportMeshToTetGen();

    // transfer a surface triangular mesh into a tet mesh
    // TODO: this part is integrated in ExportMeshToTetGen currently for better debug performance
    void CreateTetMesh();

    // TODO: Generate other kinds of mesh if needed
    // Mesh* CreateTriMesh(float size, float y=0.0f);
    // Mesh* CreateCubeMesh();
    // Mesh* CreateQuadMesh(float size, float y=0.0f);
    // Mesh* CreateDiscMesh(float radius, uint32_t segments);
    // Mesh* CreateSphere(int slices, int segments, float radius = 1.0f);
    // Mesh* CreateCapsule(int slices, int segments, float radius = 1.0f, float halfHeight = 1.0f);

    // save a mesh 
    void ExportMeshToBin(const char* path);

};

