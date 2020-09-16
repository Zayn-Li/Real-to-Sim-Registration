#pragma once

#include <vector>

#include "core.h"
#include "maths.h"


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

    void ImportMeshFromPly(const char* path);  


    // export surface mesh
    void ExportToPly(const char* path);

};

