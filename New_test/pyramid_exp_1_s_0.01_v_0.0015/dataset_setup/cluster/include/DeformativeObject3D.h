#pragma once

#include "mesh.h"

class DeformativeObject3D : public Mesh
{
public:
    struct Cluster {
        Vec3 mean;
        float radius;

        // indices of particles belonging to this cluster
        std::vector<int> indices;
    };
    
    // TODO: we can distribute memory adaptive using the resize function defined in vector (the same as m_positions)
    std::vector<Cluster> m_clusters;
    // number of clusters
    uint32_t  numClusters;

    std::vector<int> clusterIndices;
    std::vector<int> clusterOffsets;
    std::vector<Vec3> clusterPositions;
    // 2*d dimension vector with indices of particle in each link (spring), d links in total.
    std::vector<Vec2> m_links;

    // indices of particles which construct rigid skeleton.
    std::vector<uint32_t> skeleton_ind;

    DeformativeObject3D(){};
    ~DeformativeObject3D(){};
    // pick the skeleton points manually
    void SetSkeleton(std::vector<uint32_t> ind);

    // move a specific particle without destorying the tet structure
    void MoveParticle(uint32_t i,std::vector<Point3> goal_pos);

    Vec3 CalculateMean(const Vec3* particles, const int* indices, int numIndices);
    
    float CalculateRadius(const Vec3* particles, Vec3 center, const int* indices, int numIndices);
    
    void CreateLinks();

    uint32_t CreateClusters(Vec3* particles, const float* priority, int numParticles, std::vector<int>& outClusterOffsets,
                   std::vector<int>& outClusterIndices, std::vector<Vec3>& outClusterPositions, Vec3 meshOffset, float radius ,
                   float smoothing);
    void CreateDeformativeObject(float radius, float smoothing);
    void ExportDeformatives(const char* path);
    // void AddTools();




};