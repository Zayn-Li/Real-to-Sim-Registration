#include "DeformativeObject3D.h"
#include "platform.h"
// #include <pybind11/pybind11.h>
// #include <pybind11/stl.h>
// namespace py = pybind11;

#include <iostream>
#include <fstream>

// sort the particles in the longest axis
struct SweepAndPrune {
    struct Entry {
        Entry(Vec3 p, int i) : point(p), index(i) {}

        Vec3 point;
        int index;
    };

    SweepAndPrune(const Vec3* points, int n) {
        entries.reserve(n);
        for (int i = 0; i < n; ++i) entries.push_back(Entry(points[i], i));

        struct SortOnAxis {
            int axis;

            SortOnAxis(int axis) : axis(axis) {}

            bool operator()(const Entry& lhs, const Entry& rhs) const { return lhs.point[axis] < rhs.point[axis]; }
        };

        // calculate particle bounds and longest axis
        Vec3 lower(FLT_MAX), upper(-FLT_MAX);
        for (int i = 0; i < n; ++i) {
            lower = Min(points[i], lower);
            upper = Max(points[i], upper);
        }

        Vec3 edges = upper - lower;

        if (edges.x > edges.y && edges.x > edges.z)
            longestAxis = 0;
        else if (edges.y > edges.z)
            longestAxis = 1;
        else
            longestAxis = 2;

        std::sort(entries.begin(), entries.end(), SortOnAxis(longestAxis));

    }
    // find all particles in a specific ball (with center and radius)
    void QuerySphere(Vec3 center, float radius, std::vector<int>& indices) {
        // find start point in the array
        int low = 0;
        int high = int(entries.size());

        // the point we are trying to find
        float queryLower = center[longestAxis] - radius;
        float queryUpper = center[longestAxis] + radius;

        // binary search to find the start point in the sorted entries array
        while (low < high) {
            const int mid = (high + low) / 2;
            // std::cout << "HIGH - LOW - MID :" << high << "-" << low << "-" << mid << std::endl;
            if (queryLower > entries[mid].point[longestAxis])
                low = mid + 1;
            else
                high = mid;
        }

        // scan forward over potential overlaps
        float radiusSq = radius * radius;

        for (int i = low; i < int(entries.size()); ++i) {
            Vec3 p = entries[i].point;

            if (LengthSq(p - center) < radiusSq) {
                indices.push_back(entries[i].index);
            } else if (entries[i].point[longestAxis] > queryUpper) {
                // early out if ther are no more possible candidates
                break;
            }
        }
    }

    int longestAxis;  // [0, 1, 2] -> x,y,z

    std::vector<Entry> entries;
};

struct Seed {
    int index;
    float priority;

    bool operator<(const Seed& rhs) const { return priority < rhs.priority; }
};


void DeformativeObject3D::SetSkeleton(std::vector<uint32_t> ind)
{
    skeleton_ind = ind;
};

Vec3 DeformativeObject3D::CalculateMean(const Vec3* particles, const int* indices, int numIndices) {
    Vec3 sum;

    for (int i = 0; i < numIndices; ++i) sum += Vec3(particles[indices[i]]);

    if (numIndices)
        return sum / float(numIndices);
    else
        return sum;
}

float DeformativeObject3D::CalculateRadius(const Vec3* particles, Vec3 center, const int* indices, int numIndices) {
    float radiusSq = 0.0f;

    for (int i = 0; i < numIndices; ++i) {
        float dSq = LengthSq(Vec3(particles[indices[i]]) - center);
        if (dSq > radiusSq) radiusSq = dSq;
    }

    return sqrtf(radiusSq);
}


uint32_t DeformativeObject3D::CreateClusters(Vec3* particles, const float* priority, int numParticles, std::vector<int>& outClusterOffsets,
                   std::vector<int>& outClusterIndices, std::vector<Vec3>& outClusterPositions, Vec3 meshOffset, float radius = 0.5,
                   float smoothing = 0.0f) {
    std::vector<Seed> seeds;
    std::vector<Cluster> clusters;

    // flags a particle as belonging to at least one cluster
    std::vector<bool> used(numParticles, false);

    // initialize seeds
    for (int i = 0; i < numParticles; ++i) {
        Seed s;
        s.index = i;
        s.priority = priority[i];

        seeds.push_back(s);
    }

    // sort seeds on priority
    std::stable_sort(seeds.begin(), seeds.end());

    SweepAndPrune sap(particles, numParticles);
    std::vector<Vec3> centre;
    while (seeds.size()) {
        // pick highest unused particle from the seeds list
        Seed seed = seeds.back();
        seeds.pop_back();

        // if not being ever used
        if (!used[seed.index]) {
            Cluster c;

            sap.QuerySphere(Vec3(particles[seed.index]), radius, c.indices);

            // mark overlapping particles as used so they are removed from the list of potential cluster seeds
            for (int i = 0; i < int(c.indices.size()); ++i) used[c.indices[i]] = true;

            c.mean = CalculateMean(particles, &c.indices[0], int(c.indices.size()));
            centre.push_back( Vec3(particles[seed.index]) + meshOffset );

            clusters.push_back(c);
        }
    }

    if (smoothing > 0.0f) {
        for (int i = 0; i < int(clusters.size()); ++i) {
            Cluster& c = clusters[i];

            // clear cluster indices
            c.indices.resize(0);

            // calculate cluster particles using cluster mean and smoothing radius
            sap.QuerySphere(c.mean, smoothing, c.indices);

            c.mean = CalculateMean(particles, &c.indices[0], int(c.indices.size()));
        }
    }

    // write out cluster indices
    uint32_t count = 0;

    for (int c = 0; c < int(clusters.size()); ++c) {
        const Cluster& cluster = clusters[c];

        const int clusterSize = int(cluster.indices.size());

        // skip empty clusters
        if (clusterSize) {
            // write cluster indices
            for (int i = 0; i < clusterSize; ++i) outClusterIndices.push_back(cluster.indices[i]);

            // write cluster offset
            outClusterOffsets.push_back(int(outClusterIndices.size()));

            // write center
            outClusterPositions.push_back(centre[c]);

            ++count;
        }
    }

    return count;
}

void DeformativeObject3D::CreateLinks(){
    
    Vec2 links;
    bool duplicate;
    for (int i = 0; i< numParticles; i++){
        
        std::vector<int> Indices;
        std::vector<int> links_y;
        for(int j = 0; j < int(m_indices.size()); j++){
            if(int(m_indices[j]) == i){
                Indices.push_back(j);
            }
        }

        for(int k = 0; k < int(Indices.size()); k++){
            if(Indices[k]%4 == 0){
                duplicate = 0;
                if( m_indices[Indices[k]+1] > i){
                    for (int l = 0; l < int(links_y.size()); l++){
                        if (m_indices[Indices[k]+1] == links_y[l]) duplicate = 1;
                    }
                    if(!duplicate) 
                        links_y.push_back(m_indices[Indices[k]+1]);
                }

                duplicate = 0;
                if( m_indices[Indices[k]+2] > i){
                    for (int l = 0; l < int(links_y.size()); l++){
                        if (m_indices[Indices[k]+2] == links_y[l]) duplicate = 1;
                    }
                    if(!duplicate) 
                        links_y.push_back(m_indices[Indices[k]+2]);
                }

                duplicate = 0;
                if( m_indices[Indices[k]+3] > i){
                    for (int l = 0; l < int(links_y.size()); l++){
                        if (m_indices[Indices[k]+3] == links_y[l]) duplicate = 1;
                    }
                    if(!duplicate) 
                        links_y.push_back(m_indices[Indices[k]+3]);
                }
            }
            
            else if(Indices[k]%4 == 1){
                duplicate = 0;
                if( m_indices[Indices[k]-1] > i){
                    for (int l = 0; l < int(links_y.size()); l++){
                        if (m_indices[Indices[k]-1] == links_y[l]) duplicate = 1;
                    }
                    if(!duplicate) 
                        links_y.push_back(m_indices[Indices[k]-1]);
                }

                duplicate = 0;
                if( m_indices[Indices[k]+1] > i){
                    for (int l = 0; l < int(links_y.size()); l++){
                        if (m_indices[Indices[k]+1] == links_y[l]) duplicate = 1;
                    }
                    if(!duplicate) 
                        links_y.push_back(m_indices[Indices[k]+1]);
                }

                duplicate = 0;
                if( m_indices[Indices[k]+2] > i){
                    for (int l = 0; l < int(links_y.size()); l++){
                        if (m_indices[Indices[k]+2] == links_y[l]) duplicate = 1;
                    }
                    if(!duplicate) 
                        links_y.push_back(m_indices[Indices[k]+2]);
                }                         
            }
            
            else if(Indices[k]%4 == 2){
                duplicate = 0;
                if( m_indices[Indices[k]-2] > i){
                    for (int l = 0; l < int(links_y.size()); l++){
                        if (m_indices[Indices[k]-2] == links_y[l]) duplicate = 1;
                    }
                    if(!duplicate) 
                        links_y.push_back(m_indices[Indices[k]-2]);
                }

                duplicate = 0;
                if( m_indices[Indices[k]-1] > i){
                    for (int l = 0; l < int(links_y.size()); l++){
                        if (m_indices[Indices[k]-1] == links_y[l]) duplicate = 1;
                    }
                    if(!duplicate) 
                        links_y.push_back(m_indices[Indices[k]-1]);
                }

                duplicate = 0;
                if( m_indices[Indices[k]+1] > i){
                    for (int l = 0; l < int(links_y.size()); l++){
                        if (m_indices[Indices[k]+1] == links_y[l]) duplicate = 1;
                    }
                    if(!duplicate) 
                        links_y.push_back(m_indices[Indices[k]+1]);
                }

            }                         
            else if(Indices[k]%4 == 3){
                duplicate = 0;
                if( m_indices[Indices[k]-3] > i){
                    for (int l = 0; l < int(links_y.size()); l++){
                        if (m_indices[Indices[k]-3] == links_y[l]) duplicate = 1;
                    }
                    if(!duplicate) 
                        links_y.push_back(m_indices[Indices[k]-3]);
                }

                duplicate = 0;
                if( m_indices[Indices[k]-2] > i){
                    for (int l = 0; l < int(links_y.size()); l++){
                        if (m_indices[Indices[k]-2] == links_y[l]) duplicate = 1;
                    }
                    if(!duplicate) 
                        links_y.push_back(m_indices[Indices[k]-2]);
                }

                duplicate = 0;
                if( m_indices[Indices[k]-1] > i){
                    for (int l = 0; l < int(links_y.size()); l++){
                        if (m_indices[Indices[k]-1] == links_y[l]) duplicate = 1;
                    }
                    if(!duplicate) 
                        links_y.push_back(m_indices[Indices[k]-1]);
                }

            }         
        }
        
        for(int m = 0; m < int(links_y.size()); m++){
            links.x = i;
            links.y = links_y[m];
            m_links.push_back(links);
        }           
    }
}
void DeformativeObject3D::CreateDeformativeObject(float radius = 0.5, float smoothing = 0.0f)
{
    // introduce relative coordinates using the mean position of all particles 
    // the subtracted value will be added back later
    Vec3 meshOffset(0.0f);
    for (int i = 0; i < numParticles; i++) {
        meshOffset.x += m_positions[i].x;
        meshOffset.y += m_positions[i].y;
        meshOffset.z += m_positions[i].z;
    }
    meshOffset /= float(numParticles);

    Vec3* relativeParticles = new Vec3[numParticles];

    for (int i = 0; i < numParticles; i++) {
        Vec3 particles;
        particles.x = m_positions[i].x;
        particles.y = m_positions[i].y;
        particles.z = m_positions[i].z;
        relativeParticles[i] += particles - meshOffset;
    }

    // create particle sampling
    std::vector<Vec3> samples;

    // use priority to identify the order of constructing clusters (not used currently)
    std::vector<float> priority(numParticles);
    for (int i = 0; i < int(priority.size()); ++i) priority[i] = 0.0f;

    // cluster particles 
    numClusters = CreateClusters(relativeParticles, &priority[0], numParticles, clusterOffsets, clusterIndices,
                                     clusterPositions, meshOffset, radius, smoothing);

    std::cout << "[INFO]: numClusters =" << numClusters << std::endl;

    // create links (springs) between particles
    
    // CreateLinks();

    // Switch back to absolute coordinates by adding meshOffset to the centers of mass and to each particle positions

    for (int i = 0; i < numClusters; ++i) {
        clusterPositions[i] += meshOffset;
    }

}

void DeformativeObject3D::ExportDeformatives(const char* path)
{
	std::ofstream file(path);

    if (!file)
        return;

	// file << "PARTICLES" << " " << numParticles << " " << "TETS" << " " << numTet << " " << "CLUSTERS" << " " << clusterOffsets.size() << " " 
    // << "CLUSTERID" << " " << clusterIndices.size() << " " << "LINKS" << " " << m_links.size() << std::endl;
    // file << "PARTICLES" << " " << numParticles << std::endl;
    // for (uint32_t i=0; i < numParticles; ++i)
	// {		
    //     Point3 v = m_positions[i];
	// 	file  << v.x << " " << v.y << " " << v.z << std::endl;
	// }

    // file << "TETS" << " " << numTet << std::endl;
	// for (uint32_t i=0; i < numTet; ++i)
	// {
		
	// 	file << "4" << " " << m_indices[i*4] << " " << m_indices[i*4+1] << " " << m_indices[i*4+2] << " " << m_indices[i*4+3] << std::endl;
	// }
    
    file << "CLUSTEROFFSETS" << " " << clusterOffsets.size() << std::endl;
	for (uint32_t i=0; i < clusterOffsets.size(); ++i)
	{
		
        file << clusterOffsets[i] << std::endl;
	}    
    
    file << "CLUSTERINDICES" << " " << clusterIndices.size() << std::endl;
	for (uint32_t i=0; i < clusterIndices.size(); ++i)
	{
		
        file << clusterIndices[i] << std::endl;
	}
    
    // file << "LINKS" << " " << m_links.size() << std::endl;
	// for (uint32_t i=0; i < m_links.size(); ++i)
	// {
	// 	Vec2 l = m_links[i];
    //     file << l.x << " " << l.y << std::endl;
	// }      
    // file << "CLUSTERPOSITIONS" << " " << clusterIndices.size() << std::endl;
	// for (uint32_t i=0; i < clusterPositions.size(); ++i)
	// {
		
    //     file << clusterPositions[i][0] << " " << clusterPositions[i][1] << " " << clusterPositions[i][2] << std::endl;
	// }    
}

// pybind11

// PYBIND11_MODULE(deformobj, m) {
//     // deformative object class
//     py::class_<DeformativeObject3D>(m, "DeformativeObject3D")
//         .def(py::init<>())
//         .def("importmesh", &DeformativeObject3D::ImportMesh, "import mesh from a .ply, .bin or .pcd file", py::arg("path") = nullptr)
//         .def("tetgen", &DeformativeObject3D::ExportMeshToTetGen, "construct the tet mesh of a closed surface mesh")
//         .def("tetgen", &DeformativeObject3D::CreateDeformativeObject, "calculate the features of a deformative object")
//         .def_readwrite("positions", &DeformativeObject3D::m_positions)
//         .def_readwrite("indices", &DeformativeObject3D::m_indices)
//         .def_readwrite("normals", &DeformativeObject3D::m_normals)
//         .def_readwrite("numVertices", &DeformativeObject3D::numVertices)
//         .def_readwrite("numFaces", &DeformativeObject3D::numFaces)
//         .def_readwrite("numParticles", &DeformativeObject3D::numParticles)
//         .def_readwrite("numTet", &DeformativeObject3D::numTet)
//         .def_readwrite("numClusters", &DeformativeObject3D::numClusters)
//         .def_readwrite("clusterIndices", &DeformativeObject3D::clusterIndices)
//         .def_readwrite("clusterOffsets", &DeformativeObject3D::clusterOffsets)
//         .def_readwrite("links", &DeformativeObject3D::m_links);
// }