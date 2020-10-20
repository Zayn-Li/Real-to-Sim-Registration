#include<iostream>
#include<string>
#include<fstream>
#include "DeformativeObject3D.h"
int main()
{
    // std::cout << "good" << std::endl;  
    DeformativeObject3D tet_pt;
    // std::cout << "good" << std::endl;  
    tet_pt.ImportMesh("../../test_pc/initial.ply");
    // std::cout << "good" << std::endl;
    tet_pt.numParticles = tet_pt.numVertices;
    // std::cout << "good" << std::endl;    
    tet_pt.numTet = tet_pt.numFaces;
    // std::cout << "good" << std::endl;  
    tet_pt.CreateDeformativeObject(0.01, 0);
    // std::cout << "good" << std::endl;  
    tet_pt.ExportDeformatives("../../cluster_0.010.txt");
    return 1;
}