#include "mesh.h"
#include "tetgen.h"
#include "DeformativeObject3D.h"

int main(int argc, char *argv[])
{
    Mesh test;
    test.CreateVolumeMesh("../../test_pc/initial.ply","../..");
}