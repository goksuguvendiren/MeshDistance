#include <iostream>
#include "Mesh.h"

int main()
{
    Mesh mesh;
    mesh.LoadMesh("/Users/goksu/Documents/Geometry/inputs/geosedic/fprint_matrix/horse0.off");

//    for(auto asd : mesh.GetNeighbors(6)){
//        std::cerr << asd << '\n';
//    }

    mesh.GeodesicDistance(mesh.GetVertex(107));

    return 0;
}
