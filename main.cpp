#include <iostream>
#include "Mesh.h"
#include <iomanip>

int main()
{
    Mesh mesh;
    mesh.LoadMesh("/Users/goksu/Documents/Geometry/inputs/geosedic/fprint_matrix/horse0.off");

//    for(auto asd : mesh.GetNeighbors(6)){
//        std::cerr << asd << '\n';
//    }

    auto start = std::chrono::steady_clock::now();
    for (int i = 0; i < mesh.NumVertices() && i < 1000; i++){
        mesh.GeodesicDistance(mesh.GetVertex(i));
    }
    auto dist = std::chrono::steady_clock::now();
    std::cerr << "Loading took "
    << std::chrono::duration_cast<std::chrono::milliseconds>(dist - start).count()
    << "ms.\n";
    return 0;
}
