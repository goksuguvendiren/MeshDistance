#include <iostream>
#include "Mesh.h"

int main()
{
    Mesh m;
    m.LoadMesh("/Users/goksu/Documents/Geometry/inputs/geosedic/fprint_matrix/horse0.off");

    std::cerr << m.GetMesh()->mNumVertices << '\n';


    std::cout << "Hello, World!" << std::endl;
    return 0;
}