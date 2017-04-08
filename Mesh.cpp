//
// Created by Göksu Güvendiren on 23/03/2017.
//

#include <iostream>
#include <fstream>
#include <boost/heap/fibonacci_heap.hpp>

#include "Mesh.h"

void Mesh::Load(const std::string& path)
{
    std::ifstream stream;
    stream.open(path);

    std::string filetype; stream >> filetype;

    if (filetype == "OFF"){
        loadOFF(stream);
    }
    else if (filetype == "# OBJ"){
        loadOBJ(stream);
    }
}

void Mesh::loadOFF(std::ifstream& stream)
{
    int nullval;
    stream >> numVertices; stream >> numFaces; stream >> nullval;

    vertices.reserve(numVertices);
    faces.reserve(numFaces);

    for (int i = 0; i < numVertices; i++){
        float x, y, z;
        stream >> x; stream >> y; stream >> z;
        vertices.push_back(Vertex({x, y, z}, i));
    }

    for (int i = 0; i < numFaces; i++){
        int numVert; stream >> numVert;
        assert(numVert == 3);

        int a, b, c;
        stream >> a; stream >> b; stream >> c;

        addNeighbor(a, b);
        addNeighbor(a, c);
        
        addNeighbor(b, a);
        addNeighbor(b, c);
        
        addNeighbor(c, a);
        addNeighbor(c, b);
        
        faces.push_back(Triangle(vertices[a], vertices[b], vertices[c], i));
    }
}

void Mesh::loadOBJ(std::ifstream& stream)
{
    std::cerr << "loading obj " << '\n';
}
