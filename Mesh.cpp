//
// Created by Göksu Güvendiren on 23/03/2017.
//

#include <iostream>
#include <fstream>
#include <algorithm>
#include "Mesh.h"
//#include "assimp/Importer.hpp"
//#include "assimp/scene.h"
//#include "assimp/postprocess.h"

void Mesh::LoadMesh(const std::string& path)
{
    std::ifstream stream;
    stream.open(path);

    std::string filetype; stream >> filetype;

    std::cerr << filetype << '\n';

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

auto Mesh::FindMin(const std::vector<std::pair<float, unsigned int>>& costs, const std::vector<bool>& beenProcessed) const
{
    auto min = std::numeric_limits<unsigned int>::max();
    auto vert = GetVertex(0);

    for (int i = 0; i < costs.size(); i++){
        if (!beenProcessed[i] && costs[i].first < min){
            min = costs[i].first;
            vert = GetVertex(i);
        }
    }

    return vert;
}

void Mesh::GeodesicDistance(const Vertex& vert)
{
    // cost - parent vertex association. parent == -1, means that this node has not been processed.
    std::vector<std::pair<float, unsigned int>> costs (numVertices, std::make_pair(std::numeric_limits<unsigned int>::max(), -1)); // cost - parent pairs

    std::vector<bool> beenProcessed(numVertices, false);
    int numProcessed = 0;
    
    int cost;
    
    costs[vert.ID()].first = 0;
    costs[vert.ID()].second = 0;

    while(numProcessed != numVertices){
        auto node = FindMin(costs, beenProcessed);
        cost = costs[node.ID()].first;

        auto neighs = GetNeighbors(node.ID());
        for (auto& n : neighs){
            auto& neighbor = GetVertex(n);
            
            auto distance = node.EuclideanDistance(neighbor);
            
            if (costs[neighbor.ID()].first > cost + distance){ // assume weight of each vertex to be 1
                costs[neighbor.ID()].first = cost + distance;
                costs[neighbor.ID()].second = node.ID();
            }
        }
        beenProcessed[node.ID()] = true;
        numProcessed += 1;
    }
    
    float max = 0.f;
    for_each(costs.begin(), costs.end(), [&max](auto& cost){
        if (cost.first > max) max = cost.first;
    });
    std::cerr << "Finished calculating : " << max << "\n";
}

//    auto scene = importer.ReadFile(path, aiProcess_Triangulate);
//
//    if (!scene){
//        std::cerr << "Could not load scene" << '\n';
//        std::cerr << importer.GetErrorString() << '\n';
//        std::abort();
//    }
//
//    mesh = scene->mMeshes[0];
//
//    numVertices = mesh->mNumVertices;
//    numFaces = mesh->mNumFaces;
//
//    vertices.reserve(numVertices);
//    faces.reserve(numFaces);
//
//    for (auto i = 0; i < mesh->mNumVertices; i++){
//        const aiVector3D* pos = &(mesh->mVertices[i]);
//        Vertex v({pos->x, pos->y, pos->z});
//        AddVertex(std::move(v));
//    }
//
//    for (unsigned int i = 0 ; i < mesh->mNumFaces ; i++) {
//        const aiFace& Face = mesh->mFaces[i];
//        assert(Face.mNumIndices == 3);
//        Triangle t(Face.mIndices[0], Face.mIndices[1], Face.mIndices[2], *this, i);
//        AddFace(std::move(t));
//    }
