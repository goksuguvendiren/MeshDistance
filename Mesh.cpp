//
// Created by Göksu Güvendiren on 23/03/2017.
//

#include <iostream>
#include <fstream>
#include "Mesh.h"
#include "assimp/Importer.hpp"
#include "assimp/scene.h"
#include "assimp/postprocess.h"

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
    int numVertices, numFaces, nullval;
    stream >> numVertices; stream >> numFaces; stream >> nullval;

    vertices.reserve(numVertices);
    faces.reserve(numFaces);

    for (int i = 0; i < numVertices; i++){
        float x, y, z;
        stream >> x; stream >> y; stream >> z;
        vertices.push_back(Vertex({x, y, z}));
    }

    for (int i = 0; i < numFaces; i++){
        int numVert; stream >> numVert;
        assert(numVert == 3);

        int a, b, c;
        stream >> a; stream >> b; stream >> c;

        neighbors[a].push_back(b);
        neighbors[a].push_back(c);

        neighbors[b].push_back(a);
        neighbors[b].push_back(c);

        neighbors[c].push_back(a);
        neighbors[c].push_back(b);

        faces.push_back(Triangle(vertices[a], vertices[b], vertices[c], i));
    }
}

void Mesh::loadOBJ(std::ifstream& stream)
{
    std::cerr << "loading obj " << '\n';
}

auto Mesh::FindMin(const std::vector<std::pair<int, unsigned int>>& costs)
{
    auto min = std::numeric_limits<int>::infinity();
    Vertex vert;

    for (auto cost : costs){
        if (cost.first < min){
            min = cost.first;
            vert = GetVertex(cost.second);
        }
    }

    return vert;
}

void Mesh::GeodesicDistance(const Vertex& vertex) const
{
    std::vector<std::pair<int, unsigned int>> costs (numVertices, std::make_pair(std::numeric_limits<int>::infinity(), 0)); // cost - parent pairs

    int cost = 0;

    auto& node = vertex;
    costs[node.ID()].first = 0;
    costs[node.ID()].second = node.ID();

    bool finished = false;
    while(!finished){
        auto node = FindMin(costs);
//        auto& node = vertex;
        cost = costs[node.ID()].first;

        auto neighbors = GetNeighbors(node.ID()); //TODO : returns an empty list !! Be careful !
        for (auto& neighbor : neighbors){
            if (costs[neighbor.ID()].first > cost + 1){ // assume weight of each vertex to be 1
                costs[neighbor.ID()].first = cost + 1;
                costs[neighbor.ID()].second = node.ID();
            }
        }
    }
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