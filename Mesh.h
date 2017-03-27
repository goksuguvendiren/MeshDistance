//
// Created by Göksu Güvendiren on 06/03/2017.
//

#pragma once

#include <map>
#include <vector>
#include <scene.h>
#include "Triangle.h"
#include "assimp/Importer.hpp"

enum class DataStructure
{
    Array,
    MinHeap,
    FibHeap
};

class Mesh
{
    int id;
    unsigned int numVertices;
    unsigned int numFaces;

    const aiMesh* mesh;

    std::vector<Triangle> faces;
    std::vector<Vertex> vertices;

    Assimp::Importer importer;

public:
    Mesh(int mid = 1) : id(mid), mesh(nullptr) {}
    Mesh(const Mesh& m) = delete;
    Mesh(Mesh&& m) = default;


    void AddVertex(Vertex&& vert) {
        vertices.push_back(std::move(vert));
    }

    void AddFace(Triangle&& face) {
        faces.push_back(std::move(face));
    }

    int ID() const { return id; }
    const std::vector<Triangle>& Faces() const { return faces; }

    void LoadMesh(const std::string& path);
    auto GetMesh() { return mesh; }

    auto GetVertex(unsigned int id) const { return vertices[id]; }

    void GeodesicDistance(const Vertex& vertex);
    auto FindMin(const std::vector<std::pair<int, unsigned int>>& costs);

    auto GetNeighbors(unsigned int id) { return std::vector<Vertex>(); }
};