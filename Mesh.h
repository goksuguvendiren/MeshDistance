//
// Created by Göksu Güvendiren on 06/03/2017.
//

#pragma once

#include <map>
#include <vector>
#include "Triangle.h"

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

    std::vector<Triangle> faces;
    std::vector<Vertex> vertices;

    std::map<int, std::vector<int>> neighbors;

    void loadOFF(std::ifstream& stream);
    void loadOBJ(std::ifstream& stream);

public:
    Mesh(int mid = 1) : id(mid){}//, mesh(nullptr) {}
    Mesh(const Mesh& m) = delete;
    Mesh(Mesh&& m) = default;

    void addNeighbor(int id, int neigh)
    {
        if (std::find(neighbors[id].begin(), neighbors[id].end(), neigh) == neighbors[id].end())
            neighbors[id].push_back(neigh);
    }
    void AddVertex(Vertex&& vert) {
        vertices.push_back(std::move(vert));
    }

    void AddFace(Triangle&& face) {
        faces.push_back(std::move(face));
    }
    
    int NumVertices() { return numVertices; }

    int ID() const { return id; }
    const std::vector<Triangle>& Faces() const { return faces; }

    void LoadMesh(const std::string& path);
//    auto GetMesh() { return mesh; }

    const Vertex& GetVertex(unsigned int id) const { return vertices[id]; }

    void GeodesicDistance(const Vertex& vertex);
    auto FindMin(const std::vector<std::pair<float, unsigned int>>& costs, const std::vector<bool>& beenProcessed) const;

    const std::vector<int>& GetNeighbors(unsigned int id) { return neighbors[id]; }
};
