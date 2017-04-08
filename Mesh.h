//
// Created by Göksu Güvendiren on 06/03/2017.
//

#pragma once

#include <map>
#include <vector>
#include <boost/optional.hpp>
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

    std::map<unsigned int, std::vector<int>> neighbors;

    void loadOFF(std::ifstream& stream);
    void loadOBJ(std::ifstream& stream);

public:
    Mesh(int mid = 1) : id(mid) {}
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
    
    unsigned int NumVertices() const { return numVertices; }

    int ID() const { return id; }

    std::vector<unsigned int> GetFaces() const
    {
        std::vector<unsigned int> data;
        data.reserve(numFaces * 3);

        std::for_each(faces.begin(), faces.end(), [&data](const Triangle& tri){
            data.push_back(tri.PointA().ID());
            data.push_back(tri.PointB().ID());
            data.push_back(tri.PointC().ID());
        });

        return data;
    }

    const std::vector<Triangle>& GetTriangles() const { return faces; }

    std::vector<glm::vec3> GetVertexData() const
    {
        std::vector<glm::vec3> data;

        data.resize(numVertices);
        std::transform(vertices.begin(), vertices.end(), data.begin(), [](const Vertex& vertex){
            return vertex.Data();
        });

        return data;
    }

    const std::vector<Vertex>& GetVertices() const { return vertices; }

    void Load(const std::string& path);

    const Vertex& GetVertex(unsigned int id) const { return vertices[id]; }

    const std::vector<int>& GetNeighbors(unsigned int id) const { return neighbors.find(id)->second; }
};
