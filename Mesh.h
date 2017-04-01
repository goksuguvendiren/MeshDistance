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

    std::map<int, std::vector<int>> neighbors;

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
    
    int NumVertices() { return numVertices; }

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


    std::vector<glm::vec3> GetVertices() const
    {
        std::vector<glm::vec3> data;

        data.resize(numVertices);
        std::transform(vertices.begin(), vertices.end(), data.begin(), [](const Vertex& vertex){
            return vertex.Data();
        });

        return data;
    }

    void Load(const std::string& path);

    const Vertex& GetVertex(unsigned int id) const { return vertices[id]; }

    
    std::vector<std::pair<float, unsigned int>> GeodesicDistance(const Vertex& vert);
    std::vector<glm::vec3> ShortestPath(const Vertex &seed, std::vector<std::pair<float, unsigned int>>&& costs);
    std::vector<std::vector<std::pair<float, unsigned int>>> GenerateDistanceMap();
    
    auto FindMin(const std::vector<std::pair<float, unsigned int>>& costs, const std::vector<bool>& beenProcessed) const;
    
    void GeodesicDescriptor(const Vertex& vert, int k, boost::optional<std::vector<std::pair<float, unsigned int>>> costs);

    const std::vector<int>& GetNeighbors(unsigned int id) { return neighbors[id]; }
};
