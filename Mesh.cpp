//
// Created by Göksu Güvendiren on 23/03/2017.
//

#include <iostream>
#include <fstream>
#include <algorithm>
#include "Mesh.h"

void Mesh::Load(const std::string& path)
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

std::vector<glm::vec3> Mesh::ShortestPath(const Vertex &seed, std::vector<std::pair<float, unsigned int>>&& costs)
{
    std::sort(costs.begin(), costs.end(), [](auto& cost1, auto& cost2){
        return cost1.first < cost2.first;
    });

    std::vector<glm::vec3> path;
    std::transform(costs.begin(), costs.end(), path.begin(), [this](auto& cost){
        return GetVertex(cost.second).Data();
    });

    return path;
}

auto Mesh::FindMin(const std::vector<std::pair<float, unsigned int>>& costs, const std::vector<bool>& beenProcessed) const
{
    auto min = std::numeric_limits<float>::max();
    auto vert = GetVertex(0);

    for (int i = 0; i < costs.size(); i++){
        if (!beenProcessed[i] && costs[i].first < min){
            min = costs[i].first;
            vert = GetVertex(i);
        }
    }

    return vert;
}

std::vector<std::pair<float, unsigned int>> Mesh::GeodesicDistance(const Vertex& vert)
{
    // costs : cost - parent vertex association. parent == -1, means that this node has not been processed.
    std::vector<std::pair<float, unsigned int>> costs (numVertices, std::make_pair(std::numeric_limits<float>::max(), -1));;
    
    std::vector<bool> beenProcessed(numVertices, false);
    int numProcessed = 0;
    
    costs[vert.ID()].first = 0;
    costs[vert.ID()].second = 0;

    while(numProcessed != numVertices){
        auto node = FindMin(costs, beenProcessed);
        float cost = costs[node.ID()].first;

        auto neighs = GetNeighbors(node.ID());
        for (auto& n : neighs){
            auto& neighbor = GetVertex(n);
            
            auto distance = glm::length(node.Data() - neighbor.Data());
            
            if (costs[neighbor.ID()].first > cost + distance){ // assume weight of each vertex to be 1
                costs[neighbor.ID()].first = cost + distance;
                costs[neighbor.ID()].second = node.ID();
            }
        }
        beenProcessed[node.ID()] = true;
        numProcessed += 1;
    }
    return costs;
}

std::vector<std::vector<std::pair<float, unsigned int>>> Mesh::GenerateDistanceMap()
{
    std::vector<std::vector<std::pair<float, unsigned int>>> allCosts;
    allCosts.reserve(numVertices);

    for (int i = 0; i < NumVertices(); i++){
        allCosts.push_back(GeodesicDistance(GetVertex(i)));
    }
    
    return allCosts;
}

void Mesh::GeodesicDescriptor(const Vertex& vert, int k, boost::optional<std::vector<std::pair<float, unsigned int>>> costs)
{
    if (!costs) costs = GeodesicDistance(vert);

    // Find the maximum distance to the seed vertex
    float maxDist = 0;
    for (int i = 0; i < numVertices; i++){
        auto distance = (*costs)[i].first;
        maxDist = std::max(maxDist, distance);
    }

    float stepSize = maxDist / k;

    std::vector<int> histogram(k, 0);

    // Create the histogram
    for (int i = 0; i < numVertices; i++){
        auto distance = (*costs)[i].first;
        histogram[distance / stepSize]++;
    }

}