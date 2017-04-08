//
// Created by Göksu Güvendiren on 06/04/2017.
//

#include <iostream>
#include <algorithm>
#include <boost/heap/fibonacci_heap.hpp>
#include "GeometryProcessing.h"
#include "Mesh.h"

bool intersect(const Triangle& tri)
{
    return false;
}

auto lerp(float alpha, glm::vec3 v0, glm::vec3 vi)
{
    return ((1 - alpha) * v0) + (alpha * vi);
}

float alpha(float radius, const Vertex& seed, const Vertex& v0, const Vertex& vi, const std::vector<std::pair<float, unsigned int>>& costs)
{
    return (std::abs(radius - GeometryProcessing::Distance(v0.ID(), costs))) /
           (std::abs(GeometryProcessing::Distance(vi.ID(), costs) - GeometryProcessing::Distance(v0.ID(), costs)));

}

float CalculateGeodesicCurveLength(const Mesh& mesh, unsigned int seed, float radius, const std::vector<std::pair<float, unsigned int>>& costs)
{
    float curveLen = 0.0f;
    for (auto& face : mesh.GetTriangles()){
        auto v0 = face.PointA();
        auto v1 = face.PointB();
        auto v2 = face.PointC();

        auto alpha1 = alpha(radius, mesh.GetVertex(seed), v0, v1, costs);
        auto alpha2 = alpha(radius, mesh.GetVertex(seed), v0, v2, costs);

        if (alpha1 < 0 || alpha1 > 1) continue;
        if (alpha2 < 0 || alpha2 > 1) continue;

        curveLen += glm::length(lerp(alpha1, v0.Data(), v1.Data()) - lerp(alpha2, v0.Data(), v2.Data()));
    }

    return curveLen;
}

std::vector<float> GeometryProcessing::GeodesicDescriptor(const Mesh& mesh, unsigned int seed, int k, const std::vector<std::pair<float, unsigned int>>& costs)
{
    // Find the maximum distance to the seed vertex
    float maxDist = 0;
    for (int i = 0; i < mesh.NumVertices(); i++){
        auto distance = costs[i].first;
        maxDist = std::max(maxDist, distance);
    }

    float deltaRadius = maxDist / k;

    // Create the histogram
    std::vector<float> histogram(k, 0);

    for (int i = 1; i <= k; i++) {
        float radius = i * deltaRadius;

        auto len = CalculateGeodesicCurveLength(mesh, seed, radius, costs);
        std::cerr << "Length of " << i << "th iso-curve is : " << len << '\n';

        histogram[i-1] = len;
    }

    return histogram;
}

std::vector<Vertex> GeometryProcessing::GeneratePath(const Mesh &mesh, unsigned int id1, unsigned int id2,
                                                     const std::vector<std::pair<float, unsigned int>>& costs)
{
    std::vector<Vertex> path;
    auto node = id2;
    while(node != id1){
        path.push_back(mesh.GetVertex(node));
        node = costs[node].second;
    }
    path.push_back(mesh.GetVertex(id1));

    return path;
}


float GeometryProcessing::Distance(unsigned int vertex1, unsigned int vertex2,
                                   const std::vector<std::vector<std::pair<float, unsigned int>>>& distancemap)
{
    return distancemap[vertex1][vertex2].first;
}

float GeometryProcessing::Distance(unsigned int vertex, const std::vector<std::pair<float, unsigned int>>& costs)
{
    return costs[vertex].first;
}

float GeometryProcessing::Distance(unsigned int vertex, const std::vector<Vertex>& path,
                                   const std::vector<std::vector<std::pair<float, unsigned int>>>& distancemap)
{
    float minDistance = std::numeric_limits<float>::infinity();
    auto minV = path[0];

    for (auto& p_vert : path){
        float dist;
        if ((dist = Distance(vertex, p_vert.ID(), distancemap)) < minDistance){
            minDistance = dist;
            minV = p_vert;
        }
    }

    return minDistance;
}

void GeometryProcessing::BilateralMap(const Mesh& mesh, unsigned int id1, unsigned int id2,
                                      const std::vector<std::vector<std::pair<float, unsigned int>>>& distancemap)
{
    auto costs = distancemap[id1];

    auto path = GeneratePath(mesh, id1, id2, costs);
    auto distance = costs[id2].first;
    auto theta = distance / 3;

    // Color the ROI :
    std::vector<Vertex> ROI_vertices;
    for (auto& vertex : mesh.GetVertices()){
        float distanceToPath = Distance(vertex.ID(), path, distancemap);
        float distanceToVer1 = Distance(vertex.ID(), id1, distancemap);
        float distanceToVer2 = Distance(vertex.ID(), id2, distancemap);

        if (distanceToPath <= theta &&
            distanceToVer1 <= distance &&
            distanceToVer2 <= distance) {
            ROI_vertices.push_back(vertex);
        }
    }

    // Divide the ROI region to subregions (choose the seed vertex as id1) !!
    int k = 16;
    float maxDist = distance;

    // Create the histogram
    float bucketSize = maxDist / k;
    std::vector<int> histogram(k, 0);
    // TODO : Calculate Triangle Areas



    for (auto& roi_v : ROI_vertices){
        int index = costs[roi_v.ID()].first / bucketSize;
        histogram[index]++;
    }
}

void UpdateProgress(float progress)
{
    int barWidth = 70;

    std::cout << "[";
    int pos = barWidth * progress;
    for (int i = 0; i < barWidth; ++i) {
        if (i < pos) std::cout << "=";
        else if (i == pos) std::cout << ">";
        else std::cout << " ";
    }
    std::cout << "] " << int(progress * 100.0) << " %\r";
    std::cout.flush();
}

std::vector<std::vector<std::pair<float, unsigned int>>> GeometryProcessing::GenerateDistanceMap(const Mesh& mesh)
{
    std::vector<std::vector<std::pair<float, unsigned int>>> allCosts;
    allCosts.reserve(mesh.NumVertices());

    for (int i = 0; i < mesh.NumVertices(); i++){
//    for (int i = 0; i < std::min(mesh.NumVertices(), 100u); i++){
        allCosts.push_back(std::move(GeodesicDistance_FibHeap(mesh, i)));
        UpdateProgress(i / mesh.NumVertices());
    }

    return allCosts;
}

struct HeapData
{
    float cost;
    unsigned int id;
    unsigned int parent;

    HeapData(float c, unsigned int i, unsigned int p) : cost(c), id(i), parent(p) {}
};

std::vector<std::pair<float, unsigned int>> GeometryProcessing::GeodesicDistance_FibHeap(const Mesh& mesh, unsigned int id)
{
    auto vert = mesh.GetVertex(id);

    std::vector<std::pair<float, unsigned int>> costs(mesh.NumVertices());
    std::vector<bool> beenProcessed(mesh.NumVertices(), false);

    auto x = [](auto& cost1, auto& cost2){
        return cost1.cost > cost2.cost;
    };

    std::vector<boost::heap::fibonacci_heap<HeapData, boost::heap::compare<decltype(x)>>::handle_type> handles(mesh.NumVertices());
    boost::heap::fibonacci_heap<HeapData, boost::heap::compare<decltype(x)>> fibHeap(x);

    for (unsigned int i = 0; i < mesh.NumVertices(); i++) {
        auto handle = fibHeap.push(HeapData(std::numeric_limits<float>::max(), i, -1));
        handles[i] = handle;
    }

    fibHeap.update(handles[vert.ID()], HeapData(0, vert.ID(), 0));

    while(!fibHeap.empty()){
        const auto node = fibHeap.top();
        fibHeap.pop();

        costs[node.id] = std::make_pair(node.cost, node.parent);
        beenProcessed[node.id] = true;

        auto& vertex = mesh.GetVertex(node.id);
        auto neighs = mesh.GetNeighbors(node.id);

        for (auto& n : neighs){
            if (beenProcessed[n]) continue;

            auto& neighbor = mesh.GetVertex(n);
            auto distance = glm::length(vertex.Data() - neighbor.Data());

            auto hand = *handles[n];
            if (hand.cost > node.cost + distance){
                fibHeap.decrease(handles[n], HeapData(node.cost + distance, hand.id, node.id));
            }
        }
    }

    return costs;
}


Vertex GeometryProcessing::FindMin(const std::vector<Vertex>& vertices, const std::vector<std::pair<float, unsigned int>>& costs, const std::vector<bool>& beenProcessed)
{
    auto min = std::numeric_limits<float>::max();
    auto vert = vertices[0];

    for (int i = 0; i < costs.size(); i++){
        if (!beenProcessed[i] && costs[i].first < min){
            min = costs[i].first;
            vert = vertices[i];
        }
    }

    return vert;
}

std::vector<std::pair<float, unsigned int>> GeometryProcessing::GeodesicDistance_Array(const Mesh& mesh, unsigned int id)
{
    auto vert = mesh.GetVertex(id);
    // costs : cost - parent vertex association. parent == -1, means that this node has not been processed.
    std::vector<std::pair<float, unsigned int>> costs (mesh.NumVertices(), std::make_pair(std::numeric_limits<float>::max(), -1));

    std::vector<bool> beenProcessed(mesh.NumVertices(), false);
    int numProcessed = 0;

    costs[vert.ID()].first = 0;
    costs[vert.ID()].second = 0;

    while(numProcessed != mesh.NumVertices()){
        auto node = FindMin(mesh.GetVertices(), costs, beenProcessed);
        float cost = costs[node.ID()].first;

        auto neighs = mesh.GetNeighbors(node.ID());
        for (auto& n : neighs){
            auto& neighbor = mesh.GetVertex(n);

            auto distance = glm::length(node.Data() - neighbor.Data());

            if (costs[neighbor.ID()].first > cost + distance){ // assume weight of each vertex to be 1
                assert(!beenProcessed[neighbor.ID()]);
                costs[neighbor.ID()].first = cost + distance;
                costs[neighbor.ID()].second = node.ID();
            }
        }
        beenProcessed[node.ID()] = true;
        numProcessed += 1;
    }
    return costs;
}
