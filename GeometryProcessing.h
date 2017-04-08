//
// Created by Göksu Güvendiren on 06/04/2017.
//

#pragma once

#include <vector>
#include <boost/optional.hpp>
#include "glm/vec3.hpp"

class Vertex;
class Mesh;

class GeometryProcessing
{
    static Vertex FindMin(const std::vector<Vertex>& vertices, const std::vector<std::pair<float, unsigned int>>& costs, const std::vector<bool>& beenProcessed);

public:
    static std::vector<std::pair<float, unsigned int>> GeodesicDistance_Array(const Mesh& mesh, unsigned int id);
    static std::vector<std::pair<float, unsigned int>> GeodesicDistance_MinHeap(const Mesh& mesh, unsigned int id);
    static std::vector<std::pair<float, unsigned int>> GeodesicDistance_FibHeap(const Mesh& mesh, unsigned int id);
    static std::vector<std::vector<std::pair<float, unsigned int>>> GenerateDistanceMap(const Mesh& mesh);

    static std::vector<float> GeodesicDescriptor(const Mesh& mesh, unsigned int id, int k,
                                   const std::vector<std::pair<float, unsigned int>>& costs);

    static void BilateralMap(const Mesh& mesh, unsigned int id1, unsigned int id2,
                             const std::vector<std::vector<std::pair<float, unsigned int>>>& distancemap);

    static std::vector<Vertex> GeneratePath(const Mesh& mesh, unsigned int id1, unsigned int id2,
                                               const std::vector<std::pair<float, unsigned int>>& costs);
    static float Distance(unsigned int vertex, const std::vector<Vertex>& path,
                          const std::vector<std::vector<std::pair<float, unsigned int>>>& distancemap);
    static float Distance(unsigned int vertex1, unsigned int vertex2,
                          const std::vector<std::vector<std::pair<float, unsigned int>>>& distancemap);
    static float Distance(unsigned int vertex, const std::vector<std::pair<float, unsigned int>>& costs);
};
