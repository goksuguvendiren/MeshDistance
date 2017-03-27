//
// Created by Göksu Güvendiren on 23/03/2017.
//

#pragma once

#include <vector>
#include "glm/glm/vec3.hpp"

class Vertex
{
    glm::vec3 data;
    unsigned int id;

public:
    Vertex(glm::vec3 d = {0, 0, 0}) : data(d){}

    auto Data() const { return data; }
    auto ID() const { return id; }
};
