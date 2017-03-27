//
// Created by Göksu Güvendiren on 27/02/2017.
//

#include "Triangle.h"
#include "Mesh.h"
#include "Vertex.h"


Triangle::Triangle(Vertex a, Vertex b, Vertex c, int tid) : pointA(a),
                                                            pointB(b),
                                                            pointC(c),
                                                            id(tid)

{
    surfNormal = glm::normalize(glm::cross(pointB.Data() - pointA.Data(),
                                           pointC.Data() - pointA.Data()));
}

int Triangle::ID() const
{
    return id;
}


Triangle::Triangle(unsigned int a, unsigned int b, unsigned int c, const Mesh& mesh, int tid) : pointA(mesh.GetVertex(a)),
                                                                                                pointB(mesh.GetVertex(b)),
                                                                                                pointC(mesh.GetVertex(c)),
                                                                                                id(tid) {}
Triangle::~Triangle() {}