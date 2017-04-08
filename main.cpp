#include <rtk/rtk_init.hpp>
#include <rtk/window.hpp>
#include <rtk/mesh_ops.hpp>

#include <rtk/geometry/mesh.hpp>
#include <rtk/geometry/path.hpp>

#include <rtk/gl/program.hpp>
#include <rtk/gl/path.hpp>
#include <rtk/gl/mesh.hpp>
#include <rtk/gl/shader.hpp>

#include <rtk/asset/mesh_import.hpp>

#include <thread>
#include <fstream>
#include <iostream>
#include <fstream>
#include <iomanip>

#include "Mesh.h"
#include "GeometryProcessing.h"

std::string read_text_file(const std::string& path)
{
    std::ifstream f(path);
    return {std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>()};
}

int main()
{
    using namespace rtk::literals;
    using namespace std::chrono_literals;
    rtk::rtk_init i;

    rtk::window w ({ 640_px, 480_px });

    Mesh mesh;
    mesh.Load("/Users/goksu/Documents/Geometry/inputs/iso_curve_descriptor/man2.off");

    rtk::geometry::mesh geomesh;
    geomesh.set_vertices(mesh.GetVertexData());
    geomesh.set_faces(mesh.GetFaces());

    rtk::gl::mesh gl_mesh(geomesh);

    auto start = std::chrono::steady_clock::now();
//    auto distancemap = GeometryProcessing::GenerateDistanceMap(mesh);

//    GeometryProcessing::BilateralMap(mesh, 1, 10, distancemap);

    auto dist = std::chrono::steady_clock::now();
    std::cerr << "Loading took "
    << std::chrono::duration_cast<std::chrono::milliseconds>(dist - start).count()
    << "ms.\n";

    GeometryProcessing::GeodesicDescriptor(mesh, 80, 40, GeometryProcessing::GeodesicDistance_FibHeap(mesh, 80));

    rtk::gl::vertex_shader line_vs { read_text_file("../rtk/shaders/line.vert").c_str() };
    rtk::gl::fragment_shader line_fs { read_text_file("../rtk/shaders/line.frag").c_str() };
    rtk::gl::geometry_shader gs { read_text_file("../rtk/shaders/line.geom").c_str() };

    rtk::gl::vertex_shader mesh_vs { read_text_file("../rtk/shaders/basic.vert").c_str() };
    rtk::gl::fragment_shader mesh_fs { read_text_file("../rtk/shaders/basic.frag").c_str() };

    rtk::gl::program path_shader;
    path_shader.attach(line_vs);
    path_shader.attach(gs);
    path_shader.attach(line_fs);
    path_shader.link();
    path_shader.set_variable("color", glm::vec3(1.0f, 0.5f, 0.31f));

    rtk::gl::program mesh_shader;
    mesh_shader.attach(mesh_vs);
    mesh_shader.attach(mesh_fs);
    mesh_shader.link();

    auto vPath = GeometryProcessing::GeneratePath(mesh, 1, 10, GeometryProcessing::GeodesicDistance_FibHeap(mesh, 1));
    std::vector<glm::vec3> path;

    for(auto& pth : vPath){
        path.push_back(pth.Data());
    }

    rtk::geometry::path p;
    p.set_vertices(path);

    rtk::gl::path gl_p (p);

    auto normals = rtk::geometry::generate_normals(geomesh);
    gl_mesh.add_vertex_data<glm::vec3>(1, normals);

    while (!w.should_close())
    {
        w.begin_draw();

        gl_mesh.draw(mesh_shader);

        gl_p.draw(path_shader);

        w.end_draw();
        std::this_thread::sleep_for(10ms);
    }
}
