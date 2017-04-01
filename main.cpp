#include <iostream>
#include <fstream>
#include "Mesh.h"
#include <iomanip>

#include <rtk/rtk_init.hpp>
#include <rtk/window.hpp>

#include <rtk/gl/shader.hpp>
#include <rtk/geometry/mesh.hpp>
#include <rtk/geometry/path.hpp>
#include <fstream>
#include <rtk/gl/program.hpp>
#include <thread>

#include <rtk/asset/mesh_import.hpp>
#include <rtk/gl/path.hpp>
#include <rtk/gl/mesh.hpp>
#include <rtk/mesh_ops.hpp>

#include <thread>

std::string read_text_file(const std::string& path)
{
    std::ifstream f(path);
    return {std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>()};
}

int main()
{
    std::string name = "man0";

    Mesh mesh;
    mesh.Load("/Users/goksu/Documents/Geometry/inputs/geosedic/fprint_matrix/" + name + ".off");

    auto start = std::chrono::steady_clock::now();
//    for (int i = 0; i < mesh.NumVertices() && i < 1000; i++){
//        mesh.GeodesicDistance(mesh.GetVertex(i));
//    }

//    auto sth = mesh.GeodesicDistance(mesh.GetVertex(0));
//
//    auto vert0 = mesh.GetVertex(0);
//    auto vert1 = mesh.GetVertex(1);
//
//    std::cerr << vert0.EuclideanDistance(vert1) << "\n";
//
//    for (auto i : sth){
//        std::cerr << i.first << " ";
//    }

    std::fstream out("/Users/goksu/Documents/Geometry/inputs/geosedic/fprint_matrix/" + name + "_output.txt");

    auto M = mesh.GenerateDistanceMap();

    for (int i = 0; i < M.size(); i++){
        for (int j = 0; j < M.size(); j++){
            out << M[i][j].first << " " ;
        }
        out << "\n";
    }

    auto dist = std::chrono::steady_clock::now();
    std::cerr << "Loading took "
              << std::chrono::duration_cast<std::chrono::milliseconds>(dist - start).count()
              << "ms.\n";

    return 0;

}

void Drawer()
{
    using namespace rtk::literals;
    using namespace std::chrono_literals;
    rtk::rtk_init i;

    rtk::window w ({ 640_px, 480_px });

    Mesh mesh;
    mesh.Load("/Users/goksu/Documents/Geometry/inputs/geosedic/fprint_matrix/horse0.off");

    rtk::geometry::mesh geomesh;
    geomesh.set_vertices(mesh.GetVertices());
    geomesh.set_faces(mesh.GetFaces());

    rtk::gl::mesh gl_mesh(geomesh);

    auto start = std::chrono::steady_clock::now();
    for (int i = 0; i < mesh.NumVertices() && i < 1000; i++){
        mesh.GeodesicDistance(mesh.GetVertex(i));
    }
    auto dist = std::chrono::steady_clock::now();
    std::cerr << "Loading took "
    << std::chrono::duration_cast<std::chrono::milliseconds>(dist - start).count()
    << "ms.\n";

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

    rtk::geometry::path p;
    p.set_vertices(std::vector<glm::vec3>{ geomesh.get_vertices()[0], geomesh.get_vertices()[1], geomesh.get_vertices()[2] });

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
