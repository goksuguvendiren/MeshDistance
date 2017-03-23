cmake_minimum_required(VERSION 3.6)
project(Geometry)

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake_modules/")

set(CMAKE_CXX_STANDARD 14)

find_package(assimp REQUIRED)

set(SOURCE_FILES main.cpp Mesh.cpp Mesh.h Triangle.cpp Triangle.h Vertex.cpp Vertex.h)
add_executable(Geometry ${SOURCE_FILES})

target_link_libraries(Geometry ${assimp_LIBRARIES})
target_include_directories(Geometry PUBLIC ${assimp_INCLUDE_DIRS})