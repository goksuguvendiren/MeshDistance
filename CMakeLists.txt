cmake_minimum_required(VERSION 3.6)
project(Geometry)

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake_modules/")
LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/rtk/cmake_modules/")

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O0")

find_package(OpenGL REQUIRED)
find_package(GLFW REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(assimp REQUIRED)

link_directories(rtk/build)

find_package(Boost REQUIRED COMPONENTS system thread filesystem)

set(SOURCE_FILES main.cpp Mesh.cpp Mesh.h Triangle.cpp Triangle.h Vertex.cpp Vertex.h GeometryProcessing.cpp GeometryProcessing.h)
add_executable(Geometry ${SOURCE_FILES})

target_compile_definitions(Geometry PRIVATE GLFW_INCLUDE_NONE=1)

target_link_libraries(Geometry ${Boost_LIBRARIES})

target_include_directories(Geometry PUBLIC rtk)

if(CMAKE_BUILD_TYPE MATCHES "Debug")
    target_include_directories(Geometry PUBLIC rtk/glad_debug/include)
else()
    target_include_directories(Geometry PUBLIC rtk/glad/include)
endif()

target_include_directories(Geometry PUBLIC rtk/3rd_party/gsl/include)
target_include_directories(Geometry PUBLIC glm/)

target_link_libraries(Geometry m dl rtk)
target_link_libraries(Geometry ${OPENGL_LIBRARIES})
target_link_libraries(Geometry ${GLFW_LIBRARIES})
target_link_libraries(Geometry ${Boost_LIBRARIES})
target_link_libraries(Geometry ${assimp_LIBRARIES})

target_include_directories(Geometry PUBLIC ${OPENGL_INCLUDE_DIR})
target_include_directories(Geometry PUBLIC ${GLFW_INCLUDE_DIR})
target_include_directories(Geometry PUBLIC ${Boost_INCLUDE_DIRS})
target_include_directories(Geometry PUBLIC ${assimp_INCLUDE_DIRS})
