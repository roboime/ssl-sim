cmake_minimum_required(VERSION 2.8.4)

project(ssl-sim)
set(DESCRIPTION "Small Size League Simulator (Experimental)")

add_definitions("-std=c++11")
if("${CMAKE_GENERATOR}" STREQUAL "Ninja" AND "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  add_definitions("-Xclang -fcolor-diagnostics")
endif()

include(${PROJECT_SOURCE_DIR}/cmake/Utils.cmake)
standard_config()
#standard_paths(${PROJECT_SOURCE_DIR} bin lib)
standard_paths(${PROJECT_BINARY_DIR} bin lib)

# Bullet
find_package(Bullet REQUIRED)
include_directories(${BULLET_INCLUDE_DIR})

# OpenGL/GLEW/GLFW
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
include_directories(${GLEW_INCLUDE_DIRS})
find_package(PkgConfig REQUIRED)
pkg_search_module(GLFW REQUIRED glfw3)
include_directories(${GLFW_INCLUDE_DIRS})

# OpenGL Helper
add_subdirectory(src/drawsupport)
include_directories(src/drawsupport)

add_executable(ssl-sim-cli
    src/main-cli.cpp
)
target_link_libraries(ssl-sim-cli
    ${BULLET_LIBRARIES}
)

add_executable(ssl-sim-gui
    src/main-gui.cpp
    src/sslsim.cpp
    src/sslsim.h
    src/drawsupport/glfwstuff.cpp
)
target_link_libraries(ssl-sim-gui
    DrawSupport
    ${BULLET_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${OPENGL_gl_LIBRARY}
    ${OPENGL_glu_LIBRARY}
    ${GLEW_LIBRARIES}
    ${GLFW_STATIC_LIBRARIES}
)

# vim: et sw=4 ts=3 sts=4
