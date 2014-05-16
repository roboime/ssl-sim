cmake_minimum_required(VERSION 2.8.4)

project(ssl-sim)
set(DESCRIPTION "Small Size League Simulator (Experimental)")

include(${PROJECT_SOURCE_DIR}/cmake/Utils.cmake)
standard_config()
#standard_paths(${PROJECT_SOURCE_DIR} bin lib)
standard_paths(${PROJECT_BINARY_DIR} bin lib)

# Bullet
find_package(Bullet REQUIRED)
include_directories(${BULLET_INCLUDE_DIR})

# Glut/OpenGL
find_package(GLUT REQUIRED)
find_package(OpenGL REQUIRED)

# OpenGL Helper
add_subdirectory(src/openglsupport)
include_directories(src/openglsupport)

add_executable(ssl-sim
    #src/sslsim.cc
    #src/sslsim.h
    src/main.cc
)
target_link_libraries(ssl-sim
    OpenGLSupport
    ${BULLET_LIBRARIES}
)

# vim: et sw=4 ts=3 sts=4
