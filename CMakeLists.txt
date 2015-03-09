cmake_minimum_required(VERSION 2.8.4)

project(ssl-sim)
set(DESCRIPTION "Small Size League Simulator (Experimental)")


option(BUILD_CLI "Build command line interface" ON)
option(BUILD_GUI "Build graphical user interface (requires OpenGL+GLFW)" ON)


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

# Protobuf
find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})

protobuf_generate_cpp(PROTO_CPP PROTO_H
    protos/messages_robocup_ssl_detection.proto
    protos/messages_robocup_ssl_geometry.proto
    protos/messages_robocup_ssl_wrapper.proto
    protos/messages_robocup_ssl_refbox_log.proto
    protos/grSim_Replacement.proto
    protos/grSim_Commands.proto
    protos/grSim_Packet.proto
)

add_library(ssl-sim
    src/sslsim.cpp
    src/sslsim.h
    ${PROTO_CPP}
    ${PROTO_H}
)

target_link_libraries(ssl-sim
    ${BULLET_LIBRARIES}
    ${PROTOBUF_LIBRARIES}
)

if(BUILD_CLI)
    add_executable(ssl-sim-cli src/main-cli.cpp)
    target_link_libraries(ssl-sim-cli ssl-sim)
endif()

if(BUILD_GUI)
    add_executable(ssl-sim-gui src/main-gui.cpp)

    # OpenGL Helper
    add_subdirectory(src/drawsupport)
    include_directories(src/drawsupport)

    target_link_libraries(ssl-sim-gui ssl-sim DrawSupport)
endif()

# vim: et sw=4 ts=3 sts=4
