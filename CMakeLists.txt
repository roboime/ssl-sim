cmake_minimum_required(VERSION 2.8.4)

project(ssl-sim)
set(DESCRIPTION "Small Size League Simulator (Experimental)")


option(BUILD_CLI "Build command line interface" ON)
option(BUILD_GUI "Build graphical user interface (requires OpenGL+GLFW)" ON)

# Super strict warnings =)
set(CMAKE_CXX_FLAGS "-Wall -Wextra -pedantic -pedantic-errors ${CMAKE_CXX_FLAGS}")

include(${PROJECT_SOURCE_DIR}/cmake/Utils.cmake)
standard_config()
#standard_paths(${PROJECT_SOURCE_DIR} bin lib)
standard_paths(${PROJECT_BINARY_DIR} bin lib)

require_cxx14()
require_c11()

# enable clang-color with ninja
if("${CMAKE_GENERATOR}" STREQUAL "Ninja" AND "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    add_definitions("-Xclang -fcolor-diagnostics")
endif()

# Bullet
find_package(Bullet REQUIRED)
include_directories(SYSTEM ${BULLET_INCLUDE_DIR})

# Protobuf
find_package(Protobuf REQUIRED)
include_directories(SYSTEM ${PROTOBUF_INCLUDE_DIRS})

protobuf_generate_cpp(PROTO_CPP PROTO_H
    protos/messages_robocup_ssl_detection.proto
    protos/messages_robocup_ssl_geometry.proto
    protos/messages_robocup_ssl_wrapper.proto
    protos/messages_robocup_ssl_refbox_log.proto
    protos/grSim_Replacement.proto
    protos/grSim_Commands.proto
    protos/grSim_Packet.proto
)

add_library(sslsim
    src/vector.h
    src/team.h
    src/field.h
    src/field.c
    src/world.h
    src/robot.h
    src/ball.h
    src/sslsim.h
    src/sslsim.cpp
)

target_link_libraries(sslsim ${BULLET_LIBRARIES})

if(BUILD_CLI)
    set(cli_app ssl-sim-cli)
    add_executable(${cli_app} src/main.c)
    target_link_libraries(${cli_app} sslsim)
endif()

if(BUILD_GUI)
    find_package(OpenGL REQUIRED)
    find_package(GLEW REQUIRED)
    include_directories(SYSTEM ${GLEW_INCLUDE_DIRS})
    find_package(PkgConfig REQUIRED)
    pkg_search_module(GLFW REQUIRED glfw3)
    include_directories(SYSTEM ${GLFW_INCLUDE_DIRS})
    include_directories(SYSTEM imgui)
    set(gui_app ssl-sim-gui)
    add_executable(${gui_app}
        src/draw.h
        src/draw.cpp
        src/gui_main.cpp

        # utils
        src/utils/colors.h
        src/utils/gl.h
        src/utils/net.cpp
        src/utils/net.h
        src/utils/stack_vector.h

        # protobuf
        ${PROTO_CPP}
        ${PROTO_H}

        # imgui
        imgui/imconfig.h
        imgui/imgui.cpp
        imgui/imgui.h
        imgui/stb_rect_pack.h
        imgui/stb_textedit.h
        imgui/stb_truetype.h
    )
    target_link_libraries(${gui_app} sslsim
        ${PROTOBUF_LIBRARIES}
        ${BULLET_DYNAMICS_LIBRARY}
        ${BULLET_COLLISION_LIBRARY}
        ${BULLET_MATH_LIBRARY}
        ${OPENGL_LIBRARIES}
        #${OPENGL_gl_LIBRARY}
        ${GLEW_LIBRARIES}
        ${GLFW_STATIC_LIBRARIES}
    )
endif()

# vim: et sw=4 ts=3 sts=4
