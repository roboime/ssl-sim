# OpenGL/GLEW/GLFW
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
include_directories(${GLEW_INCLUDE_DIRS})
find_package(PkgConfig REQUIRED)
pkg_search_module(GLFW REQUIRED glfw3)
include_directories(${GLFW_INCLUDE_DIRS})
include_directories(.)

add_library(DrawSupport
    GL_ShapeDrawer.cpp
    GL_ShapeDrawer.h
    DemoApplication.cpp
    DemoApplication.h
    glfwstuff.cpp
    glfwstuff.h

    # ImGui
    imconfig.h
    imgui.cpp
    imgui.h
    stb_rect_pack.h
    stb_textedit.h
    stb_truetype.h
)

target_link_libraries(DrawSupport
    ${BULLET_DYNAMICS_LIBRARY}
    ${BULLET_COLLISION_LIBRARY}
    ${BULLET_MATH_LIBRARY}
    ${OPENGL_LIBRARIES}
    #${OPENGL_gl_LIBRARY}
    ${GLEW_LIBRARIES}
    ${GLFW_STATIC_LIBRARIES}
)
