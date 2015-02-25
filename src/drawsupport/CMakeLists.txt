#include_directories(
#    ${BULLET_PHYSICS_SOURCE_DIR}/src
#    ${BULLET_PHYSICS_SOURCE_DIR}/Extras/ConvexHull
#)


add_library(DrawSupport
    GLDebugFont.cpp
    GLDebugFont.h
    GL_ShapeDrawer.cpp
    GL_ShapeDrawer.h
    #GLDebugDrawer.cpp
    #GLDebugDrawer.h

    DemoApplication.cpp
    DemoApplication.h

    #GLFWDemoApplication.cpp
    GLFWDemoApplication.h
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

include_directories(.)


#target_link_libraries(OpenGLSupport
#    ${BULLET_DYNAMICS_LIBRARY}
#    ${BULLET_COLLISION_LIBRARY}
#    ${BULLET_MATH_LIBRARY}
#    ${GLUT_glut_LIBRARY}
#    ${OPENGL_gl_LIBRARY}
#    ${OPENGL_glu_LIBRARY}
#)
#
##INSTALL of other files requires CMake 2.6
#if(${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} GREATER 2.5)
#    if(INSTALL_EXTRA_LIBS)
#        if(APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)
#            install(TARGETSOpenGLSupport DESTINATION .)
#        else()
#            install(TARGETSOpenGLSupport DESTINATION lib)
#            install(DIRECTORY${CMAKE_CURRENT_SOURCE_DIR} DESTINATION include FILES_MATCHING PATTERN "*.h")
#        endif()
#    endif()
#endif()
