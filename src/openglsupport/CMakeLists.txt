# You shouldn't have to modify anything below this line #
#########################################################


include_directories(
    ${BULLET_PHYSICS_SOURCE_DIR}/src
    ${BULLET_PHYSICS_SOURCE_DIR}/Extras/ConvexHull
)


add_library(OpenGLSupport
    GLDebugFont.cpp
    GLDebugFont.h
    GL_ShapeDrawer.cpp
    GL_ShapeDrawer.h
    GLDebugDrawer.cpp
    GLDebugDrawer.h

    DemoApplication.cpp
    DemoApplication.h

    GlutDemoApplication.cpp
    GlutDemoApplication.h
    GlutStuff.cpp
    GlutStuff.h
)


target_link_libraries(OpenGLSupport
    ${BULLET_DYNAMICS_LIBRARY}
    ${BULLET_COLLISION_LIBRARY}
    ${BULLET_MATH_LIBRARY}
    ${GLUT_glut_LIBRARY}
    ${OPENGL_gl_LIBRARY}
    ${OPENGL_glu_LIBRARY}
)

#INSTALL of other files requires CMake 2.6
if(${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} GREATER 2.5)
    if(INSTALL_EXTRA_LIBS)
        if(APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)
            install(TARGETSOpenGLSupport DESTINATION .)
        else()
            install(TARGETSOpenGLSupport DESTINATION lib)
            install(DIRECTORY${CMAKE_CURRENT_SOURCE_DIR} DESTINATION include FILES_MATCHING PATTERN "*.h")
        endif()
    endif()
endif()
