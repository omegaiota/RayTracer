cmake_minimum_required(VERSION 2.8)

# Application source
set(APPLICATION_SOURCE

    # Collada Parser
    collada/collada.cpp
    collada/camera_info.cpp
    collada/light_info.cpp
    collada/sphere_info.cpp
    collada/polymesh_info.cpp
    collada/material_info.cpp

    # Dynamic Scene
    dynamic_scene/mesh.cpp
    dynamic_scene/scene.cpp
    dynamic_scene/sphere.cpp
    dynamic_scene/widgets.cpp
    dynamic_scene/skeleton.cpp
    dynamic_scene/joint.cpp

    # Static scene
    static_scene/sphere.cpp
    static_scene/triangle.cpp
    static_scene/object.cpp
    static_scene/environment_light.cpp
    static_scene/light.cpp

    # MeshEdit
    halfEdgeMesh.cpp
    meshEdit.cpp

    # PathTracer
    bvh.cpp
    bbox.cpp
    bsdf.cpp
    camera.cpp
    sampler.cpp
    pathtracer.cpp

    # Animator
    timeline.cpp

    # misc
    misc/sphere_drawing.cpp
    getopt.c
    error_dialog.cpp

    # Application
    application.cpp
    main.cpp
)

#-------------------------------------------------------------------------------
# Set include directories
#-------------------------------------------------------------------------------
include_directories(
  ${GLEW_INCLUDE_DIRS}
  ${GLFW_INCLUDE_DIRS}
  ${PROJ6850_INCLUDE_DIRS}
  ${FREETYPE_INCLUDE_DIRS}
)

#-------------------------------------------------------------------------------
# Set link directories
#-------------------------------------------------------------------------------
link_directories(
  ${GLEW_LIBRARY_DIRS}
  ${GLFW_LIBRARY_DIRS}
  ${PROJ6850_LIBRARY_DIRS}
  ${FREETYPE_LIBRARY_DIRS}
)

#-------------------------------------------------------------------------------
# Add executable
#-------------------------------------------------------------------------------
add_executable(raytracer6850 ${APPLICATION_SOURCE})

target_link_libraries( raytracer6850
    PROJ6850 ${PROJ6850_LIBRARIES}
    glew ${GLEW_LIBRARIES}
    glfw ${GLFW_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${FREETYPE_LIBRARIES}
    ${CMAKE_THREADS_INIT}
)

#-------------------------------------------------------------------------------
# Platform-specific configurations for target
#-------------------------------------------------------------------------------
if(APPLE)
  set_property( TARGET raytracer6850 APPEND_STRING PROPERTY COMPILE_FLAGS
                "-Wno-deprecated-declarations -Wno-c++11-extensions")
endif(APPLE)

# Put executable in build directory root
set(EXECUTABLE_OUTPUT_PATH ..)

# Install to project root
install(TARGETS raytracer6850 DESTINATION ${raytracer6850_SOURCE_DIR})

# Copy Freetype DLLs to the build directory
if(WIN32)
    foreach(FREETYPE_DLL ${FREETYPE_RUNTIMELIBS})
        add_custom_command(TARGET raytracer6850 POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${FREETYPE_DLL}"
                $<TARGET_FILE_DIR:raytracer6850>)
    endforeach(FREETYPE_DLL)
endif(WIN32)
