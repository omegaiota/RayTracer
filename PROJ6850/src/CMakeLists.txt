cmake_minimum_required(VERSION 2.8)

include_directories(${PROJECT_SOURCE_DIR}/include/PROJ6850)

# PROJ6850 library source files
set(PROJ6850_SOURCE
    vector2D.cpp
    vector3D.cpp
    vector4D.cpp
    matrix3x3.cpp
    matrix4x4.cpp
    quaternion.cpp
    complex.cpp
    color.cpp
    spectrum.cpp
    osdtext.cpp
    osdfont.cpp
    viewer.cpp
    base64.cpp
    lodepng.cpp
    tinyxml2.cpp
)

#-------------------------------------------------------------------------------
# Building static library (always)
#-------------------------------------------------------------------------------
add_library(PROJ6850 STATIC ${PROJ6850_SOURCE})

target_link_libraries(
  PROJ6850
  ${GLEW_LIBRARIES}
  ${GLFW_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${FREETYPE_LIBRARIES}
)

#-------------------------------------------------------------------------------
# Building shared library
#-------------------------------------------------------------------------------
if(PROJ6850_BUILD_SHARED)
  add_library(PROJ6850_SHARED SHARED ${PROJ6850_SOURCE})
  target_link_libraries(
    PROJ6850_SHARED
    ${GLEW_LIBRARIES}
    ${GLFW_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${FREETYPE_LIBRARIES}
  )
endif()

#-------------------------------------------------------------------------------
# Platform-specific target configurations
#-------------------------------------------------------------------------------

###################
# Building on OSX #
###################
if (APPLE)

  # Clang options
  target_compile_options(PROJ6850 PRIVATE -Wno-constant-conversion)

  # Framework dependencies
  target_link_libraries(PROJ6850
    ${COCOA_LIBRARIES}
    ${IOKIT_LIBRARIES}
    ${COREVIDEO_LIBRARIES})

  # Output name
  if(PROJ6850_BUILD_DEBUG)
    set_target_properties(PROJ6850 PROPERTIES OUTPUT_NAME PROJ6850_osx_d)
    if(PROJ6850_BUILD_SHARED)
      set_target_properties(PROJ6850_SHARED PROPERTIES OUTPUT_NAME PROJ6850_osx_d)
    endif()
  else(PROJ6850_BUILD_DEBUG)
    set_target_properties(PROJ6850 PROPERTIES OUTPUT_NAME PROJ6850_osx)
    if(PROJ6850_BUILD_SHARED)
      set_target_properties(PROJ6850_SHARED PROPERTIES OUTPUT_NAME PROJ6850_osx)
    endif()
  endif(PROJ6850_BUILD_DEBUG)

endif(APPLE)

##################
# Build on Linux #
##################
set(LINUX UNIX AND NOT APPLE)
if(LINUX)
  if(PROJ6850_BUILD_DEBUG)
    set_target_properties(PROJ6850 PROPERTIES OUTPUT_NAME PROJ6850_d)
    if(PROJ6850_BUILD_SHARED)
      set_target_properties(PROJ6850_SHARED PROPERTIES OUTPUT_NAME PROJ6850_d)
    endif()
  else(PROJ6850_BUILD_DEBUG)
    set_target_properties(PROJ6850 PROPERTIES OUTPUT_NAME PROJ6850)
    if(PROJ6850_BUILD_SHARED)
      set_target_properties(PROJ6850_SHARED PROPERTIES OUTPUT_NAME PROJ6850)
    endif()
  endif(PROJ6850_BUILD_DEBUG)
endif(LINUX)

####################
# Build on Windows #
####################
if(WIN32)

  if(MSVC)
    if(PROJ6850_BUILD_DEBUG)
      set_target_properties(PROJ6850 PROPERTIES OUTPUT_NAME PROJ6850_d)
      if(PROJ6850_BUILD_SHARED)
        set_target_properties(PROJ6850_SHARED PROPERTIES OUTPUT_NAME PROJ6850_d)
      endif()
    else(PROJ6850_BUILD_DEBUG)
      set_target_properties(PROJ6850 PROPERTIES OUTPUT_NAME PROJ6850)
      if(PROJ6850_BUILD_SHARED)
        set_target_properties(PROJ6850_SHARED PROPERTIES OUTPUT_NAME PROJ6850)
      endif()
    endif(PROJ6850_BUILD_DEBUG)
  endif(MSVC)

  if(MINGW)
    if(PROJ6850_BUILD_DEBUG)
      set_target_properties(PROJ6850 PROPERTIES OUTPUT_NAME PROJ6850_d)
      if(PROJ6850_BUILD_SHARED)
        set_target_properties(PROJ6850_SHARED PROPERTIES OUTPUT_NAME PROJ6850_d)
      endif()
    else(PROJ6850_BUILD_DEBUG)
      set_target_properties(PROJ6850 PROPERTIES OUTPUT_NAME PROJ6850)
      if(PROJ6850_BUILD_SHARED)
        set_target_properties(PROJ6850_SHARED PROPERTIES OUTPUT_NAME PROJ6850)
      endif()
    endif(PROJ6850_BUILD_DEBUG)
  endif(MINGW)

endif(WIN32)

#-------------------------------------------------------------------------------
# Installation configurations
#-------------------------------------------------------------------------------
install(TARGETS PROJ6850 DESTINATION lib)
if(PROJ6850_BUILD_SHARED)
  install(TARGETS PROJ6850_SHARED DESTINATION lib)
endif()

# install headers
install(FILES ${PROJ6850_HEADER} DESTINATION include/PROJ6850)
