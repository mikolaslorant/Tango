cmake_minimum_required(VERSION 3.10)

# set the project name
project(CIS562HW)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})

# Set up libraries
add_library(curve STATIC
    ./src/animation/aSplineVec3.h
    ./src/animation/aSplineVec3.cpp
    ./src/animation/aVector.h
    ./src/animation/aVector.cpp
    ./src/animation/aRotation.h
    ./src/animation/aRotation.cpp
    ./src/animation/aSplineQuat.h
    ./src/animation/aSplineQuat.cpp
    ./src/animation/aSolver.cpp
    ./src/animation/aSolver.h
)

target_include_directories(curve PUBLIC
    ./3rdparty/eigen-3.2.8
    ./src/animation
)

# Set up executables/viewers
# Find OpenGL
find_package(OpenGL REQUIRED)

# Add GLFW
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
add_subdirectory(./3rdparty/glfw-3.3.2)

# Add GLAD
add_subdirectory(./3rdparty/glad)

# Add ImGUI
add_subdirectory(./3rdparty/imgui-1.77)

# Add tinyobj
add_library(tinyobj STATIC
    ./3rdparty/tinyobj/tiny_obj_loader.h
    ./3rdparty/tinyobj/tiny_obj_loader.cc
)

# Set up executables
add_executable(CurveViewer
    ./src/viewer/camera.cpp
    ./src/viewer/camera.h
    ./src/viewer/curveMain.cpp
    ./src/viewer/curveViewer.cpp
    ./src/viewer/curveViewer.h
    ./src/viewer/objmodel.h
    ./src/viewer/objmodel.cpp
    ./src/viewer/shader.h
    ./src/viewer/drawable.h
    ./src/viewer/drawable.cpp
    ./src/viewer/viewer.cpp
    ./src/viewer/viewer.h
    ./src/viewer/utils.cpp
    ./src/viewer/utils.h
    ./src/viewer/solver.cpp
    ./src/viewer/solver.h
)

target_include_directories(CurveViewer PUBLIC 
    ./src/viewer
    ./src/animation
    ./3rdparty/imgui-1.77
    ./3rdparty/glm
    ./3rdparty/tinyobj
)

target_link_libraries(CurveViewer PUBLIC curve glad glfw imgui tinyobj)

# Set up Unity plugins
add_library(CurvePlugin SHARED
    ./src/plugin/Plugin.h
    ./src/plugin/CurvePlugin.cpp
)

target_link_libraries(CurvePlugin PUBLIC curve)

set(PLUGIN_OUTPUT_DIR "${PROJECT_SOURCE_DIR}/UnityPlugin/Assets/Plugins")
message(${PLUGIN_OUTPUT_DIR})

set_target_properties(CurvePlugin PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY_DEBUG "${PLUGIN_OUTPUT_DIR}"
    LIBRARY_OUTPUT_DIRECTORY_DEBUG "${PLUGIN_OUTPUT_DIR}"
    ARCHIVE_OUTPUT_DIRECTORY_DEBUG "${PLUGIN_OUTPUT_DIR}"
    RUNTIME_OUTPUT_DIRECTORY_RELEASE "${PLUGIN_OUTPUT_DIR}"
    LIBRARY_OUTPUT_DIRECTORY_RELEASE "${PLUGIN_OUTPUT_DIR}"
    ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${PLUGIN_OUTPUT_DIR}"
    PREFIX "__"
)