cmake_minimum_required(VERSION 3.11)

add_library(
    imgui STATIC
    imconfig.h
    imgui.cpp
    imgui.h
    imgui_demo.cpp
    imgui_draw.cpp
    imgui_impl_glfw.cpp
    imgui_impl_glfw.h
    imgui_impl_opengl3.cpp
    imgui_impl_opengl3.h
    imgui_internal.h
    imgui_widgets.cpp
    imstb_rectpack.h
    imstb_textedit.h
    imstb_truetype.h
)

target_include_directories(imgui PUBLIC ../glfw-3.3.2/include)
target_include_directories(imgui PUBLIC ../glad/include)