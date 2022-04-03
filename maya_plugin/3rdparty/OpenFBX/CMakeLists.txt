cmake_minimum_required(VERSION 3.0)
project(OpenFBX)

add_library(OpenFBX STATIC
    miniz.c
    miniz.h
    ofbx.cpp
    ofbx.h
)
target_include_directories(OpenFBX PUBLIC .)