cmake_minimum_required(VERSION 3.6)

project(rbd3d)

add_subdirectory(src)

if (RBD3D_BUILD_TESTS)
    add_subdirectory(test)
endif()
