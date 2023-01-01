project(test_viewer)

add_subdirectory(lib)

add_executable(test_viewer)

target_sources(test_viewer
PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
    
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Camera.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/RigidbodyRenderer.cpp
    
    ${CMAKE_CURRENT_SOURCE_DIR}/src/TestManager.cpp

    ${CMAKE_CURRENT_SOURCE_DIR}/src/tests/TestCollisionDetection.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/tests/TestPyramid.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/tests/TestPile.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/tests/TestJoint.cpp

)

target_include_directories(test_viewer
PRIVATE 
    ${PROJECT_SOURCE_DIR}/lib/glad/include
    ${PROJECT_SOURCE_DIR}/lib/glfw/include
    ${PROJECT_SOURCE_DIR}/lib/imgui/include
    ${PROJECT_SOURCE_DIR}/lib/opengl_framework/include
)

find_package(OpenGL REQUIRED)

target_link_libraries(test_viewer
PRIVATE 
    glad
    glfw
    imgui
    OpenGL::GL
    opengl_framework

    rbd3d
)
