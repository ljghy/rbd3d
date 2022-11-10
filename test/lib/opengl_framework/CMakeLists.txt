add_library(opengl_framework STATIC)

target_sources(opengl_framework
PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Shader.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/VertexArrayObject.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/VertexBufferObject.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/IndexBufferObject.cpp
)

target_include_directories(opengl_framework
PRIVATE 
    ${PROJECT_SOURCE_DIR}/lib/glad/include
    ${PROJECT_SOURCE_DIR}/lib/glfw/include

PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
