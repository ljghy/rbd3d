add_library(rbd3d STATIC)

target_sources(rbd3d
PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/dynamics/ContactConstraint.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/dynamics/FrictionConstraint.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/dynamics/SequentialImpulseSolver.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/dynamics/DynamicWorld.cpp
    
    ${CMAKE_CURRENT_SOURCE_DIR}/collision/DynamicBVH.cpp
    
    ${CMAKE_CURRENT_SOURCE_DIR}/collision/GJK.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/collision/CuboidContact.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/collision/NarrowPhase.cpp

    ${CMAKE_CURRENT_SOURCE_DIR}/rigidbody/RigidbodyBase.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/rigidbody/Sphere.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/rigidbody/Cuboid.cpp
)

target_include_directories(rbd3d
PUBLIC
    ${PROJECT_SOURCE_DIR}/include/glm
    ${PROJECT_SOURCE_DIR}/include
)
