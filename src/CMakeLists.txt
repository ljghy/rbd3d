add_library(rbd3d STATIC)

target_sources(rbd3d
PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/dynamics/Constraint.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/dynamics/SequentialImpulseSolver.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/dynamics/DynamicWorld.cpp
    
    ${CMAKE_CURRENT_SOURCE_DIR}/collision/AABB.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/collision/DynamicBVH.cpp
    
    ${CMAKE_CURRENT_SOURCE_DIR}/collision/NarrowPhase.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/collision/SphereVsSphere.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/collision/SphereVsCapsule.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/collision/SphereVsCuboid.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/collision/CapsuleVsCapsule.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/collision/CapsuleVsCuboid.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/collision/CuboidVsCuboid.cpp

    ${CMAKE_CURRENT_SOURCE_DIR}/rigidbody/RigidbodyBase.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/rigidbody/Sphere.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/rigidbody/Cuboid.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/rigidbody/Capsule.cpp
    
    ${CMAKE_CURRENT_SOURCE_DIR}/joint/Joint.cpp
)

target_include_directories(rbd3d
PUBLIC
    ${PROJECT_SOURCE_DIR}/include/glm
    ${PROJECT_SOURCE_DIR}/include
)
