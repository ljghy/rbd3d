add_library(rbd3d STATIC)

target_sources(rbd3d
PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/dynamics/ContactConstraint.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/dynamics/FrictionConstraint.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/dynamics/PGSSolver.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/dynamics/SISolver.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/dynamics/DynamicWorld.cpp

    ${CMAKE_CURRENT_SOURCE_DIR}/rigidbody/GJK.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/rigidbody/CollisionDetection.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/rigidbody/RigidbodyBase.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/rigidbody/Sphere.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/rigidbody/Cuboid.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/rigidbody/Plane.cpp
)

target_include_directories(rbd3d
PUBLIC
    ${PROJECT_SOURCE_DIR}/include/glm
    ${PROJECT_SOURCE_DIR}/include
)