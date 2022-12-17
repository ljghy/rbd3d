#ifndef RBD3D_COLLISION_GJK_H_
#define RBD3D_COLLISION_GJK_H_

#include <rbd3d/rigidbody/RigidbodyBase.h>

#include <array>

namespace rbd3d
{

struct SimplexVert
{
    glm::vec3 vert;
    glm::vec3 va;
};

using Simplex = std::array<SimplexVert, 4>;

SimplexVert support(const RigidbodyBase &a, const RigidbodyBase &b, const glm::vec3 &dir);
bool GJK(const RigidbodyBase &a, const RigidbodyBase &b, SimplexVert * = nullptr);

} // namespace rbd3d

#endif
