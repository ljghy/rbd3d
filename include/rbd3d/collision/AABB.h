#ifndef RBD3D_COLLISION_AABB_H_
#define RBD3D_COLLISION_AABB_H_

#include <glm/glm.hpp>

namespace rbd3d
{

struct AABB
{
    glm::vec3 lb;
    glm::vec3 ub;

    static AABB merge(const AABB &a, const AABB &b)
    {
        return {glm::min(a.lb, b.lb), glm::max(a.ub, b.ub)};
    }

    float area() const
    {
        glm::vec3 d = ub - lb;
        return d.x * d.y + d.y * d.z + d.z * d.x;
    }
};

} // namespace rbd3d

#endif
