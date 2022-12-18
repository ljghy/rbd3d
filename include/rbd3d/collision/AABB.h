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

    static bool intersect(const AABB &a, const AABB &b)
    {
        return (a.lb.x < b.ub.x && b.lb.x < a.ub.x &&
                a.lb.y < b.ub.y && b.lb.y < a.ub.y &&
                a.lb.z < b.ub.z && b.lb.z < a.ub.z);
    }

    float area() const
    {
        glm::vec3 d = ub - lb;
        return d.x * d.y + d.y * d.z + d.z * d.x;
    }

    bool operator<=(const AABB &other) const
    {
        return (lb.x >= other.lb.x && lb.y >= other.lb.y && lb.z >= other.lb.z &&
                ub.x <= other.ub.x && ub.y <= other.ub.y && ub.z <= other.ub.z);
    }
};

} // namespace rbd3d

#endif
