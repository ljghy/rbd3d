#include <rbd3d/collision/AABB.h>

namespace rbd3d
{
const float enlargedAABBScale = 1.3f;

AABB AABB::merge(const AABB &a, const AABB &b)
{
    return {glm::min(a.lb, b.lb), glm::max(a.ub, b.ub)};
}

bool AABB::intersect(const AABB &a, const AABB &b)
{
    return (a.lb.x < b.ub.x && b.lb.x < a.ub.x &&
            a.lb.y < b.ub.y && b.lb.y < a.ub.y &&
            a.lb.z < b.ub.z && b.lb.z < a.ub.z);
}

float AABB::area() const
{
    glm::vec3 d = ub - lb;
    return d.x * d.y + d.y * d.z + d.z * d.x;
}

bool AABB::operator<=(const AABB &other) const
{
    return (lb.x >= other.lb.x && lb.y >= other.lb.y && lb.z >= other.lb.z &&
            ub.x <= other.ub.x && ub.y <= other.ub.y && ub.z <= other.ub.z);
}

} // namespace rbd3d