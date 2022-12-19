#ifndef RBD3D_COLLISION_AABB_H_
#define RBD3D_COLLISION_AABB_H_

#include <glm/glm.hpp>

namespace rbd3d
{

extern const float enlargedAABBScale;

struct AABB
{
    glm::vec3 lb;
    glm::vec3 ub;

    static AABB merge(const AABB &a, const AABB &b);
    static bool intersect(const AABB &a, const AABB &b);
    float area() const;
    bool operator<=(const AABB &other) const;
};

} // namespace rbd3d

#endif
