#include <rbd3d/collision/SphereVsCapsule.h>

namespace rbd3d
{
ContactManifold collision(const Sphere &a, const Capsule &b)
{
    ContactManifold ret;

    glm::vec3 center = glm::conjugate(b.rotation()) * (a.translation() - b.translation());
    float y0 = glm::abs(center.y) > b.halfHeight() ? glm::sign(center.y) * b.halfHeight() : center.y;
    glm::vec3 norm(-center.x, y0 - center.y, -center.z);
    float dis = glm::length(norm),
          depth = a.radius() + b.radius() - dis;
    if (depth <= 0.f)
        return ret;

    ret.pointCount = 1;
    norm = dis > std::numeric_limits<float>::epsilon() ? norm / dis : glm::vec3(1.f, 0.f, 0.f);
    ret.normal = b.rotation() * norm;
    float dr = a.radius() - b.radius();
    center.y += y0;
    ret.contactPoints[0].position = b.rotation() * (0.5f * (dr * norm + center)) +
                                    b.translation();
    ret.contactPoints[0].depth = depth;

    return ret;
}
} // namespace