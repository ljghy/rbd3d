#include <rbd3d/collision/SphereVsSphere.h>

namespace rbd3d
{

ContactManifold collision(const Sphere &a, const Sphere &b)
{
    ContactManifold ret;
    ret.normal = b.translation() - a.translation();
    float dis = glm::length(ret.normal);
    float depth = a.radius() + b.radius() - dis;
    if (depth <= 0.f)
        return ret;

    ret.pointCount = 1;
    ret.normal = (dis > std::numeric_limits<float>::epsilon()) ? ret.normal / dis : glm::vec3(0.f, 1.f, 0.f);
    ret.contactPoints[0] = {0.5f * (a.translation() + b.translation() + (a.radius() - b.radius()) * ret.normal), depth};

    return ret;
}

} // namespace rbd3d