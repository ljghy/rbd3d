#include <rbd3d/collision/SphereVsCuboid.h>

namespace rbd3d
{
inline int argmin(const glm::vec3 &v)
{
    if (v[0] < v[1] && v[0] < v[2])
        return 0;
    return v[1] < v[2] ? 1 : 2;
}

ContactManifold collision(const Sphere &a, const Cuboid &b)
{
    ContactManifold ret;

    glm::vec3 center = glm::conjugate(b.rotation()) * (a.translation() - b.translation());
    const glm::vec3 h = b.halfExtent();
    glm::vec3 absCenter = glm::abs(center);

    int cond = (absCenter.x < h.x) + (absCenter.y < h.y) + (absCenter.z < h.z);
    auto &cp = ret.contactPoints[0];
    if (cond == 3) // deep penetration
    {
        ret.pointCount = 1;
        ret.normal = glm::vec3(0.f);
        glm::vec3 dp = h - center, dn = h + center;
        int ip = argmin(dp), in = argmin(dn);

        if (dp[ip] < dn[in])
        {
            center[ip] = h[ip];
            ret.normal[ip] = -1.f;
            cp.depth = dp[ip];
        }
        else
        {
            center[in] = -h[in];
            ret.normal[in] = 1.f;
            cp.depth = dn[in];
        }

        cp.position = b.rotation() * center + b.translation();
        cp.depth += a.radius();
        ret.normal = b.rotation() * ret.normal;
    }
    else if (cond == 2) // face
    {
        int i = argmin(h - absCenter);

        float depth = a.radius() - (absCenter[i] - h[i]);
        if (depth <= 0.f)
            return ret;

        ret.pointCount = 1;
        ret.normal = glm::vec3(0.f);
        ret.normal[i] = center[i] > 0.f ? -1.f : 1.f;
        center[i] = -ret.normal[i] * h[i];
        cp.position = b.rotation() * center + b.translation();
        cp.depth = depth;
        ret.normal = b.rotation() * ret.normal;
    }
    else if (cond == 1) // edge
    {
        int i = argmin(absCenter - h);

        cp.position = glm::sign(center) * h;
        cp.position[i] = center[i];
        glm::vec3 n = cp.position - center;
        float len = glm::length(n);
        cp.depth = a.radius() - len;
        if (cp.depth <= 0.f)
            return ret;

        ret.pointCount = 1;
        ret.normal = b.rotation() * (n / len);
        cp.position = b.rotation() * cp.position + b.translation();
    }
    else if (cond == 0) // vertex
    {
        cp.position = b.rotation() * (glm::sign(center) * h) + b.translation();
        glm::vec3 n = cp.position - a.translation();
        float len = glm::length(n);
        cp.depth = a.radius() - len;
        if (cp.depth <= 0.f)
            return ret;

        ret.pointCount = 1;
        ret.normal = len > 0.f ? n / len : glm::normalize(b.translation() - cp.position);
    }

    return ret;
}

} // namespace rbd3d