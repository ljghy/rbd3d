#include <rbd3d/collision/NarrowPhase.h>

namespace rbd3d
{

ContactManifold collision(const RigidbodyBase &a, const RigidbodyBase &b)
{
    if (a.shape() == RigidbodyShape::SPHERE)
    {
        if (b.shape() == RigidbodyShape::SPHERE)
            return collision(dynamic_cast<const Sphere &>(a), dynamic_cast<const Sphere &>(b));
        if (b.shape() == RigidbodyShape::CUBOID)
            return collision(dynamic_cast<const Sphere &>(a), dynamic_cast<const Cuboid &>(b));
    }
    if (a.shape() == RigidbodyShape::CUBOID)
    {
        if (b.shape() == RigidbodyShape::SPHERE)
            return collision(dynamic_cast<const Cuboid &>(a), dynamic_cast<const Sphere &>(b));
        if (b.shape() == RigidbodyShape::CUBOID)
            return collision(dynamic_cast<const Cuboid &>(a), dynamic_cast<const Cuboid &>(b));
    }
    return ContactManifold{};
}

ContactManifold collision(const Sphere &a, const Sphere &b)
{
    ContactManifold ret;
    ret.normal = b.translation() - a.translation();
    float dis = glm::length(ret.normal);
    float depth = a.radius() + b.radius() - dis;
    if (dis == 0.f || depth <= 0.f)
        return ret;

    ret.pointCount = 1;
    ret.normal *= 1.f / dis;
    ret.contactPoints[0] = {0.5f * (a.translation() + b.translation() + (a.radius() - b.radius()) * ret.normal), depth};

    return ret;
}

inline int argmin(const glm::vec3 &v)
{
    if (v[0] < v[1] && v[0] < v[2])
        return 0;
    return v[1] < v[2] ? 1 : 2;
}

ContactManifold collision(const Sphere &a, const Cuboid &b)
{
    ContactManifold ret;
    if (!GJK(a, b))
        return ret;

    ret.pointCount = 1;

    glm::vec3 center = glm::conjugate(b.rotation()) * (a.translation() - b.translation());
    const glm::vec3 &halfSize = 0.5f * b.size();
    glm::vec3 absCenter = glm::abs(center);

    int cond = (absCenter.x < halfSize.x) + (absCenter.y < halfSize.y) + (absCenter.z < halfSize.z);
    auto &cp = ret.contactPoints[0];
    if (cond == 3) // deep penetration
    {
        ret.normal = glm::vec3(0.f);
        glm::vec3 dp = halfSize - center, dn = halfSize + center;
        int ip = argmin(dp), in = argmin(dn);

        if (dp[ip] < dn[in])
        {
            center[ip] = halfSize[ip];
            ret.normal[ip] = -1.f;
            cp.depth = dp[ip];
        }
        else
        {
            center[in] = -halfSize[in];
            ret.normal[in] = 1.f;
            cp.depth = dn[in];
        }

        cp.position = b.rotation() * center + b.translation();
        cp.depth += a.radius();
        ret.normal = b.rotation() * ret.normal;
    }
    else if (cond == 2) // face
    {
        int i = argmin(halfSize - absCenter);

        ret.normal = glm::vec3(0.f);
        ret.normal[i] = -glm::sign(center[i]);
        center[i] = glm::sign(center[i]) * halfSize[i];
        cp.position = b.rotation() * center + b.translation();
        cp.depth = a.radius() - (absCenter[i] - halfSize[i]);
        ret.normal = b.rotation() * ret.normal;
    }
    else if (cond == 1) // edge
    {
        int i = argmin(absCenter - halfSize);

        cp.position = glm::sign(center) * halfSize;
        cp.position[i] = center[i];
        glm::vec3 n = cp.position - center;
        float len = glm::length(n);
        cp.depth = a.radius() - len;
        ret.normal = b.rotation() * (n / len);

        cp.position = b.rotation() * cp.position + b.translation();
    }
    else if (cond == 0) // vertex
    {
        glm::vec3 p = b.rotation() * (glm::sign(center) * halfSize) + b.translation();
        glm::vec3 n = p - a.translation();
        float len = glm::length(n);
        ret.normal = len > 0.f ? n / len : glm::normalize(b.translation() - p);
        cp = {p, a.radius() - len};
    }

    return ret;
}

} // namespace rbd3d