#include <rbd3d/collision/NarrowPhase.h>

namespace rbd3d
{

ContactManifold collision(const RigidbodyBase &a, const RigidbodyBase &b)
{
    if (a.type() == RigidbodyType::SPHERE)
    {
        if (b.type() == RigidbodyType::SPHERE)
            return collision(dynamic_cast<const Sphere &>(a), dynamic_cast<const Sphere &>(b));
        if (b.type() == RigidbodyType::CUBOID)
            return collision(dynamic_cast<const Sphere &>(a), dynamic_cast<const Cuboid &>(b));
        if (b.type() == RigidbodyType::PLANE)
            return collision(dynamic_cast<const Sphere &>(a), dynamic_cast<const Plane &>(b));
    }
    if (a.type() == RigidbodyType::CUBOID)
    {
        if (b.type() == RigidbodyType::SPHERE)
            return collision(dynamic_cast<const Cuboid &>(a), dynamic_cast<const Sphere &>(b));
        if (b.type() == RigidbodyType::CUBOID)
            return collision(dynamic_cast<const Cuboid &>(a), dynamic_cast<const Cuboid &>(b));
        if (b.type() == RigidbodyType::PLANE)
            return collision(dynamic_cast<const Cuboid &>(a), dynamic_cast<const Plane &>(b));
    }
    if (a.type() == RigidbodyType::PLANE)
    {
        if (b.type() == RigidbodyType::SPHERE)
            return collision(dynamic_cast<const Plane &>(a), dynamic_cast<const Sphere &>(b));
        if (b.type() == RigidbodyType::CUBOID)
            return collision(dynamic_cast<const Plane &>(a), dynamic_cast<const Cuboid &>(b));
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

ContactManifold collision(const Sphere &a, const Plane &b)
{
    ContactManifold ret;
    float dis = glm::dot(a.translation() - b.origin(), b.normal()), depth = a.radius() - dis;
    if (depth <= 0)
        return ret;
    ret.pointCount = 1;
    ret.contactPoints[0] = {a.translation() - b.normal() * a.radius(), depth};
    ret.normal = -b.normal();
    return ret;
}

ContactManifold collision(const Cuboid &a, const Plane &b)
{
    ContactManifold ret;
    glm::vec3 sgn;
    glm::vec3 sup = a.support(-b.normal(), sgn);
    float depth = glm::dot(b.origin() - sup, b.normal());
    if (depth <= 0)
        return ret;

    ++ret.pointCount;
    ret.contactPoints[0] = {sup, depth};
    ret.normal = -b.normal();

    float dmin = 0.f;
    int i = 0;
    for (int j = 0; j < 3; ++j)
    {
        glm::vec3 axis(0.f);
        axis[j] = 1.f;
        axis = a.rotation() * axis;

        float d = glm::dot(axis, b.normal());
        if (d > 0)
            d = -d;
        if (d < dmin)
        {
            dmin = d;
            i = j;
        }
    }

    int i1 = (i + 1) % 3, i2 = (i + 2) % 3;

    float sx[]{1.f, -1.f, -1.f, 1.f};
    float sy[]{1.f, 1.f, -1.f, -1.f};
    glm::vec3 h = 0.5f * a.size(), s;
    s[i] = sgn[i];
    for (int j = 0; j < 4; ++j)
    {
        if (sgn[i1] == sx[j] && sgn[i2] == sy[j])
            continue;
        s[i1] = sx[j];
        s[i2] = sy[j];
        glm::vec3 vert = a.rotation() * s * h + a.translation();
        depth = glm::dot(b.origin() - vert, b.normal());
        if (depth > 0)
            ret.contactPoints[ret.pointCount++] = {vert, depth};
    }

    return ret;
}

} // namespace rbd3d