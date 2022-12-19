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
    const glm::vec3 h = b.halfExtent();
    glm::vec3 absCenter = glm::abs(center);

    int cond = (absCenter.x < h.x) + (absCenter.y < h.y) + (absCenter.z < h.z);
    auto &cp = ret.contactPoints[0];
    if (cond == 3) // deep penetration
    {
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

        ret.normal = glm::vec3(0.f);
        ret.normal[i] = -glm::sign(center[i]);
        center[i] = glm::sign(center[i]) * h[i];
        cp.position = b.rotation() * center + b.translation();
        cp.depth = a.radius() - (absCenter[i] - h[i]);
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
        ret.normal = b.rotation() * (n / len);

        cp.position = b.rotation() * cp.position + b.translation();
    }
    else if (cond == 0) // vertex
    {
        glm::vec3 p = b.rotation() * (glm::sign(center) * h) + b.translation();
        glm::vec3 n = p - a.translation();
        float len = glm::length(n);
        ret.normal = len > 0.f ? n / len : glm::normalize(b.translation() - p);
        cp = {p, a.radius() - len};
    }

    return ret;
}

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
    norm = dis > std::numeric_limits<float>::min() ? norm / dis : glm::vec3(1.f, 0.f, 0.f);
    ret.normal = b.rotation() * norm;
    float dr = a.radius() - b.radius();
    center.y += y0;
    ret.contactPoints[0].position = b.rotation() * (0.5f * (dr * norm + center)) +
                                    b.translation();
    ret.contactPoints[0].depth = depth;

    return ret;
}

ContactManifold collision(const Capsule &a, const Capsule &b)
{
    glm::vec3 ra = a.halfHeight() * a.orientation(),
              rb = b.halfHeight() * b.orientation(),
              delta = rb - ra + a.translation() - b.translation();

    ra *= 2.f;
    rb *= 2.f;
}

} // namespace rbd3d