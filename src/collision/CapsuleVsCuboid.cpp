#include <rbd3d/collision/CapsuleVsCuboid.h>

#include <utility> // std::swap
namespace rbd3d
{

static float testEdge(const glm::vec3 &c1, const glm::vec3 &o1,
                      float hh, const glm::vec3 &he,
                      glm::vec3 &v1, glm::vec3 &v2)
{
    float minDis = std::numeric_limits<float>::max();
    for (int i = 0; i < 3; ++i)
    {
        int i1 = (i + 1) % 3, i2 = (i + 2) % 3;
        for (int j = 0; j < 4; ++j)
        {
            glm::vec3 sgn(0.f);
            sgn[i1] = 2.f * (j % 2) - 1.f;
            sgn[i2] = 2.f * (j / 2) - 1.f;
            glm::vec3 c2 = he * sgn,
                      o2 = glm::vec3(0.f);
            o2[i] = 1.f;

            glm::vec3 delta = c1 - c2;
            float d12 = o1[i],
                  dd1 = glm::dot(delta, o1),
                  dd2 = delta[i],
                  s = 1.f - d12 * d12;

            glm::vec3 p1, p2;
            if (s > std::numeric_limits<float>::epsilon())
            {
                float t1 = glm::clamp((d12 * dd2 - dd1) / s, -hh, hh),
                      t2 = glm::clamp((dd2 - d12 * dd1) / s, -he[i], he[i]);
                p1 = c1 + t1 * o1;
                p2 = c2 + t2 * o2;
            }
            else
            {
                if (c1[i] - hh > he[i])
                {
                    p1 = c1 - hh * o1;
                    p2 = c2 + he[i] * o2;
                }
                else if (c1[i] + hh < -he[i])
                {
                    p1 = c1 + hh * o1;
                    p2 = c2 - he[i] * o2;
                }
                else
                {
                    p1 = c1;
                    p2 = c2;
                    p1[i] = p2[i] = (c1[i] - hh < -he[i]) ? -he[i] : c1[i] - hh;
                }
            }
            float dis = glm::distance(p1, p2);
            if (dis < minDis)
            {
                minDis = dis;
                v1 = p1;
                v2 = p2;
            }
        }
    }
    return minDis;
}

static float testFace(const glm::vec3 &c1, const glm::vec3 &o1,
                      float hh, const glm::vec3 &he,
                      glm::vec3 &v1, glm::vec3 &v2)
{
    float minDis = std::numeric_limits<float>::max();
    glm::vec3 vert[]{c1 + o1 * hh, c1 - o1 * hh};
    for (int i = 0; i < 3; ++i)
    {
        int i1 = (i + 1) % 3, i2 = (i + 2) % 3;
        for (auto &v : vert)
        {
            if (glm::abs(v[i1]) < he[i1] && glm::abs(v[i2]) < he[i2])
            {
                float dis = glm::abs(v[i]) - he[i];
                if (dis < minDis)
                {
                    minDis = dis;
                    v1 = v;
                    v2 = v;
                    v2[i] = v[i] > 0.f ? he[i] : -he[i];
                }
            }
        }
    }
    return minDis;
}

static bool shallowContact(const glm::vec3 &c1, const glm::vec3 &o1,
                           float hh, const glm::vec3 &he)
{
    float t1 = -hh, t2 = hh;
    for (int i = 0; i < 3; ++i)
    {
        if (glm::abs(o1[i]) > std::numeric_limits<float>::epsilon())
        {
            float s1 = (he[i] - c1[i]) / o1[i],
                  s2 = (-he[i] - c1[i]) / o1[i];
            if (s1 > s2)
                std::swap(s1, s2);
            if (s2 < t1 || s1 > t2)
            {
                return true;
            }
            t1 = glm::clamp(t1, s1, s2);
            t2 = glm::clamp(t2, s1, s2);
        }
        else if (glm::abs(c1[i]) > he[i])
        {
            return true;
        }
    }
    return false;
}

ContactManifold collision(const Capsule &a, const Cuboid &b)
{
    ContactManifold ret;

    glm::vec3 c1 = glm::conjugate(b.rotation()) * (a.translation() - b.translation()),
              o1 = glm::conjugate(b.rotation()) * a.orientation(),
              he = b.halfExtent();
    float hh = a.halfHeight(), r = a.radius();

    if (shallowContact(c1, o1, hh, he))
    {
        glm::vec3 e1, e2, f1, f2, p1, p2;
        float dis;
        float dis1 = testEdge(c1, o1, hh, he, e1, e2),
              dis2 = testFace(c1, o1, hh, he, f1, f2);

        if (dis1 < dis2)
        {
            p1 = e1;
            p2 = e2;
            dis = dis1;
        }
        else
        {
            p1 = f1;
            p2 = f2;
            dis = dis2;
        }

        float depth = r - dis;
        if (depth <= 0.f)
            return ret;

        ret.pointCount = 1;
        ret.contactPoints[0].position = b.rotation() * p2 + b.translation();
        ret.contactPoints[0].depth = depth;
        ret.normal = b.rotation() * glm::normalize(p2 - p1);
        return ret;
    }

    struct Query
    {
        float depth;
        int i;
        glm::vec3 sgn;
        glm::vec3 axis;
    } faceQuery{std::numeric_limits<float>::max(), 0},
        edgeQuery{std::numeric_limits<float>::max(), 0};

    // Test face
    for (int i = 0; i < 3; ++i)
    {
        float s1 = c1[i] + hh * o1[i] + (o1[i] > 0.f ? r : -r),
              s2 = 2.f * c1[i] - s1;

        float depth1 = he[i] - glm::min(s1, s2),
              depth2 = he[i] + glm::max(s1, s2);

        if (depth1 < depth2 && depth1 < faceQuery.depth)
        {
            faceQuery.depth = depth1;
            faceQuery.i = i;
            faceQuery.sgn.x = 1.f;
        }
        else if (depth2 < faceQuery.depth)
        {
            faceQuery.depth = depth2;
            faceQuery.i = i;
            faceQuery.sgn.x = -1.f;
        }
    }

    // Test Edge
    for (int i = 0; i < 3; ++i)
    {
        int i1 = (i + 1) % 3, i2 = (i + 2) % 3;
        float depth1, depth2;
        glm::vec3 axis(0.f), sgn;

        if (1.f - glm::abs(o1[i]) < std::numeric_limits<float>::epsilon())
        {
            depth1 = he[i] + hh + r - c1[i];
            depth2 = he[i] + hh + r + c1[i];
        }
        else
        {
            axis[i1] = -o1[i2];
            axis[i2] = o1[i1];
            axis /= glm::sqrt(1.f - o1[i] * o1[i]);

            glm::vec3 sup1 = c1 + r * axis + (glm::dot(axis, o1) > 0.f ? 1.f : -1.f) * o1 * hh;
            sgn = glm::sign(axis);
            glm::vec3 sup2 = he * sgn;

            float s1 = glm::dot(sup1, axis),
                  m = glm::dot(c1, axis),
                  s2 = 2.f * m - s1,
                  d = glm::abs(glm::dot(sup2, axis));

            depth1 = d - glm::min(s1, s2),
            depth2 = d + glm::max(s1, s2);
        }

        if (depth1 < depth2 && depth1 < edgeQuery.depth)
        {
            edgeQuery.depth = depth1;
            edgeQuery.i = i;
            sgn[i] = 0.f;
            edgeQuery.sgn = sgn;
            edgeQuery.axis = -axis;
        }
        else if (depth2 < edgeQuery.depth)
        {
            edgeQuery.depth = depth2;
            edgeQuery.i = i;
            sgn[i] = 0.f;
            edgeQuery.sgn = -sgn;
            edgeQuery.axis = axis;
        }
    }

    constexpr float tol = 1e-5f;
    if (faceQuery.depth < edgeQuery.depth + tol)
    {
        float sgn = faceQuery.sgn.x;
        int i0 = faceQuery.i;
        int i1 = (i0 + 1) % 3, i2 = (i0 + 2) % 3;
        float t1 = hh, t2 = -hh;
        for (int i : {i1, i2})
        {
            float s1 = (he[i] - c1[i]) / o1[i],
                  s2 = (-he[i] - c1[i]) / o1[i];

            if (s1 > s2)
                std::swap(s1, s2);
            t1 = glm::clamp(t1, s1, s2);
            t2 = glm::clamp(t2, s1, s2);
        }

        glm::vec3 p1 = c1 + t1 * o1, p2 = c1 + t2 * o1;
        float depth1 = he[i0] + r - p1[i0] * sgn,
              depth2 = he[i0] + r - p2[i0] * sgn;

        if (depth1 > 0.f)
        {
            ret.pointCount = 1;
            auto &cp = ret.contactPoints[0];
            cp.position = p1;
            cp.position[i0] = sgn * he[i0];
            cp.position = b.rotation() * cp.position + b.translation();
            cp.depth = depth1;
        }

        if (depth2 > 0.f)
        {
            auto &cp = ret.contactPoints[ret.pointCount];
            ++ret.pointCount;
            cp.position = p2;
            cp.position[i0] = sgn * he[i0];
            cp.position = b.rotation() * cp.position + b.translation();
            cp.depth = depth2;
        }

        if (ret.pointCount)
        {
            ret.normal = glm::vec3(0.f);
            ret.normal[i0] = -sgn;
            ret.normal = b.rotation() * ret.normal;
        }
    }
    else
    {
        int i0 = edgeQuery.i, i1 = (i0 + 1) % 3, i2 = (i0 + 2) % 3;
        glm::vec3 c2 = edgeQuery.sgn * he, o2(0.f);
        o2[i0] = 1.f;

        glm::vec3 delta = c1 - c2;
        float d12 = o1[i0],
              dd1 = glm::dot(delta, o1),
              dd2 = delta[i0],
              s = 1.f - d12 * d12;

        float t1 = glm::clamp((d12 * dd2 - dd1) / s, -hh, hh),
              t2 = glm::clamp((dd2 - d12 * dd1) / s, -he[i0], he[i0]);

        glm::vec3 p1 = c1 + t1 * o1,
                  p2 = c2 + t2 * o2,
                  norm = p2 - p1;

        float dis = glm::length(norm),
              depth = r - dis;

        if (depth > 0.f)
        {
            ret.pointCount = 1;
            auto &cp = ret.contactPoints[0];
            cp.position = 0.5f * (p1 + norm / dis * r + p2);
            cp.position = b.rotation() * cp.position + b.translation();
            cp.depth = depth;
            ret.normal = b.rotation() * edgeQuery.axis;
        }
    }

    return ret;
}
} // namespace rbd3d