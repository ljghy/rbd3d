#include <rbd3d/collision/CapsuleVsCapsule.h>

namespace rbd3d
{
ContactManifold collision(const Capsule &a, const Capsule &b)
{
    ContactManifold ret;

    glm::vec3 c1 = a.translation(),
              c2 = b.translation(),
              o1 = a.orientation(),
              o2 = b.orientation(),
              delta = c1 - c2;

    float h1 = a.halfHeight(), h2 = b.halfHeight(),
          rab = a.radius() + b.radius(), dr = a.radius() - b.radius();

    float d12 = glm::dot(o1, o2),
          dd1 = glm::dot(delta, o1),
          dd2 = glm::dot(delta, o2),
          s = 1.f - d12 * d12;

    float r2 = a.radius() * a.radius(),
          parallelThreshold = r2 / (h1 * h1 + r2 + 1e-3f);

    if (glm::abs(s) > parallelThreshold)
    {
        float t1 = glm::clamp((d12 * dd2 - dd1) / s, -h1, h1),
              t2 = glm::clamp((dd2 - d12 * dd1) / s, -h2, h2);

        glm::vec3 p1 = c1 + t1 * o1,
                  p2 = c2 + t2 * o2,
                  norm = p2 - p1;

        float dis = glm::length(norm),
              depth = rab - dis;
        if (depth <= 0.f)
            return ret;

        ret.pointCount = 1;
        ret.normal = dis > std::numeric_limits<float>::epsilon()
                         ? norm / dis
                         : glm::cross(o1, o2) / glm::sqrt(s);
        ret.contactPoints[0].position = 0.5f * (p1 + p2 + dr * ret.normal);
        ret.contactPoints[0].depth = depth;
    }
    else
    {
        float t1 = glm::clamp((dd1 + h1) / d12, -h2, h2),
              t2 = glm::clamp((dd1 - h1) / d12, -h2, h2);

        glm::vec3 q1 = c2 + t1 * o2,
                  q2 = c2 + t2 * o2;

        float s1 = glm::clamp(glm::dot(q1 - c1, o1), -h1, h1),
              s2 = glm::clamp(glm::dot(q2 - c1, o1), -h1, h1);

        glm::vec3 p1 = c1 + s1 * o1,
                  p2 = c1 + s2 * o1;

        glm::vec3 n1 = q1 - p1, n2 = q2 - p2;
        float dis1 = glm::length(n1), dis2 = glm::length(n2);

        float depth1 = rab - dis1,
              depth2 = rab - dis2;

        if (depth1 > 0)
        {
            n1 = dis1 > std::numeric_limits<float>::epsilon()
                     ? n1 / dis1
                     : (s1 > 0 ? o1 : -o1);
            ret.contactPoints[0].position = 0.5f * (p1 + q1 + dr * n1);
            ret.contactPoints[0].depth = depth1;
            ++ret.pointCount;
            ret.normal = n1;
        }

        if (depth2 > 0)
        {
            n2 = dis2 > std::numeric_limits<float>::epsilon()
                     ? n2 / dis2
                     : (s2 > 0 ? o1 : -o1);
            ret.contactPoints[ret.pointCount].position = 0.5f * (p2 + q2 + dr * n2);
            ret.contactPoints[ret.pointCount].depth = depth2;
            ++ret.pointCount;
            ret.normal = n2;
        }
    }

    return ret;
}
} // namespace rbd3d