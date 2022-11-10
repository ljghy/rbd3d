#ifndef RBD3D_COLLISION_COLLISION_DETECTION_H_
#define RBD3D_COLLISION_COLLISION_DETECTION_H_

#include <rbd3d/rigidbody/GJK.h>

#include <rbd3d/rigidbody/Plane.h>

#include <vector>
#include <limits>

namespace rbd3d
{

struct CollisionInfo
{
    bool happen;
    glm::vec3 normal; // point to B
    float depth;
    glm::vec3 contactPointA;
    glm::vec3 contactPointB;

    RigidbodyBase *rigidbodyA;
    RigidbodyBase *rigidbodyB;
};

constexpr float EPA_TOL = 1e-4f;
constexpr size_t PRE_ALLOC = 8u;
constexpr int EPA_MAX_ITER = 64;

struct Polytope
{
    std::vector<SimplexVert> vertices;
    std::vector<glm::ivec3> faces;
};

glm::vec3 normal(const Polytope &p, size_t idx);
void extendPolytope(Polytope &p, const SimplexVert &vert);

// EPA
template <typename RigidbodyTypeA, typename RigidbodyTypeB>
inline CollisionInfo collisionInfo(const RigidbodyTypeA &a, const RigidbodyTypeB &b)
{
    CollisionInfo info{false};

    if (b.type() == RigidbodyType::PLANE)
    {
        if (a.type() == RigidbodyType::PLANE)
            return info;

        const Plane &pb = dynamic_cast<const Plane &>(b);
        glm::vec3 n = -pb.normal();
        glm::vec3 sup = a.support(n);

        info.depth = glm::dot(sup - pb.origin(), n);

        if (info.depth <= 0)
        {
            info.happen = false;
            return info;
        }
        info.happen = true;
        info.normal = n;
        info.contactPointA = sup;
        info.contactPointB = sup - info.depth * n;
        info.rigidbodyA = const_cast<RigidbodyTypeA *>(&a);
        info.rigidbodyB = const_cast<RigidbodyTypeB *>(&b);
        return info;
    }
    else if (a.type() == RigidbodyType::PLANE)
    {
        return collisionInfo(b, a);
    }

    Polytope p;
    p.vertices.reserve(PRE_ALLOC);
    p.faces.reserve(PRE_ALLOC);
    p.vertices.resize(4);
    p.faces.resize(4);

    if (!collision(a, b, p.vertices.data()))
        return info;

    info.happen = true;

    p.faces[0] = {0, 2, 1};
    glm::vec3 n = normal(p, 0);

    if (glm::dot(p.vertices[0].vert, n) < 0)
    {
        p.faces[0] = {0, 1, 2};
        p.faces[1] = {0, 3, 1};
        p.faces[2] = {0, 2, 3};
        p.faces[3] = {1, 3, 2};
    }
    else
    {
        p.faces[1] = {0, 1, 3};
        p.faces[2] = {0, 3, 2};
        p.faces[3] = {1, 2, 3};
    }

    int iter = 0;
    while (true)
    {
        float minDis = std::numeric_limits<float>::max();
        glm::vec3 minNormal;
        size_t minIdx = 0;
        for (size_t i = 0; i < p.faces.size(); ++i)
        {
            n = glm::normalize(normal(p, i));
            float dis = glm::max(glm::abs(glm::dot(p.vertices[p.faces[i].x].vert, n)),
                                 glm::max(
                                     glm::abs(glm::dot(p.vertices[p.faces[i].y].vert, n)),
                                     glm::abs(glm::dot(p.vertices[p.faces[i].z].vert, n))));
            if (dis < minDis)
            {
                minDis = dis;
                minNormal = n;
                minIdx = i;
            }
        }

        auto sup = support(a, b, minNormal);

        ++iter;
        if (glm::abs(glm::dot(minNormal, sup.vert)) <= minDis + EPA_TOL || iter > EPA_MAX_ITER)
        {
            info.normal = minNormal;
            info.depth = minDis;
            info.contactPointA = sup.va;
            info.contactPointB = sup.va - sup.vert;
            info.rigidbodyA = const_cast<RigidbodyTypeA *>(&a);
            info.rigidbodyB = const_cast<RigidbodyTypeB *>(&b);
            return info;
        }
        extendPolytope(p, sup);
    }
}

} // namespace rbd3d

#endif
