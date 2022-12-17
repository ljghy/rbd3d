#ifndef RBD3D_COLLISION_CONTACT_MANIFOLD_H_
#define RBD3D_COLLISION_CONTACT_MANIFOLD_H_

#include <glm/glm.hpp>
#include <array>

namespace rbd3d
{

struct ContactPoint
{
    glm::vec3 position;
    float depth;
};

struct ContactManifold
{
    int pointCount = 0;                        // = 0: not happen
    std::array<ContactPoint, 4> contactPoints; // (x, y, z, penetration depth)
    glm::vec3 normal;

    ContactManifold() = default;

    ContactManifold(const ContactManifold &other)
        : pointCount(other.pointCount)
    {
        if (pointCount)
        {
            for (int i = 0; i < pointCount; ++i)
                contactPoints[i] = other.contactPoints[i];
            normal = other.normal;
        }
    }

    ContactManifold &operator=(const ContactManifold &other)
    {
        if (this != &other)
        {
            pointCount = other.pointCount;
            if (pointCount)
            {
                for (int i = 0; i < pointCount; ++i)
                    contactPoints[i] = other.contactPoints[i];
                normal = other.normal;
            }
        }
        return *this;
    }
};

} // namespace rbd3d

#endif