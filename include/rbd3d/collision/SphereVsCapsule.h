#ifndef RBD3D_COLLISION_SPHERE_VS_CAPSULE_H_
#define RBD3D_COLLISION_SPHERE_VS_CAPSULE_H_

#include <rbd3d/collision/ContactManifold.h>
#include <rbd3d/rigidbody/Sphere.h>
#include <rbd3d/rigidbody/Capsule.h>

namespace rbd3d
{
ContactManifold collision(const Sphere &a, const Capsule &b);
inline ContactManifold collision(const Capsule &a, const Sphere &b)
{
    ContactManifold ret = collision(b, a);
    if (ret.pointCount)
        ret.normal = -ret.normal;
    return ret;
}
} // namespace rbd3d

#endif