#ifndef RBD3D_COLLISION_CAPSULE_VS_CUBOID_H_
#define RBD3D_COLLISION_CAPSULE_VS_CUBOID_H_

#include <rbd3d/collision/ContactManifold.h>
#include <rbd3d/rigidbody/Capsule.h>
#include <rbd3d/rigidbody/Cuboid.h>

namespace rbd3d
{
ContactManifold collision(const Capsule &a, const Cuboid &b);
inline ContactManifold collision(const Cuboid &a, const Capsule &b)
{
    ContactManifold ret = collision(b, a);
    if (ret.pointCount)
        ret.normal = -ret.normal;
    return ret;
}
} // namespace rbd3d

#endif