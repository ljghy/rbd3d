#ifndef RBD3D_COLLISION_CAPSULE_VS_CAPSULE_H_
#define RBD3D_COLLISION_CAPSULE_VS_CAPSULE_H_

#include <rbd3d/collision/ContactManifold.h>
#include <rbd3d/rigidbody/Capsule.h>

namespace rbd3d
{
ContactManifold collision(const Capsule &a, const Capsule &b);
}

#endif