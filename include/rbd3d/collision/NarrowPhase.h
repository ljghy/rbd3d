#ifndef RBD3D_COLLISION_NARROW_PHASE_H_
#define RBD3D_COLLISION_NARROW_PHASE_H_

#include <rbd3d/collision/SphereVsSphere.h>
#include <rbd3d/collision/SphereVsCapsule.h>
#include <rbd3d/collision/SphereVsCuboid.h>
#include <rbd3d/collision/CapsuleVsCapsule.h>
#include <rbd3d/collision/CapsuleVsCuboid.h>
#include <rbd3d/collision/CuboidVsCuboid.h>

namespace rbd3d
{
ContactManifold collision(const RigidbodyBase &a, const RigidbodyBase &b);
} // namespace rbd3d

#endif