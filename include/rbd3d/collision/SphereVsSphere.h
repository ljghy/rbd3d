#ifndef RBD3D_COLLISION_SPHERE_VS_SPHERE_H_
#define RBD3D_COLLISION_SPHERE_VS_SPHERE_H_

#include <rbd3d/collision/ContactManifold.h>
#include <rbd3d/rigidbody/Sphere.h>

namespace rbd3d
{
ContactManifold collision(const Sphere &a, const Sphere &b);
}

#endif