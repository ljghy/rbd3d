#ifndef RBD3D_COLLISION_CUBOID_VS_CUBOID_H_
#define RBD3D_COLLISION_CUBOID_VS_CUBOID_H_

#include <rbd3d/collision/ContactManifold.h>
#include <rbd3d/rigidbody/Cuboid.h>

namespace rbd3d
{

ContactManifold collision(const Cuboid &a, const Cuboid &b);

} // namespace rbd3d

#endif
