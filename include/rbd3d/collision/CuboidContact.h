#ifndef RBD3D_COLLISION_CUBOID_CONTACT_H_
#define RBD3D_COLLISION_CUBOID_CONTACT_H_

#include <rbd3d/rigidbody.h>
#include <rbd3d/collision/ContactManifold.h>

namespace rbd3d
{

ContactManifold cuboidContact(const Cuboid &, const Cuboid &);

} // namespace rbd3d

#endif
