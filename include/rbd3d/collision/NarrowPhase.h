#ifndef RBD3D_COLLISION_NARROW_PHASE_H_
#define RBD3D_COLLISION_NARROW_PHASE_H_

#include <rbd3d/collision/ContactManifold.h>
#include <rbd3d/collision/GJK.h>
#include <rbd3d/rigidbody.h>

#include <rbd3d/collision/CuboidContact.h>

namespace rbd3d
{

// Collision Table
//          Sphere   Capsule   Cuboid 
// Sphere     O         X        O    
// Capsule              X        X    
// Cuboid                        O    

ContactManifold collision(const RigidbodyBase &a, const RigidbodyBase &b);

// Sphere vs Sphere
ContactManifold collision(const Sphere &a, const Sphere &b);

// Sphere vs Cuboid
ContactManifold collision(const Sphere &a, const Cuboid &b);
inline ContactManifold collision(const Cuboid &a, const Sphere &b)
{
    ContactManifold ret = collision(b, a);
    if (ret.pointCount)
        ret.normal = -ret.normal;
    return ret;
}

// Cuboid vs Cuboid
inline ContactManifold collision(const Cuboid &a, const Cuboid &b) { return cuboidContact(a, b); }

} // namespace rbd3d

#endif