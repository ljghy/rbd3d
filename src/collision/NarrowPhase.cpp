#include <rbd3d/collision/NarrowPhase.h>

namespace rbd3d
{

ContactManifold collision(const RigidbodyBase &a, const RigidbodyBase &b)
{
    if (a.shape() == RigidbodyShape::SPHERE)
    {
        if (b.shape() == RigidbodyShape::SPHERE)
            return collision(dynamic_cast<const Sphere &>(a), dynamic_cast<const Sphere &>(b));
        if (b.shape() == RigidbodyShape::CAPSULE)
            return collision(dynamic_cast<const Sphere &>(a), dynamic_cast<const Capsule &>(b));
        if (b.shape() == RigidbodyShape::CUBOID)
            return collision(dynamic_cast<const Sphere &>(a), dynamic_cast<const Cuboid &>(b));
    }
    if (a.shape() == RigidbodyShape::CAPSULE)
    {
        if (b.shape() == RigidbodyShape::SPHERE)
            return collision(dynamic_cast<const Capsule &>(a), dynamic_cast<const Sphere &>(b));
        if (b.shape() == RigidbodyShape::CAPSULE)
            return collision(dynamic_cast<const Capsule &>(a), dynamic_cast<const Capsule &>(b));
        if (b.shape() == RigidbodyShape::CUBOID)
            return collision(dynamic_cast<const Capsule &>(a), dynamic_cast<const Cuboid &>(b));
    }
    if (a.shape() == RigidbodyShape::CUBOID)
    {
        if (b.shape() == RigidbodyShape::SPHERE)
            return collision(dynamic_cast<const Cuboid &>(a), dynamic_cast<const Sphere &>(b));
        if (b.shape() == RigidbodyShape::CAPSULE)
            return collision(dynamic_cast<const Cuboid &>(a), dynamic_cast<const Capsule &>(b));
        if (b.shape() == RigidbodyShape::CUBOID)
            return collision(dynamic_cast<const Cuboid &>(a), dynamic_cast<const Cuboid &>(b));
    }
    return ContactManifold{};
}

} // namespace rbd3d