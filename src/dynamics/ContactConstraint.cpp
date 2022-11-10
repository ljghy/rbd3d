#include <rbd3d/dynamics/ContactConstraint.h>
#include <limits>

namespace rbd3d
{

ContactConstraint::ContactConstraint(const CollisionInfo &info)
    : ConstraintBase(info.rigidbodyA, info.rigidbodyB),
      normal(info.normal), depth(info.depth),
      pa(info.contactPointA),
      pb(info.contactPointB)
{
}

glm::vec3 ContactConstraint::va() const
{
    return -normal;
}

glm::vec3 ContactConstraint::wa() const
{
    return glm::cross(normal, pa - a->translation());
}

glm::vec3 ContactConstraint::vb() const
{
    return normal;
}

glm::vec3 ContactConstraint::wb() const
{
    return glm::cross(pb - b->translation(), normal);
}

glm::vec2 ContactConstraint::bound() const
{
    return {0.f, std::numeric_limits<float>::max()};
}

float ContactConstraint::vel() const
{
    return depth;
}

} // namespace rbd3d