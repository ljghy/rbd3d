#include <rbd3d/dynamics/FrictionConstraint.h>

namespace rbd3d
{

FrictionConstraint::FrictionConstraint(const CollisionInfo &info,
                                       const glm::vec3 &_tangent)
    : ConstraintBase(info.rigidbodyA, info.rigidbodyB),
      tangent(_tangent),
      pa(info.contactPointA),
      pb(info.contactPointB)
{
}

glm::vec3 FrictionConstraint::va() const
{
    return -tangent;
}

glm::vec3 FrictionConstraint::wa() const
{
    return glm::cross(tangent, pa - a->translation());
}

glm::vec3 FrictionConstraint::vb() const
{
    return tangent;
}

glm::vec3 FrictionConstraint::wb() const
{
    return glm::cross(pb - b->translation(), tangent);
}

glm::vec2 FrictionConstraint::bound() const
{
    float fm = a->friction() * b->friction();
    return {-fm, fm};
}

float FrictionConstraint::vel() const
{
    return 0.f;
}
} // namespace rbd3d