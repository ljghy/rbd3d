#include <rbd3d/dynamics/FrictionConstraint.h>

namespace rbd3d
{

FrictionConstraint::FrictionConstraint(RigidbodyBase *a,
                                       RigidbodyBase *b,
                                       const glm::vec3 &position,
                                       const glm::vec3 &tangent)
    : ConstraintBase(a, b)
{
    va = -tangent;
    wa = glm::cross(tangent, position - a->translation());
    vb = tangent;
    wb = glm::cross(position - b->translation(), tangent);
    float fm = a->friction() * b->friction();
    bound = glm::vec2(-fm, fm);
    vel = 0.f;
    res = 0.f;
}
} // namespace rbd3d