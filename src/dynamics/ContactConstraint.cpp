#include <rbd3d/dynamics/ContactConstraint.h>
#include <limits>

namespace rbd3d
{

ContactConstraint::ContactConstraint(RigidbodyBase *a, RigidbodyBase *b,
                                     const glm::vec3 &normal, float depth, const glm::vec3 &position)
    : ConstraintBase(a, b)
{
    va = -normal;
    wa = glm::cross(normal, position - a->translation());
    vb = normal;
    wb = glm::cross(position - b->translation(), normal);
    bound = glm::vec2(0.f, std::numeric_limits<float>::max());
    vel = depth;
    res = glm::max(a->restitution(), b->restitution());
}

} // namespace rbd3d