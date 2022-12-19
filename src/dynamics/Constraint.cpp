#include <rbd3d/dynamics/Constraint.h>

namespace rbd3d
{

Constraint::Constraint(RigidbodyBase *_a, RigidbodyBase *_b,
                       const glm::vec3 &normal, float depth, const glm::vec3 &position)
    : a(_a), b(_b),
      va(-normal), wa(glm::cross(normal, position - a->translation())),
      vb(normal), wb(glm::cross(position - b->translation(), normal)),
      bound({0.f, std::numeric_limits<float>::max()}),
      violation(depth), restitution(glm::max(a->restitution(), b->restitution()))
{
}

Constraint::Constraint(RigidbodyBase *_a, RigidbodyBase *_b,
                       const glm::vec3 &position, const glm::vec3 &tangent)
    : a(_a), b(_b),
      va(-tangent), wa(glm::cross(tangent, position - a->translation())),
      vb(tangent), wb(glm::cross(position - b->translation(), tangent)),
      violation(0.f), restitution(0.f)
{
    float fm = a->friction() * b->friction();
    bound = glm::vec2(-fm, fm);
}
} // namespace rbd3d