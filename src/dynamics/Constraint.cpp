#include <rbd3d/dynamics/Constraint.h>

namespace rbd3d
{

Constraint Constraint::contactConstraint(RigidbodyBase *a, RigidbodyBase *b,
                                         const glm::vec3 &normal, float depth, const glm::vec3 &position)
{
    return {
        a,
        b,
        -normal,
        glm::cross(normal, position - a->translation()),
        normal,
        glm::cross(position - b->translation(), normal),
        {0.f, std::numeric_limits<float>::max()},
        depth,
        glm::max(a->restitution(), b->restitution())};
}

Constraint Constraint::frictionConstraint(RigidbodyBase *a, RigidbodyBase *b,
                                          const glm::vec3 &position, const glm::vec3 &tangent)
{
    float fm = a->friction() * b->friction();
    return {
        a,
        b,
        -tangent,
        glm::cross(tangent, position - a->translation()),
        tangent,
        glm::cross(position - b->translation(), tangent),
        {-fm, fm},
        0.f,
        0.f};
}

Constraint Constraint::jointTranslationConstraint(RigidbodyBase *a, RigidbodyBase *b,
                                                  const glm::vec3 &normal, float depth, const glm::vec3 &position)
{
    return {
        a,
        b,
        -normal,
        glm::cross(normal, position - a->translation()),
        normal,
        glm::cross(position - b->translation(), normal),
        {0.f, std::numeric_limits<float>::max()},
        depth,
        0.f};
}

Constraint Constraint::jointRotationConstraint(RigidbodyBase *a, RigidbodyBase *b,
                                               const glm::vec3 &axis, float theta)
{
    return {
        a,
        b,
        glm::vec3(0.f),
        axis,
        glm::vec3(0.f),
        -axis,
        {0.f, std::numeric_limits<float>::max()},
        theta,
        0.f};
}

} // namespace rbd3d