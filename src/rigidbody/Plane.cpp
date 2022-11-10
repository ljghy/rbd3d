#include <rbd3d/rigidbody/Plane.h>

namespace rbd3d
{

Plane::Plane(const glm::vec3 &_normal,
             const glm::vec3 &_origin,
             float _restitution,
             float _friction)
    : RigidbodyBase(), m_normal(glm::normalize(_normal)), m_origin(_origin)
{
    m_mass = 0.f;
    m_invMass = 0.f;
    m_invInertia = glm::mat3(0.f);
    m_restitution = _restitution;
    m_friction = _friction;
}

glm::vec3 Plane::origin() const
{
    return m_origin;
}
glm::vec3 Plane::normal() const
{
    return m_normal;
}

} // namespace rbd3d