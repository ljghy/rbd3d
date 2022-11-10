#include <rbd3d/rigidbody/Sphere.h>

namespace rbd3d
{

Sphere::Sphere(float _radius,
               float _mass,
               float _restitution,
               float _friction,
               const glm::vec3 &_translationOffset,
               const glm::quat &_rotationOffset,
               const glm::vec3 &_initialVelocity,
               const glm::vec3 &_initialAngularVelocity)
    : RigidbodyBase(_mass, _restitution, _friction, _translationOffset, _rotationOffset, _initialVelocity, _initialAngularVelocity), m_radius(_radius)
{
    setInertia();
}

float Sphere::radius() const
{
    return m_radius;
}

glm::vec3 Sphere::support(const glm::vec3 &dir) const
{
    return translation() + m_radius * glm::normalize(dir);
}

void Sphere::setInertia()
{
    float I = 0.4f * m_mass * m_radius * m_radius;
    m_inertia = glm::mat3(I);
    m_invInertia = glm::mat3(1.f / I);
}

}; // namespace rbd3d