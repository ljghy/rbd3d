#include <rbd3d/rigidbody/Cuboid.h>

namespace rbd3d
{

Cuboid::Cuboid(const glm::vec3 &_size,
               float _mass,
               float _restitution,
               float _friction,
               const glm::vec3 &_translationOffset,
               const glm::quat &_rotationOffset,
               const glm::vec3 &_initialVelocity,
               const glm::vec3 &_initialAngularVelocity)
    : RigidbodyBase(_mass, _restitution, _friction, _translationOffset, _rotationOffset, _initialVelocity, _initialAngularVelocity), m_size(_size)
{
    setInertia();
}

glm::vec3 Cuboid::support(const glm::vec3 &dir) const
{
    glm::vec3 localDir = glm::inverse(rotation()) * dir;
    glm::vec3 s(0.f);
    if (localDir.x > 0)
        s.x = m_size.x;
    if (localDir.y > 0)
        s.y = m_size.y;
    if (localDir.z > 0)
        s.z = m_size.z;
    s -= 0.5f * m_size;

    s = translation() + rotation() * s;

    return s;
}

glm::vec3 Cuboid::size() const
{
    return m_size;
}

void Cuboid::setInertia()
{
    glm::vec3 diag = glm::vec3(glm::dot(m_size, m_size)) - m_size * m_size;
    m_inertia = glm::mat3(m_mass / 12.f);
    m_invInertia = glm::mat3(m_invMass * 12.f);

    for (uint8_t i = 0; i < 3; ++i)
    {
        m_inertia[i][i] *= diag[i];
        m_invInertia[i][i] /= diag[i];
    }
}

} // namespace rbd3d