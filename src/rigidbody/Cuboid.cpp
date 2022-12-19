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
    : RigidbodyBase(_mass, _restitution, _friction, _translationOffset, _rotationOffset, _initialVelocity, _initialAngularVelocity),
      m_halfExtent(0.5f * _size)
{
    setInertia();
}

glm::vec3 Cuboid::support(const glm::vec3 &dir) const
{
    glm::vec3 localDir = glm::conjugate(m_rotation) * dir;
    glm::vec3 s = glm::sign(localDir) * m_halfExtent;
    s = m_translation + m_rotation * s;
    return s;
}

glm::vec3 Cuboid::support(const glm::vec3 &dir, glm::vec3 &sgn) const
{
    glm::vec3 localDir = glm::conjugate(m_rotation) * dir;
    sgn = glm::sign(localDir);
    glm::vec3 s = sgn * m_halfExtent;
    s = m_translation + m_rotation * s;
    return s;
}

glm::vec3 Cuboid::size() const
{
    return 2.f * m_halfExtent;
}

void Cuboid::setSize(const glm::vec3 &s)
{
    m_halfExtent = 0.5f * s;
    setInertia();
}

glm::vec3 Cuboid::halfExtent() const
{
    return m_halfExtent;
}

void Cuboid::setInertia()
{
    glm::vec3 diag = glm::vec3(glm::dot(m_halfExtent, m_halfExtent)) - m_halfExtent * m_halfExtent;
    m_inertia = glm::mat3(m_mass / 3.f);
    m_invInertia = glm::mat3(m_invMass * 3.f);

    for (uint8_t i = 0; i < 3; ++i)
    {
        m_inertia[i][i] *= diag[i];
        m_invInertia[i][i] /= diag[i];
    }
}

AABB Cuboid::tightAABB() const
{
    glm::vec3 r(0.f);
    glm::vec3 sgn[]{{1.f, 1.f, 1.f},
                    {1.f, 1.f, -1.f},
                    {1.f, -1.f, -1.f},
                    {1.f, -1.f, 1.f}};
    for (const auto &s : sgn)
        r = glm::max(glm::abs(m_rotation * (s * m_halfExtent)), r);
    return {m_translation - r, m_translation + r};
}

AABB Cuboid::enlargedAABB(float scale) const
{
    glm::vec3 r(0.f);
    glm::vec3 sgn[]{{1.f, 1.f, 1.f},
                    {1.f, 1.f, -1.f},
                    {1.f, -1.f, -1.f},
                    {1.f, -1.f, 1.f}};
    for (const auto &s : sgn)
        r = glm::max(glm::abs(m_rotation * (s * m_halfExtent)), r);
    r *= scale;
    return {m_translation - r, m_translation + r};
}

} // namespace rbd3d