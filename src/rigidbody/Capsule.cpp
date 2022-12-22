#include <rbd3d/rigidbody/Capsule.h>

namespace rbd3d
{

Capsule::Capsule(float _radius,
                 float _height,
                 float _mass,
                 float _restitution,
                 float _friction,
                 const glm::vec3 &_translationOffset,
                 const glm::quat &_rotationOffset,
                 const glm::vec3 &_initialVelocity,
                 const glm::vec3 &_initialAngularVelocity)
    : RigidbodyBase(_mass, _restitution, _friction, _translationOffset, _rotationOffset, _initialVelocity, _initialAngularVelocity),
      m_radius(_radius), m_halfHeight(0.5f * _height)
{
    setInertia();
}

float Capsule::radius() const
{
    return m_radius;
}

void Capsule::setRadius(float r)
{
    m_radius = r;
    setInertia();
}

float Capsule::height() const
{
    return m_halfHeight * 2.f;
}

void Capsule::setHeight(float h)
{
    m_halfHeight = h * 0.5f;
    setInertia();
}

float Capsule::halfHeight() const
{
    return m_halfHeight;
}

glm::vec3 Capsule::support(const glm::vec3 &dir) const
{
    glm::vec3 o = m_rotation * glm::vec3(0.f, m_halfHeight, 0.f);
    return m_translation + m_radius * dir + (glm::dot(dir, o) > 0.f ? 1.f : -1.f) * o;
}

void Capsule::setInertia()
{
    float sqrR = m_radius * m_radius, sqrH = m_halfHeight * m_halfHeight * 4.f;
    float v_hs = 4.f / 3.f * glm::pi<float>() * sqrR * m_radius,
          v_cy = 2.f * glm::pi<float>() * sqrR * m_halfHeight,
          v = v_hs + v_cy;
    float density = m_mass / v,
          m_hs = density * v_hs,
          m_cy = density * v_cy;

    m_invInertia = m_inertia = glm::mat3(0.f);
    if (m_invMass == 0.f)
        return;

    float d0 = m_cy * (sqrH / 12.f + sqrR * 0.25f) +
               m_hs * (0.4f * sqrR + 0.5f * sqrH + 0.75f * m_halfHeight * m_radius),
          d1 = (m_cy * 0.5f + m_hs * 0.4f) * sqrR;

    m_inertia[0][0] = m_inertia[2][2] = d0;
    m_inertia[1][1] = d1;

    m_invInertia[0][0] = m_invInertia[2][2] = 1.f / d0;
    m_invInertia[1][1] = 1.f / d1;
}

static glm::vec3 sign2(glm::vec3 v) // 1 or -1
{
    for (int i = 0; i < 3; ++i)
        v[i] = v[i] > 0.f ? 1.f : -1.f;
    return v;
}

AABB Capsule::tightAABB() const
{
    glm::vec3 r = m_halfHeight * (m_rotation * glm::vec3(0.f, 1.f, 0.f));
    r = glm::abs(r + sign2(r) * glm::vec3(m_radius));
    return {m_translation - r, m_translation + r};
}

AABB Capsule::enlargedAABB(float scale) const
{
    glm::vec3 r = m_halfHeight * (m_rotation * glm::vec3(0.f, 1.f, 0.f));
    r = scale * glm::abs(r + sign2(r) * glm::vec3(m_radius));
    return {m_translation - r, m_translation + r};
}

glm::vec3 Capsule::orientation() const
{
    return m_rotation * glm::vec3(0.f, 1.f, 0.f);
}

} // namespace rbd3d