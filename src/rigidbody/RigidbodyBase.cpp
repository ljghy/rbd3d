#include <rbd3d/rigidbody/RigidbodyBase.h>

namespace rbd3d
{

RigidbodyBase::RigidbodyBase(float _mass,
                             float _restitution,
                             float _friction,
                             const glm::vec3 &_translationOffset,
                             const glm::quat &_rotationOffset,
                             const glm::vec3 &_initialVelocity,
                             const glm::vec3 &_initialAngularVelocity)
    : m_mass(_mass),
      m_invMass(_mass > 0 ? 1.f / _mass : 0.f),
      m_inertia(m_mass),
      m_invInertia(m_invMass),
      m_restitution(_restitution),
      m_friction(_friction),
      m_translation(_translationOffset),
      m_rotation(_rotationOffset),
      m_velocity(_initialVelocity),
      m_angularVelocity(_initialAngularVelocity),
      m_force(glm::vec3(0.f)),
      m_torque(glm::vec3(0.f)),
      m_type(RigidbodyType::DYNAMIC),
      m_collisionGroup(0),
      m_collisionFilter(0x0)
{
}

glm::vec3 RigidbodyBase::translation() const
{
    return m_translation;
}

glm::quat RigidbodyBase::rotation() const
{
    return m_rotation;
}

glm::mat3 RigidbodyBase::rotationMat() const
{
    return glm::mat3_cast(m_rotation);
}

void RigidbodyBase::translate(const glm::vec3 &translation)
{
    m_translation += translation;
}

void RigidbodyBase::rotate(const glm::quat &rotation)
{
    m_rotation *= rotation;
    m_rotation = glm::normalize(m_rotation);
}

void RigidbodyBase::setTranslation(const glm::vec3 &translation)
{
    m_translation = translation;
}

void RigidbodyBase::setRotation(const glm::quat &rotation)
{
    m_rotation = rotation;
}

float RigidbodyBase::mass() const
{
    return m_mass;
}

glm::mat3 RigidbodyBase::inertia() const
{
    return m_inertia;
}

float RigidbodyBase::invMass() const
{
    return m_invMass;
}

const glm::mat3 &RigidbodyBase::invInertia() const
{
    return m_invInertia;
}

void RigidbodyBase::setMass(float mass)
{
    m_mass = mass;
    m_invMass = mass > 0.f ? 1.f / mass : 0.f;
    setInertia();
}

glm::vec3 RigidbodyBase::velocity() const
{
    return m_velocity;
}

glm::vec3 RigidbodyBase::angularVelocity() const
{
    return m_angularVelocity;
}

void RigidbodyBase::setVelocity(const glm::vec3 &v)
{
    m_velocity = v;
}

void RigidbodyBase::setAngularVelocity(const glm::vec3 &w)
{
    m_angularVelocity = w;
}

void RigidbodyBase::applyImpulse(const glm::vec3 &i)
{
    m_velocity += m_invMass * i;
}
void RigidbodyBase::applyAngularImpulse(const glm::vec3 &i)
{
    m_angularVelocity += m_invInertia * i;
}

glm::vec3 RigidbodyBase::momentum() const
{
    return m_mass * m_velocity;
}

glm::vec3 RigidbodyBase::angularMomentum() const
{
    return m_inertia * m_angularVelocity;
}

glm::vec3 RigidbodyBase::force() const
{
    return m_force;
}
glm::vec3 RigidbodyBase::torque() const
{
    return m_torque;
}

void RigidbodyBase::setForce(const glm::vec3 &force)
{
    m_force = force;
}
void RigidbodyBase::setTorque(const glm::vec3 &torque)
{
    m_torque = torque;
}

void RigidbodyBase::addForce(const glm::vec3 &force)
{
    m_force += force;
}
void RigidbodyBase::addTorque(const glm::vec3 &torque)
{
    m_torque += torque;
}

float RigidbodyBase::restitution() const
{
    return m_restitution;
}

void RigidbodyBase::setRestitution(float r)
{
    m_restitution = r;
}

float RigidbodyBase::friction() const
{
    return m_friction;
}

void RigidbodyBase::setFriction(float f)
{
    m_friction = f;
}

RigidbodyType RigidbodyBase::type() const
{
    return m_type;
}

void RigidbodyBase::setType(RigidbodyType ty)
{
    m_type = ty;
}

uint16_t RigidbodyBase::collisionGroup() const
{
    return m_collisionGroup;
}

uint16_t RigidbodyBase::collisionFilter() const
{
    return m_collisionFilter;
}

void RigidbodyBase::setCollisionGroup(uint16_t g)
{
    m_collisionGroup = g;
}

void RigidbodyBase::setCollisionFilter(uint16_t f)
{
    m_collisionFilter = f;
}

bool RigidbodyBase::collidableWith(uint16_t g) const
{
    return !((1 << g) & m_collisionFilter);
}

}; // namespace rbd3d
