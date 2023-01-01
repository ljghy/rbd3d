#ifndef RBD3D_RIGIDBODY_RIGIDBODY_BASE_H_
#define RBD3D_RIGIDBODY_RIGIDBODY_BASE_H_

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <rbd3d/collision/AABB.h>

#include <cstdint>

namespace rbd3d
{

enum class RigidbodyShape
{
    CUBOID,
    CAPSULE,
    SPHERE,
};

enum class RigidbodyType
{
    STATIC,
    DYNAMIC,
    KINEMETIC
};

class RigidbodyBase
{
public:
    friend class SequentialImpulseSolver;

    RigidbodyBase(float _mass = 1.f,
                  float _restitution = 0.5f,
                  float _friction = 0.5f,
                  const glm::vec3 &_translationOffset = glm::vec3(0.f),
                  const glm::quat &_rotationOffset = glm::quat(glm::vec3(0.f)),
                  const glm::vec3 &_initialVelocity = glm::vec3(0.f),
                  const glm::vec3 &_initialAngularVelocity = glm::vec3(0.f));
    virtual ~RigidbodyBase() = default;

    glm::vec3 translation() const;
    glm::quat rotation() const;
    glm::mat3 rotationMat() const;

    void translate(const glm::vec3 &translation);
    void rotate(const glm::quat &rotation);

    void setTranslation(const glm::vec3 &translation);
    void setRotation(const glm::quat &rotation);

    float mass() const;
    glm::mat3 inertia() const;

    float invMass() const;
    const glm::mat3 &invInertia() const;

    float restitution() const;
    void setRestitution(float);

    float friction() const;
    void setFriction(float);

    void setMass(float mass);

    glm::vec3 velocity() const;
    glm::vec3 angularVelocity() const;

    void setVelocity(const glm::vec3 &);
    void setAngularVelocity(const glm::vec3 &);

    void applyImpulse(const glm::vec3 &);
    void applyAngularImpulse(const glm::vec3 &);

    glm::vec3 momentum() const;
    glm::vec3 angularMomentum() const;

    glm::vec3 force() const;
    glm::vec3 torque() const;

    void setForce(const glm::vec3 &force);
    void setTorque(const glm::vec3 &torque);

    void addForce(const glm::vec3 &force);
    void addTorque(const glm::vec3 &torque);

    virtual RigidbodyShape shape() const = 0;

    virtual glm::vec3 support(const glm::vec3 &dir) const = 0;

    virtual AABB tightAABB() const = 0;
    virtual AABB enlargedAABB(float scale) const = 0;

    RigidbodyType type() const;
    void setType(RigidbodyType ty);

    uint16_t collisionGroup() const;
    uint16_t collisionFilter() const;

    void setCollisionGroup(uint16_t);
    void setCollisionFilter(uint16_t);

    bool collidableWith(uint16_t) const;

protected:
    virtual void setInertia() = 0;

protected:
    float m_mass;
    float m_invMass;

    glm::mat3 m_inertia;
    glm::mat3 m_invInertia;

    float m_restitution;
    float m_friction;

    glm::vec3 m_translation;
    glm::quat m_rotation;

    glm::vec3 m_velocity;
    glm::vec3 m_angularVelocity;

    glm::vec3 m_force;
    glm::vec3 m_torque;

    RigidbodyType m_type;

    uint16_t m_collisionGroup;
    uint16_t m_collisionFilter;
};
} // namespace rbd3d

#endif
