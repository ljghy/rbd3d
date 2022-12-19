#ifndef RBD3D_RIGIDBODY_CUPSULE_H_
#define RBD3D_RIGIDBODY_CUPSULE_H_

#include <rbd3d/rigidbody/RigidbodyBase.h>

namespace rbd3d
{

class Capsule : public RigidbodyBase
{
public:
    Capsule(float _radius = 0.5f,
            float _height = 1.f,
            float _mass = 1.f,
            float _restitution = 0.5f,
            float _friction = 0.5f,
            const glm::vec3 &_translationOffset = glm::vec3(0.f),
            const glm::quat &_rotationOffset = glm::quat(glm::vec3(0.f)),
            const glm::vec3 &_initialVelocity = glm::vec3(0.f),
            const glm::vec3 &_initialAngularVelocity = glm::vec3(0.f));

    virtual glm::vec3 support(const glm::vec3 &dir) const override;

    virtual RigidbodyShape shape() const override { return RigidbodyShape::SPHERE; }

    float radius() const;
    void setRadius(float);

    float height() const;
    void setHeight(float);

    float halfHeight() const;

    virtual AABB tightAABB() const override;
    virtual AABB enlargedAABB(float scale) const override;

    glm::vec3 orientation() const;

protected:
    virtual void setInertia() override;

protected:
    float m_radius;
    float m_halfHeight;
};

} // namespace rbd3d
#endif