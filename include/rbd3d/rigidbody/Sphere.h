#ifndef RBD3D_RIGIDBODY_SPHERE_H_
#define RBD3D_RIGIDBODY_SPHERE_H_

#include <rbd3d/rigidbody/RigidbodyBase.h>
namespace rbd3d
{
class Sphere : public RigidbodyBase
{
public:
    Sphere(float _radius = 1.f,
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

protected:
    virtual void setInertia() override;

protected:
    float m_radius;
};
} // namespace rbd3d

#endif