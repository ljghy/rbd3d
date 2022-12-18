#ifndef RBD3D_RIGIDBODY_CUBOID_H_
#define RBD3D_RIGIDBODY_CUBOID_H_

#include <rbd3d/rigidbody/RigidbodyBase.h>

namespace rbd3d
{
class Cuboid : public RigidbodyBase
{
public:
    Cuboid(const glm::vec3 &_size = glm::vec3(1.f),
           float _mass = 1.f,
           float _restitution = 0.5f,
           float _friction = 0.5f,
           const glm::vec3 &_translationOffset = glm::vec3(0.f),
           const glm::quat &_rotationOffset = glm::quat(glm::vec3(0.f)),
           const glm::vec3 &_initialVelocity = glm::vec3(0.f),
           const glm::vec3 &_initialAngularVelocity = glm::vec3(0.f));
    virtual glm::vec3 support(const glm::vec3 &dir) const override;
    glm::vec3 support(const glm::vec3 &dir, glm::vec3 &sgn) const;

    virtual RigidbodyType type() const override { return RigidbodyType::CUBOID; }

    glm::vec3 size() const;
    void setSize(const glm::vec3 &);

protected:
    virtual void setInertia() override;

protected:
    glm::vec3 m_size;
};
} // namespace rbd3d
#endif
