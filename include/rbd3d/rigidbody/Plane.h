#ifndef RBD3D_RIGIDBODY_PLANE_H_
#define RBD3D_RIGIDBODY_PLANE_H_

#include <rbd3d/rigidbody/RigidbodyBase.h>

namespace rbd3d
{
class Plane : public RigidbodyBase
{
public:
    Plane(const glm::vec3 &_normal = glm::vec3(0.f, 1.f, 0.f),
          const glm::vec3 &_origin = glm::vec3(0.f),
          float _restitution = 0.5f,
          float _friction = 0.5f);
    virtual glm::vec3 support(const glm::vec3 &dir) const override
    {
        assert(0);
        return {};
    }

    virtual RigidbodyType type() const override { return RigidbodyType::PLANE; }

    glm::vec3 origin() const;
    glm::vec3 normal() const;

    void setMass(float) {}

    float mass() const
    {
        assert(0);
        return 0.f;
    }
    glm::mat3 inertia() const
    {
        assert(0);
        return {};
    }

protected:
    virtual void setInertia() override {}

protected:
    glm::vec3 m_normal;
    glm::vec3 m_origin;
};
} // namespace rbd3d
#endif
