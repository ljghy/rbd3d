#ifndef RBD3D_DYNAMICS_CONTACT_CONSTRAINT_H_
#define RBD3D_DYNAMICS_CONTACT_CONSTRAINT_H_

#include <rbd3d/rigidbody/CollisionDetection.h>
#include <rbd3d/dynamics/ConstraintBase.h>

namespace rbd3d
{
struct ContactConstraint : public ConstraintBase
{
    virtual glm::vec3 va() const override;
    virtual glm::vec3 wa() const override;
    virtual glm::vec3 vb() const override;
    virtual glm::vec3 wb() const override;
    virtual glm::vec2 bound() const override;
    virtual float vel() const override;

    ContactConstraint(const CollisionInfo &);

private:
    glm::vec3 normal;
    float depth;
    glm::vec3 pa;
    glm::vec3 pb;
};
} // namespace rbd3d

#endif