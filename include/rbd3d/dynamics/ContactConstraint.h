#ifndef RBD3D_DYNAMICS_CONTACT_CONSTRAINT_H_
#define RBD3D_DYNAMICS_CONTACT_CONSTRAINT_H_

#include <rbd3d/dynamics/ConstraintBase.h>

namespace rbd3d
{
struct ContactConstraint : public ConstraintBase
{
    ContactConstraint(RigidbodyBase *a, RigidbodyBase *b,
                      const glm::vec3 &normal, float depth, const glm::vec3 &position);
};
} // namespace rbd3d

#endif