#ifndef RBD3D_DYNAMICS_FRICTION_CONSTRAINT_H_
#define RBD3D_DYNAMICS_FRICTION_CONSTRAINT_H_

#include <rbd3d/dynamics/ConstraintBase.h>

namespace rbd3d
{
struct FrictionConstraint : public ConstraintBase
{
    FrictionConstraint(RigidbodyBase *a,
                       RigidbodyBase *b,
                       const glm::vec3 &position,
                       const glm::vec3 &tangent);
};
} // namespace rbd3d

#endif