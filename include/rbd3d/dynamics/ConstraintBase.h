#ifndef RBD3D_DYNAMICS_CONSTRAINT_BASE_H_
#define RBD3D_DYNAMICS_CONSTRAINT_BASE_H_

#include <rbd3d/rigidbody/RigidbodyBase.h>

namespace rbd3d
{
struct ConstraintBase
{
    RigidbodyBase *a;
    RigidbodyBase *b;

    glm::vec3 va;
    glm::vec3 wa;
    glm::vec3 vb;
    glm::vec3 wb;
    glm::vec2 bound;
    float vel;
    float res;

    ConstraintBase(RigidbodyBase *_a, RigidbodyBase *_b) : a(_a), b(_b) {}

    virtual ~ConstraintBase() = default;
};
} // namespace rbd3d

#endif