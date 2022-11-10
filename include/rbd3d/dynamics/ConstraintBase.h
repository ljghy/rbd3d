#ifndef RBD3D_DYNAMICS_CONSTRAINT_BASE_H_
#define RBD3D_DYNAMICS_CONSTRAINT_BASE_H_

#include <rbd3d/rigidbody/RigidbodyBase.h>

namespace rbd3d
{
struct ConstraintBase
{
    RigidbodyBase *a;
    RigidbodyBase *b;

    virtual glm::vec3 va() const = 0;
    virtual glm::vec3 wa() const = 0;
    virtual glm::vec3 vb() const = 0;
    virtual glm::vec3 wb() const = 0;
    virtual glm::vec2 bound() const = 0;
    virtual float vel() const = 0;

    ConstraintBase(RigidbodyBase *_a, RigidbodyBase *_b) : a(_a), b(_b) {}

    virtual ~ConstraintBase() = default;
};
} // namespace rbd3d

#endif