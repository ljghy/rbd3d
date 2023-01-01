#ifndef RBD3D_DYNAMICS_CONSTRAINT_BASE_H_
#define RBD3D_DYNAMICS_CONSTRAINT_BASE_H_

#include <rbd3d/rigidbody/RigidbodyBase.h>

namespace rbd3d
{
struct Constraint
{
    RigidbodyBase *a;
    RigidbodyBase *b;

    glm::vec3 va;
    glm::vec3 wa;
    glm::vec3 vb;
    glm::vec3 wb;
    glm::vec2 bound;
    float violation;
    float restitution;

    static Constraint contactConstraint(RigidbodyBase *a, RigidbodyBase *b,
                                        const glm::vec3 &normal, float depth, const glm::vec3 &position);
    static Constraint frictionConstraint(RigidbodyBase *a, RigidbodyBase *b,
                                         const glm::vec3 &position, const glm::vec3 &tangent);
    static Constraint jointTranslationConstraint(RigidbodyBase *a, RigidbodyBase *b,
                                                 const glm::vec3 &normal, float depth, const glm::vec3 &position);
    static Constraint jointRotationConstraint(RigidbodyBase *a, RigidbodyBase *b,
                                              const glm::vec3 &axis, float theta);
};
} // namespace rbd3d

#endif