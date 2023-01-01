#ifndef RBD3D_DYNAMICS_DYNAMIC_WORLD_H_
#define RBD3D_DYNAMICS_DYNAMIC_WORLD_H_

#include <rbd3d/collision.h>
#include <rbd3d/joint/Joint.h>
#include <rbd3d/dynamics/Constraint.h>
#include <rbd3d/dynamics/SequentialImpulseSolver.h>

#include <vector>

namespace rbd3d
{
class DynamicWorld
{
public:
    DynamicWorld(const glm::vec3 &_gravity = glm::vec3(0.f, -9.8f, 0.f),
                 float solverBias = 0.1f,
                 float solverTol = 1e-4f,
                 uint32_t solverIters = 64u);

    float update(float deltaTime);
    float fixedUpdate(float deltaTime, float fixedDeltaTime);

    void addRigidbody(RigidbodyBase &);
    void addJoint(Joint &);

    void setGravity(const glm::vec3 &);

private:
    DynamicWorld(const DynamicWorld &) = delete;
    DynamicWorld &operator=(const DynamicWorld &) = delete;

    void clearForce();
    void applyExtForce();
    void applyGravity();
    void detectCollision();
    void checkJoints();
    void solveConstraints(float deltaTime);
    void integrate(float deltaTime);
    void resetAccumulator();
    void updateBVH();

    void addConstraint(const Constraint &);

private:
    std::vector<RigidbodyBase *> m_rigidbodyList;
    std::vector<Joint *> m_jointList;

    glm::vec3 m_gravity;

    size_t m_constraintCount;
    std::vector<Constraint> m_constraints;

    SequentialImpulseSolver m_solver;

    DynamicBVH m_BVH;

    float m_accumulator;
};
} // namespace rbd3d
#endif