#ifndef RBD3D_DYNAMICS_DYNAMIC_WORLD_H_
#define RBD3D_DYNAMICS_DYNAMIC_WORLD_H_

#include <rbd3d/collision.h>
#include <rbd3d/constraint.h>
#include <rbd3d/dynamics/SolverBase.h>

#include <vector>
#include <memory>

namespace rbd3d
{
class DynamicWorld
{
public:
    DynamicWorld(std::unique_ptr<SolverBase> &&_solver,
                 const glm::vec3 &_gravity = glm::vec3(0.f, -9.8f, 0.f),
                 uint32_t _substeps = 16u);

    float update(float deltaTime);

    void addRigidbody(RigidbodyBase &);

private:
    DynamicWorld(const DynamicWorld &) = delete;
    DynamicWorld &operator=(const DynamicWorld &) = delete;

    void clearForce();
    void applyExtForce();
    void applyGravity();
    void detectCollision();

    void solveConstraints(float deltaTime);

    void integrate(float deltaTime);

private:
    std::vector<RigidbodyBase *> m_rigidbodyList;

    glm::vec3 m_gravity;
    uint32_t m_substeps;

    std::vector<std::unique_ptr<ConstraintBase>> m_constraints;

    std::unique_ptr<SolverBase> m_solver;
};
} // namespace rbd3d
#endif