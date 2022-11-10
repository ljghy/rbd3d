#ifndef RBD3D_DYNAMICS_CONSTRAINT_SOLVER_H_
#define RBD3D_DYNAMICS_CONSTRAINT_SOLVER_H_

#include <rbd3d/dynamics/Matrix.h>
#include <rbd3d/rigidbody/RigidbodyBase.h>
#include <rbd3d/dynamics/SolverBase.h>

namespace rbd3d
{
class PGSSolver : public SolverBase
{
public:
    PGSSolver(float _bias = 0.05f, float _tol = 1e-3f, uint32_t _iters = 32u) : SolverBase(_bias, _tol, _iters) {}
    virtual void solve(const std::vector<std::unique_ptr<ConstraintBase>> &constraints, float deltaTime) override;

private:
    void projectedGaussSeidel(size_t c);

private:
    Matrix<glm::vec3> J;
    Matrix<glm::vec2> bound;
    Matrix<float> b;
    Matrix<float> A;
    Matrix<float> lambda;
};
} // namespace rbd3d

#endif