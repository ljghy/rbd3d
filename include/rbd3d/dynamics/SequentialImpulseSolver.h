#ifndef RBD3D_DYNAMICS_SEQUENCIAL_IMPULSE_SOLVER_H_
#define RBD3D_DYNAMICS_SEQUENCIAL_IMPULSE_SOLVER_H_

#include <rbd3d/rigidbody/RigidbodyBase.h>
#include <rbd3d/dynamics/Constraint.h>

#include <vector>

namespace rbd3d
{
class SequentialImpulseSolver
{
public:
    SequentialImpulseSolver(float _bias, float _tol, uint32_t _iters) : m_bias(_bias), m_tol(_tol), m_iters(_iters) {}
    void solve(size_t constraintCount, const std::vector<Constraint> &constraints, float deltaTime);

private:
    float m_bias;
    float m_tol;
    uint32_t m_iters;

    std::vector<float> lambda;
    std::vector<float> mu;
    std::vector<glm::vec4> pre;
};
} // namespace rbd3d

#endif