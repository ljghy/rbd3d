#ifndef RBD3D_DYNAMICS_SOLVER_BASE_H_
#define RBD3D_DYNAMICS_SOLVER_BASE_H_

#include <vector>
#include <memory>

#include <rbd3d/dynamics/ConstraintBase.h>

namespace rbd3d
{
class SolverBase
{
public:
    SolverBase(float _bias, float _tol, uint32_t _iters) : m_bias(_bias), m_tol(_tol), m_iters(_iters) {}
    virtual ~SolverBase() = default;

public:
    virtual void solve(const std::vector<std::unique_ptr<ConstraintBase>> &constraints, float deltaTime) = 0;
    void setBias(float bias) { m_bias = bias; }
    void setTol(float tol) { m_tol = tol; }
    void setIters(uint32_t iters) { m_iters = iters; }

protected:
    float m_bias;
    float m_tol;
    uint32_t m_iters;
};

} // namespace rbd3d

#endif