#include <rbd3d/dynamics/SequentialImpulseSolver.h>

namespace rbd3d
{
void SequentialImpulseSolver::solve(size_t c, const std::vector<Constraint> &constraints, float deltaTime)
{
    mu.resize(c);
    pre.resize(c);
    float idt = 1.f / deltaTime;
    for (size_t i = 0; i < c; ++i)
    {
        const auto &ci = constraints[i];
        mu[i] = ci.a->m_invMass * glm::dot(ci.va, ci.va) +
                glm::dot(ci.wa, ci.a->m_invInertia * ci.wa) +
                ci.b->m_invMass * glm::dot(ci.vb, ci.vb) +
                glm::dot(ci.wb, ci.b->m_invInertia * ci.wb);
        mu[i] = mu[i] > 0 ? 1.f / mu[i] : 0.f;

        pre[i] = {1.f + ci.restitution, m_bias * idt * ci.violation, ci.bound.x * deltaTime, ci.bound.y * deltaTime};
    }

    lambda.resize(c);
    for (uint32_t iter = 0; iter < m_iters; ++iter)
    {
        float err = 0.f;
        for (size_t i = 0; i < c; ++i)
        {
            if (mu[i] == 0)
                continue;
            float last = lambda[i];
            const auto &ci = constraints[i];

            float jv = glm::dot(ci.va, ci.a->m_velocity) +
                       glm::dot(ci.wa, ci.a->m_angularVelocity) +
                       glm::dot(ci.vb, ci.b->m_velocity) +
                       glm::dot(ci.wb, ci.b->m_angularVelocity);

            lambda[i] = glm::clamp(pre[i][0] * (pre[i][1] - jv) * mu[i], pre[i][2], pre[i][3]); // lambda * dt
            if (ci.a->m_type == RigidbodyType::DYNAMIC)
            {
                ci.a->m_velocity += ci.a->m_invMass * lambda[i] * ci.va;
                ci.a->m_angularVelocity += ci.a->m_invInertia * (lambda[i] * ci.wa);
            }
            if (ci.b->m_type == RigidbodyType::DYNAMIC)
            {
                ci.b->m_velocity += ci.b->m_invMass * lambda[i] * ci.vb;
                ci.b->m_angularVelocity += ci.b->m_invInertia * (lambda[i] * ci.wb);
            }
            err = glm::max(err, glm::abs(last - lambda[i]));
        }
        if (iter > 0 && err < m_tol)
            break;
    }
}
} // namespace rbd3d