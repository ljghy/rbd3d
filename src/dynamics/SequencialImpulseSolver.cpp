#include <rbd3d/dynamics/SequencialImpulseSolver.h>

namespace rbd3d
{
void SequencialImpulseSolver::solve(const std::vector<std::unique_ptr<ConstraintBase>> &constraints, float deltaTime)
{
    size_t c = constraints.size();

    mu.resize(c);
    for (size_t i = 0; i < c; ++i)
    {
        const auto &ci = constraints[i];
        mu[i] = ci->a->invMass() * glm::dot(ci->va, ci->va) +
                glm::dot(ci->wa, ci->a->invInertia() * ci->wa) +
                ci->b->invMass() * glm::dot(ci->vb, ci->vb) +
                glm::dot(ci->wb, ci->b->invInertia() * ci->wb);
        mu[i] = glm::abs(mu[i]) > std::numeric_limits<float>::min() ? 1.f / mu[i] : 0.f;
    }

    lambda.resize(c);
    float idt = 1.f / deltaTime;
    for (uint32_t iter = 0; iter < m_iters; ++iter)
    {
        float err = 0.f;
        for (size_t i = 0; i < c; ++i)
        {
            if (mu[i] == 0)
                continue;
            float last = lambda[i];
            const auto &ci = constraints[i];

            float jv = glm::dot(ci->va, ci->a->m_velocity) +
                       glm::dot(ci->wa, ci->a->m_angularVelocity) +
                       glm::dot(ci->vb, ci->b->m_velocity) +
                       glm::dot(ci->wb, ci->b->m_angularVelocity);
            lambda[i] = (1.f + ci->res) * glm::clamp((m_bias * idt * ci->vel - jv) * mu[i], ci->bound.x * deltaTime, ci->bound.y * deltaTime); // lambda * dt
            ci->a->m_velocity += ci->a->m_invMass * lambda[i] * ci->va;
            ci->a->m_angularVelocity += ci->a->m_invInertia * (lambda[i] * ci->wa);
            ci->b->m_velocity += ci->b->m_invMass * lambda[i] * ci->vb;
            ci->b->m_angularVelocity += ci->b->m_invInertia * (lambda[i] * ci->wb);

            err = glm::max(err, glm::abs(last - lambda[i]));
        }
        if (iter > 0 && err < m_tol)
            break;
    }
}
} // namespace rbd3d