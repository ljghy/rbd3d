#include <rbd3d/dynamics/SISolver.h>

namespace rbd3d
{
void SISolver::solve(const std::vector<std::unique_ptr<ConstraintBase>> &constraints, float deltaTime)
{
    size_t c = constraints.size();

    J.resize(c, 4);
    bound.resize(c);
    for (size_t i = 0; i < c; ++i)
    {
        const auto &ci = constraints[i];

        J(i, 0) = ci->va();
        J(i, 1) = ci->wa();
        J(i, 2) = ci->vb();
        J(i, 3) = ci->wb();
        bound(i) = ci->bound() * deltaTime;
    }

    lambda.resize(c);

    float idt = 1.f / deltaTime;
    for (uint32_t iter = 0; iter < m_iters; ++iter)
    {
        float err = 0.f;
        for (size_t i = 0; i < c; ++i)
        {
            float last = lambda(i);
            const auto &ci = constraints[i];
            float mu = ci->a->invMass() * glm::dot(J(i, 0), J(i, 0)) +
                       glm::dot(J(i, 1), ci->a->invInertia() * J(i, 1)) +
                       ci->b->invMass() * glm::dot(J(i, 2), J(i, 2)) +
                       glm::dot(J(i, 3), ci->b->invInertia() * J(i, 3));
            float jv = glm::dot(J(i, 0), ci->a->velocity()) +
                       glm::dot(J(i, 1), ci->a->angularVelocity()) +
                       glm::dot(J(i, 2), ci->b->velocity()) +
                       glm::dot(J(i, 3), ci->b->angularVelocity());
            lambda(i) = glm::clamp((m_bias * idt * ci->vel() - jv) / mu, bound(i).x, bound(i).y); // lambda * dt

            ci->a->applyImpulse(lambda(i) * J(i, 0));
            ci->a->applyAngularImpulse(lambda(i) * J(i, 1));
            ci->b->applyImpulse(lambda(i) * J(i, 2));
            ci->b->applyAngularImpulse(lambda(i) * J(i, 3));

            err = glm::max(err, glm::abs(last - lambda(i)));
        }
        if (iter > 0 && err < m_tol)
            break;
    }
}
} // namespace rbd3d