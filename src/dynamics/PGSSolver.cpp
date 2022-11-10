#include <rbd3d/dynamics/PGSSolver.h>

namespace rbd3d
{
void PGSSolver::solve(const std::vector<std::unique_ptr<ConstraintBase>> &constraints, float deltaTime)
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
        bound(i) = ci->bound();
    }

    b.resize(c);
    float idt = 1.f / deltaTime;
    for (size_t i = 0; i < c; ++i)
    {
        const auto &ci = constraints[i];
        b(i) = ci->vel() * m_bias * (idt * idt) -
               (glm::dot(J(i, 0), ci->a->velocity() * idt + ci->a->invMass() * ci->a->force()) +
                glm::dot(J(i, 1), ci->a->angularVelocity() * idt + ci->a->invInertia() * ci->a->torque()) +
                glm::dot(J(i, 2), ci->b->velocity() * idt + ci->b->invMass() * ci->b->force()) +
                glm::dot(J(i, 3), ci->b->angularVelocity() * idt + ci->b->invInertia() * ci->b->torque()));
    }

    A.resize(c, c);
    for (size_t i = 0; i < c; ++i)
    {
        for (size_t j = 0; j <= i; ++j)
        {
            const auto &ci = constraints[i];
            const auto &cj = constraints[j];

            A(i, j) = 0;
            if (ci->a == cj->a)
                A(i, j) += ci->a->invMass() * glm::dot(J(i, 0), J(j, 0)) +
                           glm::dot(J(i, 1), ci->a->invInertia() * J(j, 1));
            if (ci->b == cj->b)
                A(i, j) += ci->b->invMass() * glm::dot(J(i, 2), J(j, 2)) +
                           glm::dot(J(i, 3), ci->b->invInertia() * J(j, 3));
            if (ci->a == cj->b)
                A(i, j) += ci->a->invMass() * glm::dot(J(i, 0), J(j, 2)) +
                           glm::dot(J(i, 1), ci->a->invInertia() * J(j, 3));
            if (ci->b == cj->a)
                A(i, j) += ci->b->invMass() * glm::dot(J(i, 2), J(j, 0)) +
                           glm::dot(J(i, 3), ci->b->invInertia() * J(j, 1));
        }
    }

    projectedGaussSeidel(c);

    for (size_t i = 0; i < c; ++i)
    {
        const auto &ci = constraints[i];
        ci->a->addForce(lambda(i) * J(i, 0));
        ci->a->addTorque(lambda(i) * J(i, 1));
        ci->b->addForce(lambda(i) * J(i, 2));
        ci->b->addTorque(lambda(i) * J(i, 3));
    }
}

void PGSSolver::projectedGaussSeidel(size_t c)
{
    lambda.resize(c);

    float delta;
    for (size_t iter = 0; iter < m_iters; ++iter)
    {
        delta = 0.f;
        for (size_t i = 0; i < c; ++i)
        {
            if (A(i, i) != 0.f)
            {
                float last = lambda(i);
                float l = b(i);

                for (size_t j = 0; j < i; ++j)
                    l -= A(i, j) * lambda(j);

                for (size_t j = i + 1; j < c; ++j)
                    l -= A(j, i) * lambda(j);
                lambda(i) = glm::clamp(l / A(i, i), bound(i).x, bound(i).y);

                delta = glm::max(delta, std::abs(lambda(i) - last));
            }
        }
        if (delta < m_tol)
            break;
    }
}

} // namespace rbd3d