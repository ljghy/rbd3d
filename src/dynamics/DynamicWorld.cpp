#include <rbd3d/dynamics/DynamicWorld.h>

#include <algorithm>
#include <limits>

#include <chrono>

namespace rbd3d
{

DynamicWorld::DynamicWorld(std::unique_ptr<SolverBase> &&_solver,
                           const glm::vec3 &_gravity,
                           uint32_t _substeps)
    : m_solver(std::move(_solver)),
      m_gravity(_gravity),
      m_substeps(glm::max(1u, _substeps))

{
}

float DynamicWorld::update(float deltaTime)
{
    auto start = std::chrono::high_resolution_clock::now();
    float dt = deltaTime / m_substeps;

    for (uint32_t i = 0; i < m_substeps; ++i)
    {
        clearForce();
        applyExtForce();

        m_constraints.clear();
        detectCollision();

        solveConstraints(dt);

        integrate(dt);
    }
    auto end = std::chrono::high_resolution_clock::now();
    float dur = static_cast<std::chrono::duration<float>>(end - start).count();
    return dur;
}

void DynamicWorld::solveConstraints(float deltaTime)
{
    m_solver->solve(m_constraints, deltaTime);
}

void DynamicWorld::clearForce()
{
    for (auto &r : m_rigidbodyList)
    {
        r->setForce(glm::vec3(0.f));
        r->setTorque(glm::vec3(0.f));
    }
}

void DynamicWorld::applyExtForce()
{
    applyGravity();
}

void DynamicWorld::applyGravity()
{
    for (auto &r : m_rigidbodyList)
    {
        r->setForce(r->force() + m_gravity * r->mass());
    }
}

void DynamicWorld::detectCollision()
{
    for (size_t i = 1; i < m_rigidbodyList.size(); ++i)
    {
        for (size_t j = 0; j < i; ++j)
        {
            auto info = collisionInfo(*m_rigidbodyList[i], *m_rigidbodyList[j]);
            if (info.happen)
            {
                m_constraints.push_back(std::make_unique<ContactConstraint>(info));

                if (info.rigidbodyA->friction() * info.rigidbodyB->friction() > 0.f)
                {
                    glm::vec3 an = glm::abs(info.normal), t1, t2;
                    if (an.x > an.y && an.x > an.z)
                        t1 = glm::normalize(glm::vec3(-info.normal.y / info.normal.x, 1.f, 0.f));
                    else
                        t1 = glm::normalize(an.y > an.z ? glm::vec3(1.f, -info.normal.x / info.normal.y, 0.f) : glm::vec3(1.f, 0.f, -info.normal.x / info.normal.z));
                    t2 = glm::cross(info.normal, t1);
                    m_constraints.push_back(std::make_unique<FrictionConstraint>(info, t1));
                    m_constraints.push_back(std::make_unique<FrictionConstraint>(info, t2));
                }
            }
        }
    }
}

void DynamicWorld::integrate(float deltaTime)
{
    for (auto &r : m_rigidbodyList)
    {
        r->setVelocity(r->velocity() + r->invMass() * r->force() * deltaTime);
        r->setAngularVelocity(r->angularVelocity() + r->invInertia() * r->torque() * deltaTime);
        r->translate(r->velocity() * deltaTime);
        r->setRotation(glm::normalize(r->rotation() + glm::quat(0.f, r->angularVelocity()) * r->rotation() * deltaTime));
    }
}

void DynamicWorld::addRigidbody(RigidbodyBase &rigidbody)
{
    assert(std::find(m_rigidbodyList.begin(), m_rigidbodyList.end(), &rigidbody) == m_rigidbodyList.end());
    m_rigidbodyList.push_back(&rigidbody);
}

} // namespace rbd3d