#include <rbd3d/dynamics/DynamicWorld.h>

#include <chrono>

namespace rbd3d
{

DynamicWorld::DynamicWorld(const glm::vec3 &_gravity,
                           float solverBias,
                           float solverTol,
                           uint32_t solverIters)
    : m_gravity(_gravity),
      m_solver(solverBias, solverTol, solverIters),
      m_accumulator(0.f)
{
}

void DynamicWorld::resetAccumulator()
{
    m_accumulator = 0.f;
}

float DynamicWorld::fixedUpdate(float deltaTime, float fixedDeltaTime)
{
    float dur = 0.f;
    m_accumulator += deltaTime;
    while (m_accumulator >= fixedDeltaTime)
    {
        dur += update(fixedDeltaTime);
        m_accumulator -= fixedDeltaTime;
    }
    return dur;
}

float DynamicWorld::update(float deltaTime)
{
    auto start = std::chrono::high_resolution_clock::now();

    clearForce();
    applyExtForce();

    integrate(deltaTime);
    updateBVH();

    m_constraintCount = 0;
    detectCollision();
    checkJoints();
    solveConstraints(deltaTime);

    auto end = std::chrono::high_resolution_clock::now();
    float dur = static_cast<std::chrono::duration<float>>(end - start).count();
    return dur;
}

void DynamicWorld::solveConstraints(float deltaTime)
{
    m_solver.solve(m_constraintCount, m_constraints, deltaTime);
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

void DynamicWorld::addConstraint(const Constraint &c)
{
    if (m_constraintCount >= m_constraints.size())
        m_constraints.push_back(c);
    else
        m_constraints[m_constraintCount] = c;
    ++m_constraintCount;
}

void DynamicWorld::detectCollision()
{
    std::vector<RigidbodyBase *> possibleCollisions;
    possibleCollisions.reserve(m_rigidbodyList.size());

    for (auto &rigidbodyA : m_rigidbodyList)
    {
        possibleCollisions.clear();
        m_BVH.detectCollision(rigidbodyA, possibleCollisions);

        for (auto &rigidbodyB : possibleCollisions)
        {
            auto manifold = collision(*rigidbodyA, *rigidbodyB);
            for (int k = 0; k < manifold.pointCount; ++k)
            {
                addConstraint(Constraint::contactConstraint(rigidbodyA, rigidbodyB,
                                                            manifold.normal,
                                                            manifold.contactPoints[k].depth,
                                                            manifold.contactPoints[k].position));

                if (rigidbodyA->friction() * rigidbodyB->friction() > 0.f)
                {
                    glm::vec3 t1, t2;
                    if (glm::abs(manifold.normal.x) >= 0.5f)
                        t1 = glm::vec3(manifold.normal.y, -manifold.normal.x, 0.f);
                    else
                        t1 = glm::vec3(0.f, manifold.normal.z, -manifold.normal.y);

                    t1 = glm::normalize(t1);
                    t2 = glm::cross(manifold.normal, t1);
                    addConstraint(Constraint::frictionConstraint(rigidbodyA, rigidbodyB,
                                                                 manifold.contactPoints[k].position, t1));
                    addConstraint(Constraint::frictionConstraint(rigidbodyA, rigidbodyB,
                                                                 manifold.contactPoints[k].position, t2));
                }
            }
        }
    }
}

void DynamicWorld::checkJoints()
{
    for (const auto &j : m_jointList)
    {
        j->checkTranslationConstraints(m_constraintCount, m_constraints);
        j->checkRotationConstraints(m_constraintCount, m_constraints);
    }
}

void DynamicWorld::integrate(float deltaTime)
{
    for (RigidbodyBase *r : m_rigidbodyList)
    {
        if (r->type() != RigidbodyType::STATIC)
        {
            r->setVelocity(r->velocity() + r->invMass() * r->force() * deltaTime);
            r->setAngularVelocity(r->angularVelocity() + r->invInertia() * r->torque() * deltaTime);
            r->translate(r->velocity() * deltaTime);
            r->setRotation(glm::normalize(r->rotation() + glm::quat(0.f, 0.5f * r->angularVelocity()) * r->rotation() * deltaTime));
        }
    }
}

void DynamicWorld::addRigidbody(RigidbodyBase &rigidbody)
{
    m_rigidbodyList.push_back(&rigidbody);
    m_BVH.insertLeaf(rigidbody.enlargedAABB(enlargedAABBScale), &rigidbody);
}

void DynamicWorld::addJoint(Joint &joint)
{
    m_jointList.push_back(&joint);
}

void DynamicWorld::updateBVH()
{
    m_BVH.update();
}

void DynamicWorld::setGravity(const glm::vec3 &g)
{
    m_gravity = g;
}

} // namespace rbd3d