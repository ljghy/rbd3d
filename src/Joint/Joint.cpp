#include <rbd3d/joint/Joint.h>

namespace rbd3d
{
void Joint::create(const JointCreateInfo &info)
{
    m_rigidbodyA = info.a;
    m_rigidbodyB = info.b;
    m_globalAnchor = info.anchor;
    m_globalRotation = info.rotation;
    m_translationLB = info.translationLB;
    m_translationUB = info.translationUB;
    m_rotationLB = info.rotationLB;
    m_rotationUB = info.rotationUB;
    setLocalTransform();
}

void Joint::addJointConstraint(size_t &constraintCount, std::vector<Constraint> &constraints, const Constraint &c) const
{
    if (constraintCount >= constraints.size())
        constraints.push_back(c);
    else
        constraints[constraintCount] = c;
    ++constraintCount;
}

void Joint::setLocalTransform()
{
    glm::quat invRotA = glm::conjugate(m_rigidbodyA->rotation());
    m_localAnchorA = invRotA * (m_globalAnchor - m_rigidbodyA->translation());
    m_localRotationA = invRotA * m_globalRotation;

    glm::quat invRotB = glm::conjugate(m_rigidbodyB->rotation());
    m_localAnchorB = invRotB * (m_globalAnchor - m_rigidbodyB->translation());
    m_localRotationB = invRotB * m_globalRotation;
}

void Joint::checkTranslationConstraints(size_t &constraintCount, std::vector<Constraint> &constraints) const
{
    const glm::quat rotA = m_rigidbodyA->rotation();
    const glm::vec3 transA = m_rigidbodyA->translation();
    glm::quat invRotA = glm::conjugate(rotA),
              invLocalRotA = glm::conjugate(m_localRotationA);
    glm::vec3 anchorBA =
        invLocalRotA * (invRotA * (m_rigidbodyB->rotation() * m_localAnchorB + m_rigidbodyB->translation() - transA) - m_localAnchorA);

    glm::vec3 norm(0.f), pos(0.f);
    for (int i = 0; i < 3; ++i)
    {
        if (anchorBA[i] < m_translationLB[i])
        {
            norm[i] = m_translationLB[i] - anchorBA[i];
            pos[i] = m_translationLB[i];
        }
        else if (anchorBA[i] > m_translationUB[i])
        {
            norm[i] = m_translationUB[i] - anchorBA[i];
            pos[i] = m_translationUB[i];
        }
    }
    float depth = glm::length(norm);
    if (depth > std::numeric_limits<float>::epsilon())
    {
        norm = rotA * (m_localRotationA * (norm / depth));
        pos = rotA * (m_localRotationA * pos + m_localAnchorA) + transA;
        addJointConstraint(constraintCount, constraints,
                           Constraint::jointTranslationConstraint(m_rigidbodyA, m_rigidbodyB, norm, depth, pos));
    }
}

void Joint::checkRotationConstraints(size_t &constraintCount, std::vector<Constraint> &constraints) const
{
    glm::quat globalRotA = m_rigidbodyA->rotation() * m_localRotationA,
              globalRotB = m_rigidbodyB->rotation() * m_localRotationB,
              relRot = glm::conjugate(globalRotA) * globalRotB;
    glm::vec3 angles = glm::eulerAngles(relRot);

    float sp = glm::sin(angles.x), cp = glm::cos(angles.x),
          cy = glm::cos(angles.y);

    glm::vec3 axes[]{
        {1.f, sp * sp / cy, cp * sp / cy},
        {0.f, cp, -sp},
        {0.f, sp / cy, cp / cy}};

    // glm::vec3 axes[]{
    //     {cy, sp * sp, cp * sp},
    //     {0.f, cp, -sp},
    //     {0.f, sp, cp}};

    for (int i = 0; i < 3; ++i)
    {
        if (angles[i] < m_rotationLB[i])
        {
            addJointConstraint(constraintCount, constraints,
                               Constraint::jointRotationConstraint(m_rigidbodyA, m_rigidbodyB, -axes[i], m_rotationLB[i] - angles[i]));
        }
        else if (angles[i] > m_rotationUB[i])
        {
            addJointConstraint(constraintCount, constraints,
                               Constraint::jointRotationConstraint(m_rigidbodyA, m_rigidbodyB, axes[i], angles[i] - m_rotationUB[i]));
        }
    }
}

} // namespace rbd3d