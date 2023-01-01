#ifndef RBD3D_JOINT_JOINT_H_
#define RBD3D_JOINT_JOINT_H_

#include <rbd3d/rigidbody/RigidbodyBase.h>
#include <rbd3d/dynamics/Constraint.h>

#include <vector>

namespace rbd3d
{

struct JointCreateInfo
{
    RigidbodyBase *a;
    RigidbodyBase *b;
    glm::vec3 anchor = glm::vec3(0.f);
    glm::quat rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    glm::vec3 translationLB = glm::vec3(0.f);
    glm::vec3 translationUB = glm::vec3(0.f);
    glm::vec3 rotationLB = glm::vec3(0.f);
    glm::vec3 rotationUB = glm::vec3(0.f);
};

class Joint
{
public:
    Joint() = default;
    void create(const JointCreateInfo &info);
    void checkTranslationConstraints(size_t &, std::vector<Constraint> &) const;
    void checkRotationConstraints(size_t &, std::vector<Constraint> &) const;

private:
    void setLocalTransform();
    void addJointConstraint(size_t &, std::vector<Constraint> &, const Constraint &) const;

private:
    RigidbodyBase *m_rigidbodyA;
    RigidbodyBase *m_rigidbodyB;

    glm::vec3 m_globalAnchor;
    glm::quat m_globalRotation;

    glm::vec3 m_localAnchorA;
    glm::quat m_localRotationA;

    glm::vec3 m_localAnchorB;
    glm::quat m_localRotationB;

    glm::vec3 m_rotationLB;
    glm::vec3 m_rotationUB;
    glm::vec3 m_translationLB;
    glm::vec3 m_translationUB;
};

} // namespace rbd3d

#endif
