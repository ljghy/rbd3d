#ifndef TEST_VIEWER_TEST_JOINT_H_
#define TEST_VIEWER_TEST_JOINT_H_

#include "../TestBase.h"
#include "../RigidbodyRenderer.h"
#include "../Camera.h"

#include <rbd3d/rbd3d.h>

class TestJoint : public TestBase
{
public:
    virtual void onUpdate(float) override;
    virtual void onRender(int viewportWidth, int viewportHeight) override;
    virtual void onImGuiRender() override;

    TestJoint(Shader &);

private:
    rbd3d::DynamicWorld m_world;

    static constexpr size_t width = 8, height = 8;

    rbd3d::Cuboid m_cube;
    rbd3d::Cuboid m_nodes[height][width];
    rbd3d::Joint m_jointsV[height - 1][width];
    rbd3d::Joint m_jointsH[height - 1][width - 1];

    RigidbodyRenderer m_cubeRenderer;
    RigidbodyRenderer m_renderers[height][width];

    Camera m_camera;

    float m_updateDur;

    glm::vec3 m_gravity;
    glm::vec3 m_cubePos;
};

#endif