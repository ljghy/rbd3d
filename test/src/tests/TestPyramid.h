#ifndef TEST_VIEWER_TEST_STACK_H_
#define TEST_VIEWER_TEST_STACK_H_

#include "../TestBase.h"
#include "../RigidbodyRenderer.h"
#include "../Camera.h"

#include <rbd3d/rbd3d.h>

class TestPyramid : public TestBase
{
public:
    virtual void onUpdate(float) override;
    virtual void onRender(int viewportWidth, int viewportHeight) override;
    virtual void onImGuiRender() override;

    TestPyramid();

private:
    rbd3d::DynamicWorld m_world;

    static constexpr uint32_t layers = 16;

    rbd3d::Cuboid m_cube[(layers + 1) * layers / 2];
    rbd3d::Cuboid m_ground;
    rbd3d::Cuboid m_wall;
    rbd3d::Sphere m_sphere;

    RigidbodyRenderer m_cubeRenderer[(layers + 1) * layers / 2];
    RigidbodyRenderer m_groundRenderer;
    RigidbodyRenderer m_wallRenderer;
    RigidbodyRenderer m_sphereRenderer;

    Camera m_camera;

    float m_updateDur;
};

#endif