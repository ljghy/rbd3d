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

    rbd3d::Cuboid m_cube[10];
    rbd3d::Plane m_ground;

    RigidbodyRenderer m_cubeRenderer[10];
    RigidbodyRenderer m_groundRenderer;

    Camera m_camera;

    float m_updateDur;

};

#endif