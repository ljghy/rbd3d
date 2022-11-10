#ifndef TEST_VIEWER_TEST_COLLISION_WITH_GROUND_H_
#define TEST_VIEWER_TEST_COLLISION_WITH_GROUND_H_

#include "../TestBase.h"
#include "../RigidbodyRenderer.h"
#include "../Camera.h"

#include <rbd3d/rbd3d.h>

class TestCollisionWithGround : public TestBase
{
public:
    virtual void onUpdate(float) override;
    virtual void onRender(int viewportWidth, int viewportHeight) override;
    virtual void onImGuiRender() override;

    TestCollisionWithGround();

private:
    rbd3d::DynamicWorld m_world;

    rbd3d::Cuboid m_cube[3];
    rbd3d::Sphere m_sphere;
    rbd3d::Plane m_ground;

    RigidbodyRenderer m_cubeRenderer[3];
    RigidbodyRenderer m_sphereRenderer;
    RigidbodyRenderer m_groundRenderer;

    Camera m_camera;
};

#endif