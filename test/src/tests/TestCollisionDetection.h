#ifndef TEST_VIEWER_TEST_COLLISION_DETECTION_H_
#define TEST_VIEWER_TEST_COLLISION_DETECTION_H_

#include "../TestBase.h"
#include "../RigidbodyRenderer.h"
#include "../Camera.h"

#include <rbd3d/collision.h>

class TestCollisionDetection : public TestBase
{
public:
    virtual void onUpdate(float) override;
    virtual void onRender(int viewportWidth, int viewportHeight) override;
    virtual void onImGuiRender() override;

    TestCollisionDetection();

private:
    rbd3d::Cuboid m_cube;
    rbd3d::Sphere m_sphere;

    rbd3d::CollisionInfo m_collisionInfo;

    RigidbodyRenderer m_cubeRenderer;
    RigidbodyRenderer m_sphereRenderer;

    Camera m_camera;
};

#endif