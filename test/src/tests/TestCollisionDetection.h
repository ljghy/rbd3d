#ifndef TEST_VIEWER_TEST_COLLISION_DETECTION_H_
#define TEST_VIEWER_TEST_COLLISION_DETECTION_H_

#include "../TestBase.h"
#include "../RigidbodyRenderer.h"
#include "../Camera.h"

#include <rbd3d/collision.h>
#include <rbd3d/collision/NarrowPhase.h>

class TestCollisionDetection : public TestBase
{
public:
    virtual void onUpdate(float) override;
    virtual void onRender(int viewportWidth, int viewportHeight) override;
    virtual void onImGuiRender() override;

    TestCollisionDetection(Shader &);

private:
    rbd3d::Capsule m_cube;
    rbd3d::Sphere m_sphere;
    rbd3d::Cuboid m_cube2;
    rbd3d::Capsule m_capsule;

    rbd3d::ContactManifold m_cm1, m_cm2;

    RigidbodyRenderer m_cubeRenderer;
    RigidbodyRenderer m_cubeRenderer2;
    RigidbodyRenderer m_sphereRenderer;
    RigidbodyRenderer m_capsuleRenderer;

    Shader m_pointShader;

    Camera m_camera;

    VertexBufferObject m_VBO;
    VertexArrayObject m_VAO;
};

#endif