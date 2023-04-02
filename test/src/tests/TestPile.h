#ifndef TEST_VIEWER_TEST_PILE_H_
#define TEST_VIEWER_TEST_PILE_H_

#include "../TestBase.h"
#include "../RigidbodyRenderer.h"
#include "../Camera.h"

#include <rbd3d/rbd3d.h>
#include <array>

class TestPile : public TestBase
{
public:
    virtual void onUpdate(float) override;
    virtual void onRender(int viewportWidth, int viewportHeight) override;
    virtual void onImGuiRender() override;

    TestPile(Shader &);
    virtual ~TestPile() override;

    void addRandomRigidbody();

private:
    rbd3d::DynamicWorld m_world;

    static constexpr size_t maxRigidbodyCount = 200;

    size_t m_rigidbodyCount;
    rbd3d::Cuboid m_box[5];
    std::array<rbd3d::RigidbodyBase *, maxRigidbodyCount> m_rigidbodies;

    Camera m_camera;
    std::array<RigidbodyRenderer, 5 + maxRigidbodyCount> m_renderers;

    float m_updateDur;

    size_t m_frame;
};

#endif