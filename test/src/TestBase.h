#ifndef TEST_VIEWER_TEST_BASE_H_
#define TEST_VIEWER_TEST_BASE_H_

class TestBase
{
public:
    TestBase() = default;
    virtual ~TestBase() = default;

    virtual void onUpdate(float deltaTime) {}
    virtual void onRender(int viewportWidth, int viewportHeight) {}
    virtual void onImGuiRender() {}
};

#endif