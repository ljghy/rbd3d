#ifndef TEST_VIEWER_TEST_BASE_H_
#define TEST_VIEWER_TEST_BASE_H_

#include <opengl_framework/Shader.h>

class TestBase
{
public:
    TestBase(Shader &shader) : m_baseShader(shader) {}
    virtual ~TestBase() = default;

    virtual void onUpdate(float deltaTime) {}
    virtual void onRender(int viewportWidth, int viewportHeight) {}
    virtual void onImGuiRender() {}

protected:
    Shader &m_baseShader;
};

#endif