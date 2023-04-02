#ifndef TEST_VIEWER_TEST_MANAGER_H_
#define TEST_VIEWER_TEST_MANAGER_H_

#include "TestBase.h"

#include <vector>
#include <string>
#include <functional>

#include <opengl_framework/Shader.h>

class TestManager
{
public:
    template <typename TestType>
    static void registTest(const std::string &testname)
    {
        m_testList.push_back(std::make_pair(testname, []()
                                            { return new TestType(m_shader); }));
    }

    static void initShader();
    static void onUpdate(float deltaTime);
    static void onRender(int viewportWidth, int viewportHeight);
    static void onImGuiRender();

    static void cleanup();

private:
    TestManager(){};

    static TestBase *m_currentTest;
    static std::vector<std::pair<std::string, std::function<TestBase *()>>> m_testList;

    static Shader m_shader;
};

#endif