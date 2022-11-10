#include "TestManager.h"
#include <imgui.h>

TestBase *TestManager::m_currentTest = nullptr;
std::vector<std::pair<std::string, std::function<TestBase *()>>> TestManager::m_testList{};

void TestManager::onUpdate(float deltaTime)
{
    if (m_currentTest != nullptr)
        m_currentTest->onUpdate(deltaTime);
}

void TestManager::onRender(int viewportWidth, int viewportHeight)
{
    if (m_currentTest != nullptr)
        m_currentTest->onRender(viewportWidth, viewportHeight);
}

void TestManager::onImGuiRender()
{
    ImGui::Begin("Tests");
    if (m_currentTest == nullptr)
    {
        for (auto &test : m_testList)
            if (ImGui::Button(test.first.c_str()))
                m_currentTest = test.second();
    }
    else
    {
        if (ImGui::Button("<-"))
        {
            delete m_currentTest;
            m_currentTest = nullptr;
        }
        else
            m_currentTest->onImGuiRender();
    }
    ImGui::End();
}

void TestManager::cleanup()
{
    delete m_currentTest;
}