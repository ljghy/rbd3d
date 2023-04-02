#include "TestManager.h"
#include <imgui.h>

TestBase *TestManager::m_currentTest = nullptr;
std::vector<std::pair<std::string, std::function<TestBase *()>>> TestManager::m_testList{};
Shader TestManager::m_shader{};

void TestManager::initShader()
{
    const char *vertShaderSrc =
        "#version 330 core\n"
        "layout (location = 0) in vec3 aPos;\n"
        "layout (location = 1) in vec3 aNorm;\n"
        "uniform mat4 u_model;\n"
        "uniform mat4 u_MVP;\n"
        "out vec3 norm;\n"
        "void main()\n"
        "{\n"
        "    gl_Position = u_MVP * vec4(aPos, 1.0);\n"
        "    norm = normalize(mat3(transpose(inverse(u_model))) * aNorm);\n"
        "}";
    const char *fragShaderSrc =
        "#version 330 core\n"
        "in vec3 norm;\n"
        "out vec4 color;\n"
        "uniform vec3 u_diffuse;\n"
        "\n"
        "void main()\n"
        "{\n"
        "    vec3 lightDir = normalize(vec3(1.5, 2.0, 1.0));\n"
        "    vec3 lightColor= vec3(0.8, 0.8, 0.8);\n"
        "    vec3 ambient = vec3(0.5, 0.5, 0.5);\n"
        "    color = vec4(u_diffuse * (max(dot(norm, lightDir), 0.0) * lightColor + ambient), 1.0);\n"
        "}";
    m_shader.create(vertShaderSrc, fragShaderSrc);
}

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