#include "TestJoint.h"
#include <imgui.h>

TestJoint::TestJoint(Shader &shader)
    : TestBase(shader)
{
    m_gravity = glm::vec3(0.f, -9.8f, 0.f);
    m_cube.setSize(glm::vec3(2.f, 2.f, 2.f));
    m_cubePos = glm::vec3(0.f, -1.f, 0.f);
    m_cube.setTranslation(m_cubePos);
    m_cube.setType(rbd3d::RigidbodyType::STATIC);
    m_cube.setCollisionGroup(1);
    m_world.addRigidbody(m_cube);
    m_cubeRenderer.create(m_cube);

    for (size_t i = 0; i < height; ++i)
        for (size_t j = 0; j < width; ++j)
        {
            m_nodes[i][j].setCollisionFilter(0x1);
            m_nodes[i][j].setTranslation(glm::vec3(0.5f * (j - width * 0.5f), 5.f - 0.5f * i, 0.f));
            m_nodes[i][j].setSize(glm::vec3(0.6f, 0.6f, 0.2f));
            if (i > 0)
            {
                rbd3d::JointCreateInfo info{&m_nodes[i - 1][j], &m_nodes[i][j],
                                            glm::vec3(0.5f * (j - width * 0.5f), 5.25f - 0.5f * i, 0.f),
                                            glm::quat(1.f, 0.f, 0.f, 0.f),
                                            glm::vec3(0.f),
                                            glm::vec3(0.f),
                                            glm::radians(glm::vec3(-5.f)),
                                            glm::radians(glm::vec3(5.f))};
                m_jointsV[i - 1][j].create(info);
                m_world.addJoint(m_jointsV[i - 1][j]);
                if (j > 0)
                {
                    rbd3d::JointCreateInfo info{&m_nodes[i][j - 1], &m_nodes[i][j],
                                                glm::vec3(0.5f * (j - width * 0.5f - 0.5f), 5.f - 0.5f * i, 0.f),
                                                glm::quat(1.f, 0.f, 0.f, 0.f),
                                                glm::vec3(-0.f, 0.f, 0.f),
                                                glm::vec3(0.f, 0.f, 0.f),
                                                glm::vec3(-1.f),
                                                glm::vec3(1.f)};
                    m_jointsH[i - 1][j - 1].create(info);
                    m_world.addJoint(m_jointsH[i - 1][j - 1]);
                }
            }
            else
            {
                m_nodes[i][j].setType(rbd3d::RigidbodyType::STATIC);
            }

            m_world.addRigidbody(m_nodes[i][j]);
            m_renderers[i][j].create(m_nodes[i][j]);
        }
}

void TestJoint::onUpdate(float deltaTime)
{
    m_world.setGravity(m_gravity);
    m_cube.setTranslation(m_cubePos);

    m_updateDur = m_world.fixedUpdate(deltaTime, 1.f / 60.f);

    const float vel = 5.f;
    if (ImGui::IsKeyDown(ImGuiKey_W))
        m_camera.translate(deltaTime * m_camera.dir * vel);
    if (ImGui::IsKeyDown(ImGuiKey_S))
        m_camera.translate(-deltaTime * m_camera.dir * vel);
    if (ImGui::IsKeyDown(ImGuiKey_A))
        m_camera.translate(-deltaTime * m_camera.right * vel);
    if (ImGui::IsKeyDown(ImGuiKey_D))
        m_camera.translate(deltaTime * m_camera.right * vel);
    if (ImGui::IsKeyDown(ImGuiKey_Space))
        m_camera.translate(deltaTime * m_camera.up * vel);
    if (ImGui::IsKeyDown(ImGuiKey_LeftShift))
        m_camera.translate(-deltaTime * m_camera.up * vel);

    const float sensitivity = 20.f;
    ImGuiIO &io = ImGui::GetIO();
    ImVec2 mouse_delta = io.MouseDelta;
    if (ImGui::IsMouseDragging(1))
        m_camera.rotate(sensitivity * mouse_delta.x * deltaTime, -sensitivity * mouse_delta.y * deltaTime);

    m_camera.yfov -= 5.f * io.MouseWheel;
    m_camera.yfov = glm::clamp(m_camera.yfov, 1.f, 120.f);
}

void TestJoint::onRender(int viewportWidth, int viewportHeight)
{
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_MULTISAMPLE);

    glViewport(0, 0, viewportWidth, viewportHeight);
    glClearColor(0.9f, 0.9f, 0.9f, 1.f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    for (size_t i = 0; i < height; ++i)
        for (size_t j = 0; j < width; ++j)
            m_renderers[i][j].render(m_baseShader, viewportWidth, viewportHeight, m_camera, glm::vec3(0.2f, 0.8f, 0.3f));
    m_cubeRenderer.render(m_baseShader, viewportWidth, viewportHeight, m_camera, glm::vec3(0.2f, 0.3f, 0.8f));
}

void TestJoint::onImGuiRender()
{
    ImGui::SliderFloat3("Gravity", &m_gravity[0], -10.f, 10.f);
    ImGui::SliderFloat3("Cube Pos", &m_cubePos[0], -5.f, 5.f);
    ImGui::Text("Physics: %.2f ms", m_updateDur * 1e3f);
}