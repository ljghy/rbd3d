#include "TestCollisionDetection.h"
#include <imgui.h>

TestCollisionDetection::TestCollisionDetection()
    : m_cube(glm::vec3(1.f)),
      m_sphere(0.5f)
{
    m_sphere.setTranslation(glm::vec3(2.f, 0.f, 0.f));
    m_cubeRenderer.create(m_cube);
    m_sphereRenderer.create(m_sphere);
}

void TestCollisionDetection::onUpdate(float deltaTime)
{
    m_collisionInfo = rbd3d::collisionInfo(m_cube, m_sphere);

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

void TestCollisionDetection::onRender(int viewportWidth, int viewportHeight)
{
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_MULTISAMPLE);

    glViewport(0, 0, viewportWidth, viewportHeight);
    glClearColor(0.9f, 0.9f, 0.9f, 1.f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glm::vec3 color = m_collisionInfo.happen ? glm::vec3(1.0f, 0.6f, 0.2f) : glm::vec3(0.2f, 0.6f, 1.0f);
    m_cubeRenderer.render(viewportWidth, viewportHeight, m_camera, color);
    m_sphereRenderer.render(viewportWidth, viewportHeight, m_camera, color);
}

void TestCollisionDetection::onImGuiRender()
{
    static glm::vec3 cubePos(0.f, 0.f, 0.f);
    ImGui::SliderFloat3("Cube Position", &cubePos.x, -2.f, 2.f);
    m_cube.setTranslation(cubePos);

    if (m_collisionInfo.happen)
    {
        auto &n = m_collisionInfo.normal;
        ImGui::Text("Normal: (%.3f, %.3f, %.3f), Depth: %.3f", n.x, n.y, n.z, m_collisionInfo.depth);
    }
}