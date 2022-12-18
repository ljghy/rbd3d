#include "TestPyramid.h"
#include <imgui.h>

TestPyramid::TestPyramid()
    : m_ground(glm::vec3(40.f, 0.5f, 20.f), 0.f, 0.5f, 0.5f, glm::vec3(0.f, -0.25f, 0.f)),
      m_wall(glm::vec3(40.f, 20.f, 0.5f), 0.f, 0.5f, 0.5f, glm::vec3(0.f, 10.f, -5.f)),
      m_sphere(0.5f, 10.f)
{
    m_wall.setType(rbd3d::RigidbodyType::STATIC);
    m_ground.setType(rbd3d::RigidbodyType::STATIC);
    for (int i = 0; i < layers; ++i)
        for (int j = 0; j <= i; ++j)
        {
            int k = (i + 1) * i / 2 + j;
            m_cube[k].setSize(glm::vec3(0.9f, 0.5f, 0.7f));
            m_cube[k].setTranslation(glm::vec3(-0.5f * i + j, 0.5f * (layers - i) - 0.25f, 0.f));
            m_cubeRenderer[k].create(m_cube[k]);
        }
    m_sphere.setTranslation(glm::vec3(0.f, 5.f, 5.f));
    m_sphere.setVelocity(glm::vec3(0.f, 0.f, -20.f));

    m_groundRenderer.create(m_ground);
    m_wallRenderer.create(m_wall);
    m_sphereRenderer.create(m_sphere);

    m_world.addRigidbody(m_ground);
    m_world.addRigidbody(m_wall);
    for (auto &c : m_cube)
        m_world.addRigidbody(c);
    m_world.addRigidbody(m_sphere);

    m_camera.pos = glm::vec3(8.f, 15.f, 15.f);
    m_camera.rotate(-30.f, 0.f);
}

void TestPyramid::onUpdate(float deltaTime)
{
    m_updateDur = m_world.fixedUpdate(deltaTime, 0.02f);

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

void TestPyramid::onRender(int viewportWidth, int viewportHeight)
{
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_MULTISAMPLE);

    glViewport(0, 0, viewportWidth, viewportHeight);
    glClearColor(0.9f, 0.9f, 0.9f, 1.f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    for (int i = 0; i < layers; ++i)
        for (int j = 0; j <= i; ++j)
        {
            int k = (i + 1) * i / 2 + j;
            float Ek = 0.5f * m_cube[k].mass() * glm::dot(m_cube[k].velocity(), m_cube[k].velocity()) +
                       0.5f * glm::dot(m_cube[k].angularVelocity(), m_cube[k].inertia() * m_cube[k].angularVelocity());
            float r = 1.f - glm::exp(-Ek);
            m_cubeRenderer[k].render(viewportWidth, viewportHeight, m_camera, glm::vec3(0.2f + 0.8f * r, 0.6f * (1.f - r), 0.2f));
        }
    m_groundRenderer.render(viewportWidth, viewportHeight, m_camera, glm::vec3(0.1f, 0.3f, 0.5f));
    m_wallRenderer.render(viewportWidth, viewportHeight, m_camera, glm::vec3(0.1f, 0.3f, 0.5f));
    m_sphereRenderer.render(viewportWidth, viewportHeight, m_camera, glm::vec3(0.8f, 1.f, 0.2f));
}

void TestPyramid::onImGuiRender()
{
    static float groundFriction = 5.f;
    ImGui::SliderFloat("Ground Friction", &groundFriction, 0.f, 10.f);
    m_ground.setFriction(groundFriction);
    ImGui::Text("Physics: %.2f ms", m_updateDur * 1e3f);
}