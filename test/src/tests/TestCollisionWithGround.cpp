#include "TestCollisionWithGround.h"
#include <imgui.h>

TestCollisionWithGround::TestCollisionWithGround()
    : m_cube{{glm::vec3(1.8f), 10.f}, glm::vec3(1.f), glm::vec3(0.5f)},
      m_sphere(0.5f),
      m_ground(glm::vec3(0.0f, 1.f, 0.f))
{
    m_cube[0].setTranslation(glm::vec3(0.f, 1.f, 0.f));
    m_cube[1].setTranslation(glm::vec3(0.f, 2.6f, 0.f));
    m_cube[2].setTranslation(glm::vec3(0.f, 3.5f, 0.f));

    m_sphere.setTranslation(glm::vec3(0.f, 6.f, 0.f));
    for (size_t i = 0; i < 3; ++i)
        m_cubeRenderer[i].create(m_cube[i]);
    m_sphereRenderer.create(m_sphere);
    m_groundRenderer.create(m_ground);

    m_world.addRigidbody(m_sphere);
    for (auto &c : m_cube)
        m_world.addRigidbody(c);
    m_world.addRigidbody(m_ground);
}

void TestCollisionWithGround::onUpdate(float deltaTime)
{
    m_world.fixedUpdate(deltaTime, 0.02f);

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

void TestCollisionWithGround::onRender(int viewportWidth, int viewportHeight)
{
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_MULTISAMPLE);

    glViewport(0, 0, viewportWidth, viewportHeight);
    glClearColor(0.9f, 0.9f, 0.9f, 1.f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    for (auto &cr : m_cubeRenderer)
        cr.render(viewportWidth, viewportHeight, m_camera, glm::vec3(1.f, 0.6f, 0.2f));
    m_sphereRenderer.render(viewportWidth, viewportHeight, m_camera, glm::vec3(1.f, 0.6f, 0.2f));
    m_groundRenderer.render(viewportWidth, viewportHeight, m_camera, glm::vec3(0.1f, 0.3f, 0.5f));
}

void TestCollisionWithGround::onImGuiRender()
{
    static float groundFriction = 5.f;
    ImGui::SliderFloat("Ground Friction", &groundFriction, 0.f, 10.f);
    m_ground.setFriction(groundFriction);
}