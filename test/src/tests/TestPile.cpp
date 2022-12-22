#include "TestPile.h"
#include <imgui.h>
#include <cstdlib>
#include <ctime>

TestPile::TestPile()
    : m_box{
          {glm::vec3(16.f, 0.5f, 10.f)}, // bottom
          {glm::vec3(16.f, 4.f, 0.5f)},  // front
          {glm::vec3(16.f, 4.f, 0.5f)},  // back
          {glm::vec3(0.5f, 4.f, 10.f)},  // left
          {glm::vec3(0.5f, 4.f, 10.f)}   // right
      }
{
    m_camera.pos = glm::vec3(0.f, 16.f, 16.f);

    m_box[0].setTranslation(glm::vec3(0.f, 0.25f, 0.f));
    m_box[1].setTranslation(glm::vec3(0.f, 2.f, 4.75f));
    m_box[2].setTranslation(glm::vec3(0.f, 2.f, -4.75f));
    m_box[3].setTranslation(glm::vec3(7.75f, 2.f, 0.f));
    m_box[4].setTranslation(glm::vec3(-7.75f, 2.f, 0.f));
    for (size_t i = 0; i < 5; ++i)
    {
        m_box[i].setType(rbd3d::RigidbodyType::STATIC);
        m_world.addRigidbody(m_box[i]);
        m_renderers[i].create(m_box[i]);
    }
    m_rigidbodyCount = 5;
    m_frame = 0;
    srand(time(0));
}

TestPile::~TestPile()
{
    for (size_t i = 0; i < m_rigidbodyCount - 5; ++i)
        delete m_rigidbodies[i];
}

void TestPile::addRandomRigidbody()
{
    if (m_rigidbodyCount < maxRigidbodyCount && m_frame % 5 == 0)
    {
        float x = float(rand()) / RAND_MAX * 10.f - 5.f,
              z = float(rand()) / RAND_MAX * 6.f - 3.f;
        float rx = float(rand()) / RAND_MAX * 360.f - 180.f,
              ry = float(rand()) / RAND_MAX * 360.f - 180.f,
              rz = float(rand()) / RAND_MAX * 360.f - 180.f;
        glm::vec3 pos(x, 20.f, z);
        glm::quat rot(glm::radians(glm::vec3(rx, ry, rz)));

        switch (rand() % 3)
        {
        case 0:
        {
            float r = 0.5f * float(rand()) / RAND_MAX + 0.5f;
            rbd3d::Sphere *s = new rbd3d::Sphere(r);
            s->setTranslation(pos);
            s->setRotation(rot);
            m_world.addRigidbody(*s);
            m_renderers[m_rigidbodyCount].create(*s);
            m_rigidbodies[m_rigidbodyCount - 5] = s;
            ++m_rigidbodyCount;
        }
        break;
        case 1:
        {
            float r = 0.25f * float(rand()) / RAND_MAX + 0.25f,
                  h = 0.5f * float(rand()) / RAND_MAX + 0.5f;
            rbd3d::Capsule *c = new rbd3d::Capsule(r, h);
            c->setTranslation(pos);
            c->setRotation(rot);
            m_world.addRigidbody(*c);
            m_renderers[m_rigidbodyCount].create(*c);
            m_rigidbodies[m_rigidbodyCount - 5] = c;
            ++m_rigidbodyCount;
        }
        break;
        case 2:
        {
            float sx = 0.5f * float(rand()) / RAND_MAX + 0.5f,
                  sy = 0.5f * float(rand()) / RAND_MAX + 0.5f,
                  sz = 0.5f * float(rand()) / RAND_MAX + 0.5f;
            rbd3d::Cuboid *c = new rbd3d::Cuboid(glm::vec3(sx, sy, sz));
            c->setTranslation(pos);
            c->setRotation(rot);
            m_world.addRigidbody(*c);
            m_renderers[m_rigidbodyCount].create(*c);
            m_rigidbodies[m_rigidbodyCount - 5] = c;
            ++m_rigidbodyCount;
        }
        break;
        }
    }
}

void TestPile::onUpdate(float deltaTime)
{
    addRandomRigidbody();
    m_updateDur = m_world.fixedUpdate(deltaTime, 1.f / 60.f);
    ++m_frame;

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

void TestPile::onRender(int viewportWidth, int viewportHeight)
{
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_MULTISAMPLE);

    glViewport(0, 0, viewportWidth, viewportHeight);
    glClearColor(0.9f, 0.9f, 0.9f, 1.f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    for (size_t i = 0; i < 5; ++i)
        m_renderers[i].render(viewportWidth, viewportHeight, m_camera, glm::vec3(0.2f, 0.3f, 0.8f));
    for (size_t i = 5; i < m_rigidbodyCount; ++i)
    {
        auto r = m_rigidbodies[i - 5];
        float Ek = 0.5f * r->mass() * glm::dot(r->velocity(), r->velocity()) +
                   0.5f * glm::dot(r->angularVelocity(), r->inertia() * r->angularVelocity());
        float red = 1.f - glm::exp(-0.1f * Ek);
        m_renderers[i].render(viewportWidth, viewportHeight, m_camera, glm::vec3(0.2f + 0.8f * red, 0.6f * (1.f - red), 0.2f));
    }
}

void TestPile::onImGuiRender()
{
    ImGui::Text("Rigidbodies: %d, Physics: %.2f ms", (int)m_rigidbodyCount, m_updateDur * 1e3f);
}