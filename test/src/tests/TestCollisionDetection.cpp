#include "TestCollisionDetection.h"
#include <imgui.h>

TestCollisionDetection::TestCollisionDetection()
    : m_cube(glm::vec3(0.75f)),
      m_cube2(glm::vec3(1.f, 0.8f, 0.5f)),
      m_sphere(1.f)
{
    m_cubeRenderer.create(m_cube);
    m_cube2.setTranslation(glm::vec3(2.f, 0.f, 0.f));
    m_cubeRenderer2.create(m_cube2);
    m_sphereRenderer.create(m_sphere);

    const char *vertSrc = {
        "#version 330 core\n"
        "layout (location = 0) in vec3 aPos;\n"
        "uniform mat4 u_MVP;\n"
        "void main()\n"
        "{\n"
        "   gl_Position = u_MVP * vec4(aPos, 1.0);\n"
        "}\n"};

    const char *fragSrc = {
        "#version 330 core\n"
        "out vec4 FragColor;\n"
        "void main()\n"
        "{\n"
        "    FragColor = vec4(1.0, 0.0, 0.0, 1.0);\n"
        "}"};

    const char *geometrySrc = {
        "#version 330 core\n"
        "layout (points) in;\n"
        "layout (points, max_vertices = 1) out;\n"
        "void main() {\n"
        "   gl_Position = gl_in[0].gl_Position;\n"
        "   gl_PointSize = 10.0;\n"
        "   EmitVertex();\n"
        "   EndPrimitive();\n"
        "}"};

    m_pointShader.create(vertSrc, fragSrc, geometrySrc);

    m_VBO.create(nullptr, 3 * 4 * sizeof(float), GL_DYNAMIC_DRAW);

    VertexBufferLayout layout;
    layout.push(GL_FLOAT, 3);

    m_VAO.create();
    m_VAO.bind();
    m_VAO.addBuffer(m_VBO, layout);
}

void TestCollisionDetection::onUpdate(float deltaTime)
{
    m_cm = rbd3d::collision(m_cube, m_cube2);

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
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    glViewport(0, 0, viewportWidth, viewportHeight);
    glClearColor(0.9f, 0.9f, 0.9f, 1.f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glm::vec3 color = m_cm.pointCount ? glm::vec3(1.0f, 0.6f, 0.2f) : glm::vec3(0.2f, 0.6f, 1.0f);
    m_cubeRenderer.render(viewportWidth, viewportHeight, m_camera, color);
    m_cubeRenderer2.render(viewportWidth, viewportHeight, m_camera, color);
    m_sphereRenderer.render(viewportWidth, viewportHeight, m_camera, color);

    if (m_cm.pointCount)
    {
        m_VBO.bind();
        std::vector<float> data(3 * m_cm.pointCount);
        for (int i = 0; i < m_cm.pointCount; ++i)
            for (int j = 0; j < 3; ++j)
                data[3 * i + j] = m_cm.contactPoints[i].position[j];
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(float) * m_cm.pointCount * 3, data.data());
        m_VAO.bind();
        m_pointShader.use();
        glm::mat4 view = m_camera.getViewMatrix();
        glm::mat4 proj = m_camera.getProjMatrix();
        glm::mat4 mvp = proj * view;
        m_pointShader.setUniformMatrix4fv("u_MVP", &mvp[0][0]);

        glEnable(GL_PROGRAM_POINT_SIZE);
        glDrawArrays(GL_POINTS, 0, m_cm.pointCount);
    }
}

void TestCollisionDetection::onImGuiRender()
{
    static glm::vec3 cubePos(0.f, 0.f, 0.f);
    static glm::vec3 cubeRot(0.f, 0.f, 0.f);
    ImGui::SliderFloat3("Cube Position", &cubePos.x, -2.f, 4.f);
    ImGui::SliderFloat3("Cube Rotation", &cubeRot.x, -180.f, 180.f);
    m_cube.setTranslation(cubePos);
    m_cube.setRotation(glm::quat(glm::radians(cubeRot)));

    if (m_cm.pointCount)
    {
        auto &n = m_cm.normal;

        ImGui::Text("Normal: (%.4f, %.4f, %.4f)", n.x, n.y, n.z);
        for (int i = 0; i < m_cm.pointCount; ++i)
        {
            auto &cp = m_cm.contactPoints[i];
            ImGui::Text("Pos: (%.4f, %.4f, %.4f), Depth: %.4f", cp.position.x, cp.position.y, cp.position.z, cp.depth);
        }
    }
}