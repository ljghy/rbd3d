#include "RigidbodyRenderer.h"

RigidbodyRenderer::RigidbodyRenderer()
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
        "    vec3 lightDir = normalize(vec3(1.0, 1.0, 1.0));\n"
        "    vec3 lightColor= vec3(0.8, 0.8, 0.8);\n"
        "    vec3 ambient = vec3(0.5, 0.5, 0.5);\n"
        "    color = vec4(u_diffuse * (max(dot(norm, lightDir), 0.0) * lightColor + ambient), 1.0);\n"
        "}";
    m_shader.create(vertShaderSrc, fragShaderSrc);
}

void RigidbodyRenderer::render(int viewportWidth,
                               int viewportHeight,
                               Camera &camera,
                               const glm::vec3 &color)
{
    if (viewportWidth * viewportHeight == 0)
        return;
    m_shader.use();
    camera.resize(viewportWidth, viewportHeight);

    glm::mat4 model = glm::translate(glm::mat4(1.f), m_collider->translation()) * glm::mat4_cast(m_collider->rotation());
    glm::mat4 view = camera.getViewMatrix();
    glm::mat4 proj = camera.getProjMatrix();
    glm::mat4 mvp = proj * view * model;

    m_shader.setUniformMatrix4fv("u_model", &model[0][0]);
    m_shader.setUniformMatrix4fv("u_MVP", &mvp[0][0]);
    m_shader.setUniform3fv("u_diffuse", &color.x);

    m_VAO.bind();
    m_IBO.bind();
    glDrawElements(GL_TRIANGLES, m_IBO.getCount(), GL_UNSIGNED_INT, 0);
}
