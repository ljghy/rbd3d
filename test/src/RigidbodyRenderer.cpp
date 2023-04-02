#include "RigidbodyRenderer.h"

RigidbodyRenderer::RigidbodyRenderer()
{
}

void RigidbodyRenderer::render(Shader &shader,
                               int viewportWidth,
                               int viewportHeight,
                               Camera &camera,
                               const glm::vec3 &color)
{
    if (viewportWidth * viewportHeight == 0)
        return;
    shader.use();
    camera.resize(viewportWidth, viewportHeight);

    glm::mat4 model = glm::translate(glm::mat4(1.f), m_collider->translation()) * glm::mat4_cast(m_collider->rotation());
    glm::mat4 view = camera.getViewMatrix();
    glm::mat4 proj = camera.getProjMatrix();
    glm::mat4 mvp = proj * view * model;

    shader.setUniformMatrix4fv("u_model", &model[0][0]);
    shader.setUniformMatrix4fv("u_MVP", &mvp[0][0]);
    shader.setUniform3fv("u_diffuse", &color.x);

    m_VAO.bind();
    m_IBO.bind();
    glDrawElements(GL_TRIANGLES, m_IBO.getCount(), GL_UNSIGNED_INT, 0);
}
