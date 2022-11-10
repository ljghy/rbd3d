#ifndef TEST_VIEWER_RIGIDBODY_RENDERER_H_
#define TEST_VIEWER_RIGIDBODY_RENDERER_H_

#include <opengl_framework/VertexBufferObject.h>
#include <opengl_framework/VertexArrayObject.h>
#include <opengl_framework/IndexBufferObject.h>
#include <opengl_framework/Shader.h>

#include <rbd3d/collision.h>

#include "Camera.h"

class RigidbodyRenderer
{
public:
    RigidbodyRenderer();

    template <typename ColliderType>
    void create(ColliderType &);

    void render(int viewportWidth, int viewportHeight, Camera &camera, const glm::vec3 &color);

private:
    void createShader();

private:
    rbd3d::RigidbodyBase *m_collider;

    Shader m_shader;
    VertexBufferObject m_VBO;
    VertexArrayObject m_VAO;
    IndexBufferObject m_IBO;
};

template <>
inline void RigidbodyRenderer::create(rbd3d::Plane &plane)
{
    m_collider = &plane;

    const glm::vec3 &n = plane.normal();
    const glm::vec3 &o = plane.origin();
    glm::vec3 absn = glm::abs(n), t;
    if (absn.x >= absn.y && absn.x >= absn.z)
    {
        t = glm::normalize(glm::vec3(-n.y / n.x, 1.f, 0.f));
    }
    else if (absn.y > absn.x && absn.y >= absn.z)
    {
        t = glm::normalize(glm::vec3(0.f, -n.z / n.y, 1.f));
    }
    else
    {
        t = glm::normalize(glm::vec3(1.f, 0.f, -n.x / n.z));
    }
    glm::vec3 bt = glm::cross(n, t);

    constexpr float l = 50.f;

    glm::vec3 v[]{
        o + t * l + bt * l,
        o - t * l + bt * l,
        o - t * l - bt * l,
        o + t * l - bt * l};
    float vertices[24]{
        v[0].x, v[0].y, v[0].z, n.x, n.y, n.z,
        v[1].x, v[1].y, v[1].z, n.x, n.y, n.z,
        v[2].x, v[2].y, v[2].z, n.x, n.y, n.z,
        v[3].x, v[3].y, v[3].z, n.x, n.y, n.z};

    unsigned int indices[]{0, 1, 2,
                           0, 2, 3};

    m_VBO.create(vertices, sizeof(vertices));
    m_IBO.create(indices, sizeof(indices));

    VertexBufferLayout layout;
    layout.push(GL_FLOAT, 3);
    layout.push(GL_FLOAT, 3);

    m_VAO.create();
    m_VAO.bind();
    m_VAO.addBuffer(m_VBO, layout);
}

template <>
inline void RigidbodyRenderer::create(rbd3d::Cuboid &cube)
{
    m_collider = &cube;
    glm::vec3 v = 0.5f * cube.size();

    float vertices[]{
        v.x, v.y, v.z, 1, 0, 0,
        v.x, -v.y, v.z, 1, 0, 0,
        v.x, -v.y, -v.z, 1, 0, 0,
        v.x, v.y, -v.z, 1, 0, 0,

        v.x, v.y, v.z, 0, 1, 0,
        v.x, v.y, -v.z, 0, 1, 0,
        -v.x, v.y, -v.z, 0, 1, 0,
        -v.x, v.y, v.z, 0, 1, 0,

        v.x, v.y, v.z, 0, 0, 1,
        -v.x, v.y, v.z, 0, 0, 1,
        -v.x, -v.y, v.z, 0, 0, 1,
        v.x, -v.y, v.z, 0, 0, 1,

        -v.x, v.y, v.z, -1, 0, 0,
        -v.x, v.y, -v.z, -1, 0, 0,
        -v.x, -v.y, -v.z, -1, 0, 0,
        -v.x, -v.y, v.z, -1, 0, 0,

        v.x, -v.y, v.z, 0, -1, 0,
        -v.x, -v.y, v.z, 0, -1, 0,
        -v.x, -v.y, -v.z, 0, -1, 0,
        v.x, -v.y, -v.z, 0, -1, 0,

        v.x, v.y, -v.z, 0, 0, -1,
        v.x, -v.y, -v.z, 0, 0, -1,
        -v.x, -v.y, -v.z, 0, 0, -1,
        -v.x, v.y, -v.z, 0, 0, -1};

    unsigned int indices[]{
        0, 1, 2, 0, 2, 3,
        4, 5, 6, 4, 6, 7,
        8, 9, 10, 8, 10, 11,
        12, 13, 14, 12, 14, 15,
        16, 17, 18, 16, 18, 19,
        20, 21, 22, 20, 22, 23};

    m_VBO.create(vertices, sizeof(vertices));
    m_IBO.create(indices, sizeof(indices));

    VertexBufferLayout layout;
    layout.push(GL_FLOAT, 3);
    layout.push(GL_FLOAT, 3);

    m_VAO.create();
    m_VAO.bind();
    m_VAO.addBuffer(m_VBO, layout);
}

template <>
inline void RigidbodyRenderer::create(rbd3d::Sphere &sphere)
{
    m_collider = &sphere;
    float r = sphere.radius();

    constexpr int segments = 8, vertCount = (segments - 1) * segments * 2 + 2;

    float vertices[vertCount * 6];

    unsigned int offset = 0;
    for (int i = 0; i <= segments; ++i)
    {
        for (int j = 0; j < ((i == 0 || i == segments) ? 1 : segments * 2); ++j)
        {
            float theta = glm::radians(180.f / segments * i),
                  phi = glm::radians(180.f / segments * j);
            vertices[offset + 3] = glm::sin(theta) * glm::sin(phi);
            vertices[offset + 4] = glm::cos(theta);
            vertices[offset + 5] = glm::sin(theta) * glm::cos(phi);
            vertices[offset + 0] = vertices[offset + 3] * r;
            vertices[offset + 1] = vertices[offset + 4] * r;
            vertices[offset + 2] = vertices[offset + 5] * r;
            offset += 6;
        }
    }

    unsigned int indices[(segments - 1) * segments * 2 * 2 * 3];

    offset = 0;

    for (int j = 0; j < 2 * segments; ++j)
    {
        indices[offset] = 0;
        indices[offset + 1] = j + 1;
        indices[offset + 2] = (j + 1) % (segments * 2) + 1;
        offset += 3;
    }

    for (int i = 1; i < segments - 1; ++i)
    {
        unsigned int idx = 1 + 2 * segments * (i - 1);
        for (int j = 0; j < 2 * segments; ++j)
        {
            indices[offset] = indices[offset + 3] = idx + j;
            indices[offset + 1] = idx + j + 2 * segments;
            indices[offset + 5] = idx + (j + 1) % (2 * segments);
            indices[offset + 2] = indices[offset + 4] = indices[offset + 5] + 2 * segments;
            offset += 6;
        }
    }

    unsigned int idx = vertCount - 2 * segments - 1;
    for (int j = 0; j < 2 * segments; ++j)
    {
        indices[offset] = idx + j;
        indices[offset + 1] = vertCount - 1;
        indices[offset + 2] = idx + (j + 1) % (2 * segments);
        offset += 3;
    }

    m_VBO.create(vertices, sizeof(vertices));
    m_IBO.create(indices, sizeof(indices));

    VertexBufferLayout layout;
    layout.push(GL_FLOAT, 3);
    layout.push(GL_FLOAT, 3);

    m_VAO.create();
    m_VAO.bind();
    m_VAO.addBuffer(m_VBO, layout);
}

#endif