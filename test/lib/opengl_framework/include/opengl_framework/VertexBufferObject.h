#ifndef OPENGL_VERTEX_BUFFER_OBJECT_H_
#define OPENGL_VERTEX_BUFFER_OBJECT_H_

#include <glad/glad.h>

class VertexBufferObject
{
private:
    unsigned int m_id;

public:
    VertexBufferObject();
    ~VertexBufferObject();

    VertexBufferObject(const VertexBufferObject &) = delete;
    VertexBufferObject &operator=(const VertexBufferObject &) = delete;
    VertexBufferObject(VertexBufferObject &&);

    void create(const void *data, unsigned int size, GLenum drawType = GL_STATIC_DRAW);
    void destroy();

    void bind() const;
    void unbind() const;
};

#endif
