#include <glad/glad.h>
#include <opengl_framework/Shader.h>
#include <stdexcept>

Shader::Shader() : m_id(0) {}

Shader::Shader(Shader &&other) : m_id(other.m_id) { other.m_id = 0; }

void Shader::create(const char *vertSrc, const char *fragSrc, const char *geometrySrc)
{
    destroy();

    unsigned int vert, frag, geometry;
    int success;
    char infoLog[512];

    vert = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vert, 1, &vertSrc, NULL);
    glCompileShader(vert);
    glGetShaderiv(vert, GL_COMPILE_STATUS, &success);
    if (!success)
    {
        glGetShaderInfoLog(vert, 512, NULL, infoLog);
        throw std::runtime_error(std::string("Vertex shader compile error: ") + infoLog);
    };

    frag = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(frag, 1, &fragSrc, NULL);
    glCompileShader(frag);
    glGetShaderiv(frag, GL_COMPILE_STATUS, &success);
    if (!success)
    {
        glGetShaderInfoLog(frag, 512, NULL, infoLog);
        throw std::runtime_error(std::string("Fragment shader compile error: ") + infoLog);
    };

    if (geometrySrc)
    {
        geometry = glCreateShader(GL_GEOMETRY_SHADER);
        glShaderSource(geometry, 1, &geometrySrc, NULL);
        glCompileShader(geometry);
        glGetShaderiv(geometry, GL_COMPILE_STATUS, &success);
        if (!success)
        {
            glGetShaderInfoLog(geometry, 512, NULL, infoLog);
            throw std::runtime_error(std::string("Geometry shader compile error: ") + infoLog);
        };
    }

    m_id = glCreateProgram();
    glAttachShader(m_id, vert);
    glAttachShader(m_id, frag);
    if (geometrySrc)
        glAttachShader(m_id, geometry);
    glLinkProgram(m_id);

    glGetProgramiv(m_id, GL_LINK_STATUS, &success);
    if (!success)
    {
        glGetProgramInfoLog(m_id, 512, NULL, infoLog);
        throw std::runtime_error(std::string("Shader link error: ") + infoLog);
    }

    glDeleteShader(vert);
    glDeleteShader(frag);
    if (geometrySrc)
        glDeleteShader(geometry);
}

void Shader::use() const { glUseProgram(m_id); }
unsigned int Shader::getId() const { return m_id; }
void Shader::destroy()
{
    if (m_id != 0)
    {
        glDeleteProgram(m_id);
        m_id = 0;
    }
}
Shader::~Shader() { destroy(); }

void Shader::setUniform1i(const char *name, int n) const
{
    glUniform1i(glGetUniformLocation(m_id, name), n);
}

void Shader::setUniform3fv(const char *name, const float *v) const
{
    glUniform3fv(glGetUniformLocation(m_id, name), 1, v);
}

void Shader::setUniform1f(const char *name, float f) const
{
    glUniform1f(glGetUniformLocation(m_id, name), f);
}

void Shader::setUniform4fv(const char *name, const float *v) const
{
    glUniform4fv(glGetUniformLocation(m_id, name), 1, v);
}

void Shader::setUniformMatrix4fv(const char *name, const float *ptr, bool transpose, unsigned int count) const
{
    glUniformMatrix4fv(glGetUniformLocation(m_id, name), count, transpose, ptr);
}