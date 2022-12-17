#ifndef OPENGL_SHADER_H_
#define OPENGL_SHADER_H_

#include <string>

class Shader
{
private:
    unsigned int m_id;

public:
    Shader();
    ~Shader();

    void create(const char *vertSrc, const char *fragSrc, const char *geometrySrc = nullptr);

    Shader(const Shader &) = delete;
    Shader &operator=(const Shader &) = delete;
    Shader(Shader &&other);

    unsigned int getId() const;

    void use() const;
    void destroy();

    void setUniform1i(const char *name, int n) const;
    void setUniform3fv(const char *name, const float *v) const;
    void setUniform1f(const char *name, float f) const;
    void setUniform4fv(const char *name, const float *v) const;
    void setUniformMatrix4fv(const char *name, const float *ptr, bool transpose = false, unsigned int count = 1u) const;
};

#endif