#ifndef CAMERA_H_
#define CAMERA_H_

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

struct Camera
{
    float yfov;
    float near;
    float far;
    glm::vec3 pos;
    glm::vec3 dir;
    glm::vec3 up;
    glm::vec3 right;
    float yaw;
    float pitch;

    float aspect;

    Camera(float _yfov = 45.0f, float _near = 0.1f, float _far = 100.0f,
           glm::vec3 _pos = glm::vec3(0.0f, 3.0f, 5.0f),
           glm::vec3 _up = glm::vec3(0.0f, 1.0f, 0.0f),
           float _yaw = -90.f,  // degree
           float _pitch = -30.f // degree
    );

    void update();

    glm::mat4 getViewMatrix() const;
    glm::mat4 getProjMatrix() const;
    glm::mat4 getOrthoProjMatrix(float left, float right, float bottom, float top) const;

    void translate(const glm::vec3 &);
    void rotate(float yaw,  // mouse delta x, degree
                float pitch // mouse delta y, degree
    );

    void resize(int width, int height);
};

#endif
