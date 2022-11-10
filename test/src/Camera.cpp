#include "Camera.h"
#include <glm/gtc/quaternion.hpp>

Camera::Camera(float _yfov, float _near, float _far,
               glm::vec3 _pos, glm::vec3 _up,
               float _yaw, float _pitch)
    : yfov(_yfov), near(_near), far(_far), pos(_pos), yaw(_yaw), pitch(_pitch)
{
    update();
}

void Camera::update()
{
    float cp = glm::cos(glm::radians(pitch));
    dir = glm::normalize(glm::vec3(
        glm::cos(glm::radians(yaw)) * cp,
        glm::sin(glm::radians(pitch)),
        glm::sin(glm::radians(yaw)) * cp));
    right = glm::normalize(glm::cross(dir, glm::vec3(0.f, 1.f, 0.f)));
    up = glm::normalize(glm::cross(right, dir));
}

glm::mat4 Camera::getViewMatrix() const
{
    return glm::lookAt(pos, pos + dir, up);
}
glm::mat4 Camera::getProjMatrix() const
{
    return glm::perspective(glm::radians(yfov), aspect, near, far);
}

void Camera::translate(const glm::vec3 &displacement)
{
    pos += displacement;
}

void Camera::rotate(float y, float p)
{
    yaw += y;
    pitch += p;
    if (pitch > 89.0f)
        pitch = 89.0f;
    if (pitch < -89.0f)
        pitch = -89.0f;
    update();
}

void Camera::resize(int w, int h)
{
    aspect = (float)w / (float)h;
}