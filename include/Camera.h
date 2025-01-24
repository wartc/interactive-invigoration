#ifndef __CAMERA_H__
#define __CAMERA_H__

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

constexpr float SENSITIVITY = 0.02f;
constexpr float ZOOM_SENSITIVITY = 1.0f;
constexpr float MOVE_SPEED = 2.0f;
constexpr float PAN_SPEED = 0.1f;
const glm::vec3 WORLD_UP{0.0f, 1.0f, 0.0f};

class Camera {
 public:
  enum class Direction { FORWARD, BACKWARD, LEFT, RIGHT };

 private:
  // viewing properties
  float fov{45.0f};
  float aspectRatio{16.0f / 9.0f};
  float near{0.1f}, far{15.0f};

  // camera properties
  glm::vec3 pos{};
  glm::vec3 front{};
  glm::vec3 up{};
  glm::vec3 right{};

  // rotation
  float yaw{-90.0f};  // horizontal rotation (default: looking towards -Z)
  float pitch{0.0f};  // vertical rotation

 public:
  Camera(
      const glm::vec3& position = glm::vec3{0.0f, 0.0f, 0.0f},
      const glm::vec3& target = glm::vec3{0.0f, 0.0f, 0.0f}
  )
      : pos{position} {
    front = glm::normalize(target - position);
    updateVectors();
  }

  glm::mat4 getViewMatrix() { return glm::lookAt(pos, pos + front, up); }

  glm::mat4 getProjectionMatrix() {
    return glm::perspective(glm::radians(fov), aspectRatio, near, far);
  }

  void move(Direction direction, float deltaTime) {
    float velocity = MOVE_SPEED * deltaTime;
    switch (direction) {
      case Direction::FORWARD: pos += front * velocity; break;
      case Direction::BACKWARD: pos -= front * velocity; break;
      case Direction::LEFT: pos -= right * velocity; break;
      case Direction::RIGHT: pos += right * velocity; break;
    }
  }

  void rotate(float deltaYaw, float deltaPitch) {
    yaw += deltaYaw * SENSITIVITY * 1.5f;
    pitch += deltaPitch * SENSITIVITY * 1.5f;

    // avoid gimbal lock
    if (pitch > 89.0f) pitch = 89.0f;
    if (pitch < -89.0f) pitch = -89.0f;

    updateVectors();
  }

  void pan(float deltaX, float deltaY) {
    pos += -deltaX * SENSITIVITY * PAN_SPEED * right;
    pos += deltaY * SENSITIVITY * PAN_SPEED * up;
  }

  void zoom(float yoffset) {
    fov -= yoffset * ZOOM_SENSITIVITY;
    fov = glm::clamp(fov, 10.0f, 45.0f);
  }

 private:
  void updateVectors() {
    glm::vec3 newFront;
    newFront.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
    newFront.y = sin(glm::radians(pitch));
    newFront.z = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
    front = glm::normalize(newFront);

    right = glm::normalize(glm::cross(front, WORLD_UP));
    up = glm::normalize(glm::cross(right, front));
  }
};

#endif
