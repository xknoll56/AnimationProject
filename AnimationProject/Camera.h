#ifndef CAMERA_H
#define CAMERA_H

#include <glm/glm.hpp>

class Camera
{
private:

    float pitch, yaw;
    glm::vec3 fwd;
    glm::vec3 up;
    glm::vec3 right;
    glm::vec3 pos;
    float speed = 5.0f;

public:
    Camera();
    Camera(glm::vec3 pos);
    void updateView();
    void translateFwd(float inc);
    void translateRight(float inc);
    void rotateYaw(float inc);
    void rotatePitch(float inc);
    const glm::mat4& getView();
    const glm::vec3& getFwd();
    const glm::vec3& getRight();
    const glm::vec3& getUp();
    glm::mat4 view;
};

#endif // CAMERA_H
