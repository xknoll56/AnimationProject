#define PI 3.14159265359f
#include <Camera.h>

Camera::Camera()
{
    pos = glm::vec3(0,0,0);
    pitch = yaw = 0;
    targPitch = targYaw = 0;
}

Camera::Camera(glm::vec3 pos)
{
    this->pos = pos;
    pitch = yaw = 0;
    targPitch = targYaw = 0;
}


void Camera::updateView()
{

    float cosPitch = glm::cos(pitch);
    float sinPitch = glm::sin(pitch);
    float cosYaw = glm::cos(yaw);
    float sinYaw = glm::sin(yaw);

    right = { cosYaw, 0, -sinYaw };
    up = { sinYaw * sinPitch, cosPitch, cosYaw * sinPitch };
    fwd = { sinYaw * cosPitch, -sinPitch, cosPitch * cosYaw };

    view =
    {

        glm::vec4(       right.x,            up.x,            fwd.x,      0 ),
        glm::vec4(       right.y,            up.y,            fwd.y,      0 ),
        glm::vec4(       right.z,            up.z,            fwd.z,      0 ),
        glm::vec4( -glm::dot( right, pos ), -glm::dot( up, pos ), -glm::dot( fwd, pos ), 1 )
    };
}

void Camera::smoothUpdateView()
{

    pitch = glm::mix(pitch, targPitch, smooth);
    yaw = glm::mix(yaw, targYaw, smooth);

    float cosPitch = glm::cos(pitch);
    float sinPitch = glm::sin(pitch);
    float cosYaw = glm::cos(yaw);
    float sinYaw = glm::sin(yaw);

    right = { cosYaw, 0, -sinYaw };
    up = { sinYaw * sinPitch, cosPitch, cosYaw * sinPitch };
    fwd = { sinYaw * cosPitch, -sinPitch, cosPitch * cosYaw };

    view =
    {

        glm::vec4(       right.x,            up.x,            fwd.x,      0 ),
        glm::vec4(       right.y,            up.y,            fwd.y,      0 ),
        glm::vec4(       right.z,            up.z,            fwd.z,      0 ),
        glm::vec4( -glm::dot( right, pos ), -glm::dot( up, pos ), -glm::dot( fwd, pos ), 1 )
    };
}

void Camera::translateFwd(float inc)
{
    pos += inc*speed*fwd;
}

void Camera::translateRight(float inc)
{
    pos += inc*speed*right;
}

void Camera::rotateYaw(float inc)
{
    yaw+=rotationSpeed*inc;
}

void Camera::rotatePitch(float inc)
{
    pitch += rotationSpeed*inc;
    pitch = glm::clamp(pitch, -PI*0.495f, PI*0.495f);
}

void Camera::smoothRotateYaw(float inc)
{
    targYaw+=inc;
}
void Camera::smoothRotatePitch(float inc)
{
    targPitch += inc;
    targPitch = glm::clamp(targPitch, -PI*0.495f, PI*0.495f);
}

const glm::mat4& Camera::getView()
{
    return view;
}

const glm::vec3& Camera::getFwd()
{
    return fwd;
}

const glm::vec3& Camera::getRight()
{
    return right;
}

const glm::vec3& Camera::getUp()
{
    return up;
}
