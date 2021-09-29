#include "Collider.h"

Collider::~Collider()
{

}


PlaneCollider::PlaneCollider(const glm::vec3& point1, const glm::vec3& point2, const glm::vec3& point3)
{

    this->point1 = point1;
    this->point2 = point2;
    this->point3 = point3;
    this->normal = glm::normalize(glm::cross(point2-point1, point3-point1));
    type = ColliderType::PLANE;
}

SphereCollider::SphereCollider(const float radius)
{
    this->radius = radius;
    type = ColliderType::SPHERE;
}

SphereCollider::SphereCollider(const float radius, UniformRigidBody* const rb)
{
    this->radius = radius;
    type = ColliderType::SPHERE;
    this->rb = rb;
}

CubeCollider::CubeCollider(const glm::vec3& sizes)
{
    xSize = sizes.x;
    ySize = sizes.y;
    zSize = sizes.z;
    scale = glm::vec3(2*xSize, 2*ySize, 2*zSize);
    type = ColliderType::CUBE;
}

CubeCollider::CubeCollider(const glm::vec3& sizes, UniformRigidBody* const rb)
{
    xSize = sizes.x;
    ySize = sizes.y;
    zSize = sizes.z;
    this->rb = rb;
    type = ColliderType::CUBE;
    scale = glm::vec3(2*xSize, 2*ySize, 2*zSize);
}

void CubeCollider::updateContactVerts()
{
    glm::vec3 px = xSize*rb->getLocalXAxis();
    glm::vec3 py = ySize*rb->getLocalYAxis();
    glm::vec3 pz = zSize*rb->getLocalZAxis();
    contactVertBuffer[0] = -px-py-pz;
    contactVertBuffer[1] = -px-py+pz;
    contactVertBuffer[2] = -px+py-pz;
    contactVertBuffer[3] = -px+py+pz;
    contactVertBuffer[4] = +px-py-pz;
    contactVertBuffer[5] = +px-py+pz;
    contactVertBuffer[6] = +px+py-pz;
    contactVertBuffer[7] = +px+py+pz;
}

void CubeCollider::updateContactEdges()
{
    glm::vec3 px = xSize*rb->getLocalXAxis();
    glm::vec3 py = ySize*rb->getLocalYAxis();
    glm::vec3 pz = zSize*rb->getLocalZAxis();

        contactEdgeBuffer[0] = +py-pz;
        contactEdgeBuffer[1] = +py+pz;
        contactEdgeBuffer[2] = -py-pz;
        contactEdgeBuffer[3] = -py+pz;
        contactEdgeBuffer[4] = -px+pz;
        contactEdgeBuffer[5] = -px-pz;
        contactEdgeBuffer[6] = +px-pz;
        contactEdgeBuffer[7] = +px+pz;
        contactEdgeBuffer[8] = +py-px;
        contactEdgeBuffer[9] = +py+px;
        contactEdgeBuffer[10] = -py-px;
        contactEdgeBuffer[11] = -py+px;
}

glm::vec3 CubeCollider::getContactDirNormalByIndex(int i)
{
    if(i<4)
        return rb->getLocalXAxis();
    else if(i<8)
        return rb->getLocalYAxis();
    else
        return rb->getLocalZAxis();
}

float CubeCollider::getContactSizeByIndex(int i)
{
    if(i<4)
        return xSize;
    else if(i<8)
        return ySize;
    else
        return zSize;
}

CubeCollider::ContactDir CubeCollider::flipDir(ContactDir dir, glm::vec3 relPos)
{
    switch(dir)
    {
    case ContactDir::RIGHT:
        if(glm::dot(relPos, rb->getLocalXAxis())<0)
            return ContactDir::LEFT;
        else
            return dir;
        break;
    case ContactDir::UP:
        if(glm::dot(relPos, rb->getLocalYAxis())<0)
            return ContactDir::DOWN;
        else
            return dir;
        break;
    case ContactDir::FORWARD:
        if(glm::dot(relPos, rb->getLocalZAxis())<0)
            return ContactDir::BACK;
        else
            return dir;
        break;
    }
}

glm::vec3 CubeCollider::getClosestEdge(const glm::vec3& dir, ContactDir normalTo)
{
    glm::vec3 px = xSize*rb->getLocalXAxis();
    glm::vec3 py = ySize*rb->getLocalYAxis();
    glm::vec3 pz = zSize*rb->getLocalZAxis();
    switch(normalTo)
    {
    case ContactDir::RIGHT:
    {
        float s1 = glm::sign(glm::dot(dir, py));
        float s2 = glm::sign(glm::dot(dir, pz));
        return s1*py+s2*pz;
    }
    case ContactDir::UP:
    {
        float s1 = glm::sign(glm::dot(dir, px));
        float s2 = glm::sign(glm::dot(dir, pz));
        return s1*px+s2*pz;
    }
    case ContactDir::FORWARD:
    {
        float s1 = glm::sign(glm::dot(dir, py));
        float s2 = glm::sign(glm::dot(dir, px));
        return s1*py+s2*px;
    }
    }
    return px;
}

glm::vec3 CubeCollider::getClosestVert(const glm::vec3& dir)
{
    float min = 100.0f;
    glm::vec3 minVert = contactVertBuffer[0];
    for(int i = 0; i<8;i++)
    {
        float test = glm::dot(dir, contactVertBuffer[i]);
        if(test<min)
        {
            min = test;
            minVert = contactVertBuffer[i];
        }
    }
    return minVert;
}

QuadCollider::QuadCollider(const float xSize, const float zSize)
{
    this->xSize = xSize;
    this->zSize = zSize;
    type = ColliderType::QUAD;
}
