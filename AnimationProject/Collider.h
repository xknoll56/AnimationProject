#ifndef COLLIDER_H
#define COLLIDER_H

#include <UniformRigidBody.h>
#include <glm/glm.hpp>

enum ColliderType
{
    SPHERE = 0,
    CUBE = 1,
    PLANE = 2,
    QUAD = 3,
    NONE = 4
};


struct Collider
{
    ColliderType type;
    UniformRigidBody* rb = nullptr;
    bool collisionDetected = false;
    Collider()
    {
        type = ColliderType::NONE;
    }
    Collider(const Collider& other)
    {
        type = other.type;
        rb = other.rb;
        collisionDetected = other.collisionDetected;
    }
    Collider& operator= (const Collider& other)
    {
        type = other.type;
        rb = other.rb;
        collisionDetected = other.collisionDetected;
        return *this;
    }
    virtual ~Collider();
};

struct PlaneCollider: public Collider
{
    glm::vec3 normal;
    glm::vec3 point1, point2, point3;
    PlaneCollider(const glm::vec3& point1, const glm::vec3& point2, const glm::vec3& point3);
};


struct SphereCollider: public Collider
{
    float radius;

    SphereCollider(const float radius);
    SphereCollider(const float radius, UniformRigidBody* const rb);
};

struct CubeCollider: public Collider
{
    //The sizes from the origin of the cube (halfs)
    float xSize, ySize, zSize;
    glm::vec3 scale;
    glm::vec3 contactVertBuffer[8];
    glm::vec3 contactEdgeBuffer[12];

    enum ContactDir
    {
        LEFT = 0,
        RIGHT = 1,
        DOWN = 2,
        UP = 3,
        BACK = 4,
        FORWARD = 5,
        NONE = 6
    };

    CubeCollider();
    CubeCollider(const CubeCollider& other);
    CubeCollider(const glm::vec3& sizes);
    CubeCollider(const glm::vec3& sizes, UniformRigidBody* const rb);
    CubeCollider& operator= (const CubeCollider& other);
    void updateContactVerts();
    void updateContactEdges();
    glm::vec3 getContactDirNormalByIndex(int i);
    float getContactSizeByIndex(int i);
    ContactDir flipDir(ContactDir dir, glm::vec3 relPos);
    glm::vec3 getClosestEdge(const glm::vec3& dir, ContactDir normalTo);
    glm::vec3 getClosestVert(const glm::vec3& dir);

};

struct QuadCollider: public Collider
{
    float xSize, zSize;

    QuadCollider(const float xSize, const float zSize);
};
#endif // COLLIDER_H
