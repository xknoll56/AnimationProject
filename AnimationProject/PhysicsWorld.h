#ifndef PHYSICSWORLD_H
#define PHYSICSWORLD_H

#include <glm/glm.hpp>
#include <vector>
#include <Collider.h>
struct RayCastData
{
    glm::vec3 point;
    glm::vec3 normal;
    float length;
};

struct ContactInfo
{
    float penetrationDistance;
    glm::vec3 normal;
    std::vector<glm::vec3> points;
    CubeCollider::ContactDir aDir;
    CubeCollider::ContactDir bDir;
    bool faceCollision;
};

struct PhysicsWorld
{
    glm::vec3 gravity;
    std::vector<Collider*> colliders;
    RayCastData rcd;
    float friction = 0.4f;
    float restitutionSlope = 0.085f;
    float restitutionIntersect = 0.4f;
    ContactInfo edgeInfo;
    ContactInfo faceInfo;

    PhysicsWorld(std::vector<Collider*>* colliders, glm::vec3 gravity);
    PhysicsWorld(std::vector<Collider*>* colliders);
    bool Raycast(const glm::vec3& start, const glm::vec3& dir, RayCastData& data, Collider* collider);
    bool Raycast(const glm::vec3& start, const glm::vec3& dir, RayCastData& data);
    void checkForCollisions(float dt);
    glm::vec3 closestPointBetweenLines(glm::vec3& p0,  glm::vec3& p1, const glm::vec3& u, const glm::vec3& v);
    float closestDistanceBetweenLines(glm::vec3& p0,  glm::vec3& p1, const glm::vec3& u, const glm::vec3& v);
    bool detectCubeCubeCollision(float dt, CubeCollider* cubeA, CubeCollider* cubeB);
    bool detectSphereSphereCollision(SphereCollider* sphere, SphereCollider* other);
    void sphereSphereCollisionResponse(float dt, SphereCollider* sphere, SphereCollider* other);
    void spherePlaneCollision(float dt, SphereCollider* sphere);
    void updateQuantities(float dt);
    void stepWorld(float dt);
};


#endif // PHYSICSWORLD_H