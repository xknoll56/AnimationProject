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
    Collider* a;
    Collider* b;
    bool faceCollision;

    ContactInfo()
    {
        penetrationDistance = -std::numeric_limits<float>().max();
        normal = glm::vec3();
        aDir = CubeCollider::ContactDir::NONE;
        bDir = CubeCollider::ContactDir::NONE;
        faceCollision = true;
    }
};

struct PhysicsWorld
{
    glm::vec3 gravity;
    std::vector<Collider*> colliders;
    RayCastData rcd;
    float friction = 0.4f;
    float restitutionSlope = 0.085f;
    float restitutionIntersect = 0.4f;
    std::vector<ContactInfo> contacts;
    bool enableResponse = true;

    PhysicsWorld(std::vector<Collider*>* colliders, glm::vec3 gravity);
    PhysicsWorld(std::vector<Collider*>* colliders);
    PhysicsWorld();
    void setColliders(std::vector<Collider*>* colliders);
    bool contactHandled(Collider* a, Collider* b);
    bool Raycast(const glm::vec3& start, const glm::vec3& dir, RayCastData& data, Collider* collider);
    bool Raycast(const glm::vec3& start, const glm::vec3& dir, RayCastData& data);
    bool Raycast(const glm::vec3& start, const glm::vec3& dir, RayCastData& data, CubeCollider* cubeCollider);
    void checkForCollisions(float dt);
    glm::vec3 closestPointBetweenLines(glm::vec3& p0,  glm::vec3& p1, const glm::vec3& u, const glm::vec3& v);
    float closestDistanceBetweenLines(glm::vec3& p0,  glm::vec3& p1, const glm::vec3& u, const glm::vec3& v, float s0, float s1);
    bool closestPointsDoIntersect(glm::vec3& p0,  glm::vec3& p1, const glm::vec3& u, const glm::vec3& v, float s0, float s1);
    bool detectCubeCubeCollision(float dt, CubeCollider* cubeA, CubeCollider* cubeB, ContactInfo& contactInfo);
    void determineCubeCubeContactPoints(ContactInfo& info, CubeCollider* cubeA, CubeCollider* cubeB);
    void determineCubeCubePetrusionVerts(ContactInfo& info,const glm::vec3& normal, const std::vector<glm::vec3>& points, CubeCollider* toCube, CubeCollider::ContactDir dir, bool adjustPenetration);
    void cubeCubeCollisionResponse(ContactInfo& info, float dt, CubeCollider* cubeA, CubeCollider* cubeB);
    void cubeCubeCollisionResponseAlt(float dt, CubeCollider* cubeA, CubeCollider* cubeB);
    bool detectSphereSphereCollision(SphereCollider* sphere, SphereCollider* other);
    void sphereSphereCollisionResponse(float dt, SphereCollider* sphere, SphereCollider* other);
    void spherePlaneCollision(float dt, SphereCollider* sphere);
    void updateQuantities(float dt);
    void stepWorld(float dt);
};


#endif // PHYSICSWORLD_H
