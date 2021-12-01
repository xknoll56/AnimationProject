#ifndef PHYSICSWORLD_H
#define PHYSICSWORLD_H

#include <glm/glm.hpp>
#include <vector>
#include <Collider.h>
#include <QDebug>
struct RayCastData
{
    glm::vec3 point;
    glm::vec3 normal;
    float length;
    Collider* collider;
};

struct ContactInfo
{
    float penetrationDistance;
    glm::vec3 normal;
    std::vector<glm::vec3> points;
    BoxCollider::ContactDir aDir;
    BoxCollider::ContactDir bDir;
    int vertexPoints;
    int edgePoints;
    Collider* a;
    Collider* b;
    bool faceCollision;
    bool faceToFaceCollision;

    ContactInfo()
    {
        penetrationDistance = -std::numeric_limits<float>().max();
        normal = glm::vec3();
        aDir = BoxCollider::ContactDir::NONE;
        bDir = BoxCollider::ContactDir::NONE;
        faceCollision = true;
        faceToFaceCollision = false;
        vertexPoints = 0;
        edgePoints = 0;
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
    void checkForCollisions(float dt);
    void CollisionResponse(float dt);
    glm::vec3 closestPointBetweenLines(glm::vec3& p0,  glm::vec3& p1, const glm::vec3& u, const glm::vec3& v);
    float closestDistanceBetweenLines(glm::vec3& p0,  glm::vec3& p1, const glm::vec3& u, const glm::vec3& v, float s0, float s1);
    bool closestPointsDoIntersect(glm::vec3& p0,  glm::vec3& p1,  const glm::vec3& u, const glm::vec3& v, float s0, float s1, float max1, float max2);
    bool detectCubeCubeCollision(float dt, BoxCollider* cubeA, BoxCollider* cubeB, ContactInfo& contactInfo);
    void determineCubeCubeContactPoints(ContactInfo& info, BoxCollider* cubeA, BoxCollider* cubeB);
    bool detectCubeSphereCollision(float dt, BoxCollider* cube, SphereCollider* sphere, ContactInfo& contactInfo);
    void determineCubeCubePetrusionVerts(ContactInfo& info,const glm::vec3& normal, const std::vector<glm::vec3>& points, BoxCollider* toCube, BoxCollider::ContactDir dir, bool adjustPenetration);
    bool isCubeCubePetrusion(const glm::vec3& normal, const std::vector<glm::vec3>& points, BoxCollider* toCube, BoxCollider::ContactDir dir);
    void cubeCubeCollisionResponse(ContactInfo& info, float dt, BoxCollider* cubeA, BoxCollider* cubeB);
    void cubeCubeCollisionResponseDynamicVsStatic(ContactInfo& info, const glm::vec3& norm, float dt, BoxCollider* dynamicCube, BoxCollider* staticCube);
    void cubeSphereCollisionResponseStaticVsDynamic(ContactInfo& info, float dt, BoxCollider* cube, SphereCollider* sphere);
    void cubeSphereCollisionResponseDynamicVsDynamic(ContactInfo& info, float dt, BoxCollider* cube, SphereCollider* sphere);
    bool detectSphereSphereCollision(SphereCollider* sphere, SphereCollider* other);
    void sphereSphereCollisionResponse(float dt, SphereCollider* sphere, SphereCollider* other);
    void updateQuantities(float dt);
    void stepWorld(float dt);
    void stepWorld(float dt, int inc);
    bool cubeFlatOnSurface(BoxCollider* cube, glm::vec3& normal, float tolerance);
    bool cubeRaycast(const glm::vec3& start, const glm::vec3& dir, RayCastData& dat, BoxCollider* cube);
    bool sphereRaycast(const glm::vec3& start, const glm::vec3& dir, RayCastData& dat, SphereCollider* sphere);
    bool raycastAll(const glm::vec3& start, const glm::vec3& dir, RayCastData& dat);
    bool raycastAll(const glm::vec3& start, const glm::vec3& dir, RayCastData& dat, int mask);
    bool raycastAll(const glm::vec3& start, const glm::vec3& dir, RayCastData& dat, ColliderType type);
    bool narrowPhaseCollisionDetection(Collider& colA, Collider& colB);
};

class Utilities
{
public:
    static void PrintVec3(const glm::vec3& vec)
    {
        qDebug() << "x: "<<vec.x <<" y: "<<vec.y << " z: "<<vec.z;
    }
};


#endif // PHYSICSWORLD_H
