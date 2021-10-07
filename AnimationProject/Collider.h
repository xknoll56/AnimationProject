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
    unsigned int id;
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
    enum ContactDir
    {
        RIGHT = 0,
        UP = 1,
        FORWARD = 2,
        NONE = 3
    };
    struct EdgeIndices
    {
        int i0;
        int i1;
        float length;
        //the midpoints will be updated every request for edges
        glm::vec3 midPoint;
        glm::vec3 dir;
    };

    EdgeIndices edges[12];
    std::vector<int> indices;



    CubeCollider();
    CubeCollider(const CubeCollider& other);
    CubeCollider(const glm::vec3& sizes);
    CubeCollider(const glm::vec3& sizes, UniformRigidBody* const rb);
    CubeCollider& operator= (const CubeCollider& other);
    void initEdges();
    void updateContactVerts();
    void updateContactEdges();
    std::vector<EdgeIndices> getEdgesFromVertexIndices();
    std::vector<glm::vec3> getClosestVerts(const glm::vec3& dir);

};

struct QuadCollider: public Collider
{
    float xSize, zSize;

    QuadCollider(const float xSize, const float zSize);
};
#endif // COLLIDER_H
