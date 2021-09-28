#include <Common.h>

#include <QApplication>
#include <QSurfaceFormat>
#include <QWindow>
#include <QObject>
#include <QtOpenGL>
#include <QOpenGLContext>
#include <QOpenGLPaintDevice>
#include <QOpenGLFunctions>
#include <QOpenGLFunctions_4_5_Core>
#include <QOpenGLFunctions_3_3_Core>
#include <QPainter>
#include <QMouseEvent>
#include <QDebug>
#include <QCloseEvent>
#include <QElapsedTimer>
#include <QOpenGLShaderProgram>

#include <glm/glm.hpp>
#include <glm/common.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <MainWindow.h>
#include <Camera.h>
#include <Shader.h>
#include <Mesh.h>
#include <Entity.h>


float dt;
QOpenGLFunctions_4_5_Core* openglFunctions;
Shader* modelShader;
Shader* gridShader;
Mesh lineMesh;

enum ColliderType
{
    SPHERE = 0,
    CUBE = 1,
    PLANE = 2,
    QUAD = 3,
    NONE = 4
};

static void drawLine(glm::vec3 from, glm::vec3 to)
{
    glm::vec3 dir = to-from;
    float dist = glm::length(dir);
    dir = glm::normalize(dir);
    float theta = glm::acos(dir.x);
    theta = dir.z>0?-theta:theta;
    float psi = glm::asin(dir.y);
    glm::mat4 trans(1.0f);
    trans = glm::translate(trans, from);
    trans = glm::rotate(trans, theta, glm::vec3(0,1,0));
    trans = glm::rotate(trans, psi, glm::vec3(0,0,1));
    trans = glm::scale(trans, glm::vec3(dist, 1, 1));
    gridShader->setMat4("model", trans);
    lineMesh.draw(*gridShader);
}



struct RayCastData
{
    glm::vec3 point;
    glm::vec3 normal;
    float length;
};


struct UniformRigidBody
{

    //constants
    const float mass;
    const float inertia;
    float massInv;
    float inertiaInv;
    float elasticity = 0.25f;

    //state variables
    glm::vec3 position;
    glm::quat rotation;
    glm::mat3 rotationMatrix;
    glm::vec3 linearMomentum;
    glm::vec3 angularMomentum;

    //derived quantities
    glm::vec3 velocity;
    glm::vec3 angularVelocity;

    //known quantities
    glm::vec3 force;
    glm::vec3 torque;

    //applied force/torque will be applie for a single step
    std::vector<glm::vec3> appliedForces;
    std::vector<glm::vec3> appliedTorques;
    bool applyForce = false;
    bool applyTorque = false;
    bool dynamic = true;

    ColliderType type;

    UniformRigidBody(float _mass, float _inertia): mass(_mass), inertia(_inertia)
    {
        massInv = 1.0f/mass;
        inertiaInv = 1.0f/inertia;
        position = glm::vec3();
        rotation = glm::quat(glm::vec3(0,0,0));
        linearMomentum = glm::vec3();
        angularMomentum = glm::vec3();
        velocity = glm::vec3();
        angularVelocity = glm::vec3();
        force = glm::vec3();
        torque = glm::vec3();
        type = ColliderType::NONE;
    }

    virtual ~UniformRigidBody()
    {

    }

    void addForce(const glm::vec3& force)
    {
        appliedForces.push_back(force);
        applyForce = true;
    }

    void addTorque(const glm::vec3& torque)
    {
        appliedTorques.push_back(torque);
        applyTorque = true;
    }

    void setVelocity(const glm::vec3& velocity)
    {
        linearMomentum = velocity*mass;
    }

    void setAngularVelocity(const glm::vec3& angularVelocity)
    {
        angularMomentum = inertia*angularVelocity;
    }

    glm::vec3 getLocalXAxis()
    {
        return glm::vec3(rotationMatrix[0]);
    }

    glm::vec3 getLocalYAxis()
    {
        return glm::vec3(rotationMatrix[1]);
    }

    glm::vec3 getLocalZAxis()
    {
        return glm::vec3(rotationMatrix[2]);
    }

    glm::vec3 peekNextPosition(float dt)
    {
        glm::vec3 tempMomentum = linearMomentum+force*dt;
        return position + massInv*tempMomentum*dt;
    }
    void stepQuantities(float dt)
    {

        if(dynamic)
        {
        if(applyForce)
        {
            for(glm::vec3 force: appliedForces)
                linearMomentum+=dt*force;
            applyForce = false;
            appliedForces.clear();
        }
        if(applyTorque)
        {
            for(glm::vec3 torque: appliedTorques)
                angularMomentum+=dt*torque;
            applyTorque = false;
            appliedTorques.clear();
        }
        angularMomentum += torque*dt;
        linearMomentum += force*dt;
        angularVelocity = inertiaInv*angularMomentum;
        velocity = massInv*linearMomentum;
        rotation+= dt*0.5f*glm::quat(0, angularVelocity.x, angularVelocity.y, angularVelocity.z)*rotation;
        rotation = glm::normalize(rotation);
        rotationMatrix = glm::toMat3(rotation);
        position+=dt*velocity;
        }
    }
};

struct Collider
{
    ColliderType type;
    UniformRigidBody* rb = nullptr;
    bool collisionDetected = false;

    virtual ~Collider()
    {

    }
};

struct PlaneCollider: public Collider
{
    glm::vec3 normal;
    glm::vec3 point1, point2, point3;

    PlaneCollider(const glm::vec3& point1, const glm::vec3& point2, const glm::vec3& point3)
    {

        this->point1 = point1;
        this->point2 = point2;
        this->point3 = point3;
        this->normal = glm::normalize(glm::cross(point2-point1, point3-point1));
        type = ColliderType::PLANE;
    }
};


struct SphereCollider: public Collider
{
    float radius;

    SphereCollider(const float radius)
    {
        this->radius = radius;
        type = ColliderType::SPHERE;
    }

    SphereCollider(const float radius, UniformRigidBody* const rb)
    {
        this->radius = radius;
        type = ColliderType::SPHERE;
        this->rb = rb;
    }
};

struct CubeCollider: public Collider
{
    //The sizes from the origin of the cube (halfs)
    float xSize, ySize, zSize;
    glm::vec3 scale;
    glm::vec3 transformedVerts[8];
    glm::vec3 edges[6];

    CubeCollider(const glm::vec3& sizes)
    {
        xSize = sizes.x;
        ySize = sizes.y;
        zSize = sizes.z;
        transformedVerts[0] = glm::vec3(-xSize, -ySize, -zSize);
        transformedVerts[1] = glm::vec3(-xSize, -ySize, zSize);
        transformedVerts[2] = glm::vec3(xSize, -ySize, zSize);
        transformedVerts[3] = glm::vec3(xSize, -ySize, -zSize);
        transformedVerts[4] = glm::vec3(-xSize, ySize, -zSize);
        transformedVerts[5] = glm::vec3(-xSize, ySize, zSize);
        transformedVerts[6] = glm::vec3(xSize, ySize, zSize);
        transformedVerts[7] = glm::vec3(xSize, ySize, -zSize);
        edges[0] = glm::vec3(-xSize, 0, 0);
        edges[1] = glm::vec3(xSize, 0, 0);
        edges[2] = glm::vec3(0, -ySize, 0);
        edges[3] = glm::vec3(0, ySize, 0);
        edges[4] = glm::vec3(0, 0, -zSize);
        edges[5] = glm::vec3(0, 0, zSize);
        scale = glm::vec3(2*xSize, 2*ySize, 2*zSize);
        type = ColliderType::CUBE;
    }

    CubeCollider(const glm::vec3& sizes, UniformRigidBody* const rb)
    {
        xSize = sizes.x;
        ySize = sizes.y;
        zSize = sizes.z;
        this->rb = rb;   
        type = ColliderType::CUBE;
        updateTransformedVerts();
        updateEdges();
        scale = glm::vec3(2*xSize, 2*ySize, 2*zSize);
    }

    void updateTransformedVerts()
    {
        glm::vec3 aX = rb->getLocalXAxis();
        glm::vec3 aY = rb->getLocalYAxis();
        glm::vec3 aZ = rb->getLocalZAxis();
        transformedVerts[0] = rb->position-xSize*aX-ySize*aY-zSize*aZ;
        transformedVerts[1] = rb->position-xSize*aX-ySize*aY+zSize*aZ;
        transformedVerts[2] = rb->position+xSize*aX-ySize*aY+zSize*aZ;
        transformedVerts[3] = rb->position+xSize*aX-ySize*aY-zSize*aZ;
        transformedVerts[4] = rb->position-xSize*aX+ySize*aY-zSize*aZ;
        transformedVerts[5] = rb->position-xSize*aX+ySize*aY+zSize*aZ;
        transformedVerts[6] = rb->position+xSize*aX+ySize*aY+zSize*aZ;
        transformedVerts[7] = rb->position+xSize*aX+ySize*aY-zSize*aZ;
    }

    void updateEdges()
    {
        glm::vec3 aX = rb->getLocalXAxis();
        glm::vec3 aY = rb->getLocalYAxis();
        glm::vec3 aZ = rb->getLocalZAxis();
        edges[0] = rb->position-xSize*aX-zSize*aZ;
        edges[1] = rb->position-xSize*aX+zSize*aZ;
        edges[2] = rb->position-ySize*aY;
        edges[3] = rb->position+ySize*aY;
        edges[4] = rb->position-zSize*aZ;
        edges[5] = rb->position+zSize*aZ;
    }

};

struct QuadCollider: public Collider
{
    float xSize, zSize;

    QuadCollider(const float xSize, const float zSize)
    {
        this->xSize = xSize;
        this->zSize = zSize;
        type = ColliderType::QUAD;
    }
};

struct ContactInfo
{
    float penetrationDistance;
    glm::vec3 normal;
    std::vector<glm::vec3> points;
};

struct PhysicsWorld
{
private:
    //PlaneCollider standardPlane;
public:
    //std::vector<UniformRigidBody*> bodies;
    glm::vec3 gravity;
    std::vector<Collider*> colliders;
    RayCastData rcd;
    float friction = 0.4f;
    float restitutionSlope = 0.085f;
    float restitutionIntersect = 0.4f;
    ContactInfo info;

    bool Raycast(const glm::vec3& start, const glm::vec3& dir, RayCastData& data, Collider* collider)
    {
        switch(collider->type)
        {
        case ColliderType::PLANE:
            PlaneCollider* pc = dynamic_cast<PlaneCollider*>(collider);
            float d = glm::dot((pc->point1-start), pc->normal)/glm::dot(dir, pc->normal);
            glm::vec3 pos = start+dir*d;
            data.length = d;
            data.point = pos;
            data.normal = pc->normal;
            if(glm::dot(dir, pos-start)>0)
            {
                glm::vec3 dir1 = glm::normalize(glm::cross(pos-pc->point1, pc->point2-pc->point1));
                glm::vec3 dir2 = glm::normalize(glm::cross(pos-pc->point2, pc->point3-pc->point2));
                glm::vec3 dir3 = glm::normalize(glm::cross(pos-pc->point3, pc->point1-pc->point3));
                if(glm::all(glm::isnan(dir1))||glm::all(glm::isnan(dir2))||glm::all(glm::isnan(dir3)))
                    return true;
                float mag1 = glm::dot(dir1, dir2);
                float mag2 = glm::dot(dir1, dir3);
                float mag3 = glm::dot(dir2, dir3);
                if(glm::epsilonEqual(mag1, mag2, 0.1f) && glm::epsilonEqual(mag2, mag3, 0.1f))
                    return true;
            }
            break;
        }
        return false;
    }

    bool Raycast(const glm::vec3& start, const glm::vec3& dir, RayCastData& data)
    {
        for(Collider* collider: colliders)
        {
            switch(collider->type)
            {
            case ColliderType::PLANE:
                PlaneCollider* pc = dynamic_cast<PlaneCollider*>(collider);
                float d = glm::dot((pc->point1-start), pc->normal)/glm::dot(dir, pc->normal);
                glm::vec3 pos = start+dir*d;
                data.length = d;
                data.point = pos;
                data.normal = pc->normal;
                if(glm::dot(dir, pos-start)>0)
                {
                    glm::vec3 dir1 = glm::normalize(glm::cross(pos-pc->point1, pc->point2-pc->point1));
                    glm::vec3 dir2 = glm::normalize(glm::cross(pos-pc->point2, pc->point3-pc->point2));
                    glm::vec3 dir3 = glm::normalize(glm::cross(pos-pc->point3, pc->point1-pc->point3));
                    if(glm::all(glm::isnan(dir1))||glm::all(glm::isnan(dir2))||glm::all(glm::isnan(dir3)))
                        return true;
                    float mag1 = glm::dot(dir1, dir2);
                    float mag2 = glm::dot(dir1, dir3);
                    float mag3 = glm::dot(dir2, dir3);
                    if(glm::epsilonEqual(mag1, mag2, 0.1f) && glm::epsilonEqual(mag2, mag3, 0.1f))
                        return true;
                }
                break;
            }
        }
        return false;
    }

    PhysicsWorld(std::vector<Collider*>* colliders, glm::vec3 gravity)
    {
        this->gravity = gravity;
        this->colliders.reserve(colliders->size());
        for(auto& collider: *colliders)
        {
            this->colliders.push_back(collider);
        }
        for(auto& collider: *colliders)
        {
            if(collider->rb!=nullptr)
                collider->rb->force += collider->rb->mass*gravity;
        }

    }

    PhysicsWorld(std::vector<Collider*>* colliders)
    {
        gravity = glm::vec3(0, -9.81f, 0);
        this->colliders.reserve(colliders->size());
        for(auto& collider: *colliders)
        {
            this->colliders.push_back(collider);
        }
        for(auto& collider: *colliders)
        {
            if(collider->rb!=nullptr)
                collider->rb->force += collider->rb->mass*gravity;
        }

    }
    void checkForCollisions(float dt)
    {
        for(auto& collider: colliders)
        {
            switch(collider->type)
            {
            case ColliderType::SPHERE:
            {
                SphereCollider* sphere = dynamic_cast<SphereCollider*>(collider);
                spherePlaneCollision(dt, sphere);
                for(auto& other: colliders)
                {
                    if(other!=collider)
                    {
                        switch(other->type)
                        {
                        case ColliderType::SPHERE:
                            SphereCollider* otherSphere = dynamic_cast<SphereCollider*>(other);
                            if(detectSphereSphereCollision(sphere, otherSphere))
                                sphereSphereCollisionResponse(dt, sphere, otherSphere);
                            break;
                        }
                    }
                }
                break;
            }
            case ColliderType::CUBE:
            {
                CubeCollider* cube = dynamic_cast<CubeCollider*>(collider);
                for(auto& other: colliders)
                {
                    if(other!=collider)
                    {
                        switch(other->type)
                        {
                        case ColliderType::CUBE:
                            CubeCollider* otherCube = dynamic_cast<CubeCollider*>(other);
                            if(detectCubeCubeCollision(dt, cube, otherCube))
                            {
                                //respond
                            }
                            break;
                        }
                    }
                }
                break;
            }
            }
        }
    }



    bool detectCubeCubeCollision(float dt, CubeCollider* cubeA, CubeCollider* cubeB)
    {
        glm::vec3 aX = glm::normalize(cubeA->rb->getLocalXAxis());
        glm::vec3 aY = glm::normalize(cubeA->rb->getLocalYAxis());
        glm::vec3 aZ = glm::normalize(cubeA->rb->getLocalZAxis());

        glm::vec3 bX = glm::normalize(cubeB->rb->getLocalXAxis());
        glm::vec3 bY = glm::normalize(cubeB->rb->getLocalYAxis());
        glm::vec3 bZ = glm::normalize(cubeB->rb->getLocalZAxis());

        glm::vec3 T = cubeB->rb->position - cubeA->rb->position;

        float rxx = glm::dot(aX, bX);
        float rxy = glm::dot(aX, bY);
        float rxz = glm::dot(aX, bZ);

        float ryx = glm::dot(aY, bX);
        float ryy = glm::dot(aY, bY);
        float ryz = glm::dot(aY, bZ);

        float rzx = glm::dot(aZ, bX);
        float rzy = glm::dot(aZ, bY);
        float rzz = glm::dot(aZ, bZ);

        cubeA->collisionDetected = false;
        cubeB->collisionDetected = false;
        info.points.clear();

        std::string s;
        float penetration = glm::abs(glm::dot(T, aX)) - (cubeA->xSize + glm::abs(cubeB->xSize*rxx) + glm::abs(cubeB->ySize*rxy) + glm::abs(cubeB->zSize*rxz));
        //check for collisions parallel to AX
        if(penetration>0)
        {
            return false;
        }
        info.penetrationDistance = penetration;
        info.normal = aX;
        info.points.push_back(cubeA->rb->position+cubeA->xSize*aX);

        penetration = glm::abs(glm::dot(T, aY)) - (cubeA->ySize + glm::abs(cubeB->xSize*ryx) + glm::abs(cubeB->ySize*ryy) + glm::abs(cubeB->zSize*ryz));
        //check for collisions parallel to AY
        if(penetration>0)
        {
            return false;
        }
        if(penetration > info.penetrationDistance)
        {
            info.penetrationDistance = penetration;
            info.normal = aY;
        }

        penetration = glm::abs(glm::dot(T, aZ)) - (cubeA->zSize + glm::abs(cubeB->xSize*rzx) + glm::abs(cubeB->ySize*rzy) + glm::abs(cubeB->zSize*rzz));
        //check for collisions parallel to AZ
        if(penetration>0)
        {
            return false;
        }
        if(penetration > info.penetrationDistance)
        {
            info.penetrationDistance = penetration;
            info.normal = aZ;
        }

        penetration = glm::abs(glm::dot(T, bX)) - (glm::abs(cubeA->xSize*rxx) + glm::abs(cubeA->ySize*ryx) + glm::abs(cubeA->zSize*rzx) + cubeB->xSize);
        //check for collisions parallel to BX
        if(penetration>0)
        {
            return false;
        }
        if(penetration > info.penetrationDistance)
        {
            info.penetrationDistance = penetration;
            info.normal = bX;
        }

        penetration = glm::abs(glm::dot(T, bY)) - (glm::abs(cubeA->xSize*rxy) + glm::abs(cubeA->ySize*ryy) + glm::abs(cubeA->zSize*rzy) + cubeB->ySize);
        //check for collisions parallel to BY
        if(penetration>0)
        {
            return false;
        }
        if(penetration > info.penetrationDistance)
        {
            info.penetrationDistance = penetration;
            info.normal = bY;
        }

        penetration = glm::abs(glm::dot(T, bZ)) - (glm::abs(cubeA->xSize*rxz) + glm::abs(cubeA->ySize*ryz) + glm::abs(cubeA->zSize*rzz) + cubeB->zSize);
        //check for collisions parallel to BZ
        if(penetration>0)
        {
            return false;
        }
        if(penetration > info.penetrationDistance)
        {
            info.penetrationDistance = penetration;
            info.normal = bZ;
        }

        penetration = glm::abs(glm::dot(T, aZ)*ryx - glm::dot(T, aY)*rzx) - (glm::abs(cubeA->ySize*rzx)+glm::abs(cubeA->zSize*ryx)+glm::abs(cubeB->ySize*rxz)+glm::abs(cubeB->zSize*rxy));
        if(penetration>0)
        {
            return false;
        }
        if(penetration > info.penetrationDistance)
        {
            info.penetrationDistance = penetration;
            info.normal = glm::cross(aX, bX);

        }

        penetration = glm::abs(glm::dot(T, aZ)*ryy - glm::dot(T, aY)*rzy) - (glm::abs(cubeA->ySize*rzy)+glm::abs(cubeA->zSize*ryy)+glm::abs(cubeB->xSize*rxz)+glm::abs(cubeB->zSize*rxx));
        if(penetration>0)
        {
            return false;
        }
        if(penetration > info.penetrationDistance)
        {
            info.penetrationDistance = penetration;
            info.normal = glm::cross(aX, bY);
        }

        penetration = glm::abs(glm::dot(T, aZ)*ryz - glm::dot(T, aY)*rzz) - (glm::abs(cubeA->ySize*rzz)+glm::abs(cubeA->zSize*ryz)+glm::abs(cubeB->xSize*rxy)+glm::abs(cubeB->ySize*rxx));
        if(penetration>0)
        {
            return false;
        }
        if(penetration > info.penetrationDistance)
        {
            info.penetrationDistance = penetration;
            info.normal = glm::cross(aX, bZ);
        }

        penetration = glm::abs(glm::dot(T, aX)*rzx - glm::dot(T, aZ)*rxx) - (glm::abs(cubeA->xSize*rzx)+glm::abs(cubeA->zSize*rxx)+glm::abs(cubeB->ySize*ryz)+glm::abs(cubeB->zSize*ryy));
        if(penetration>0)
        {
            return false;
        }
        if(penetration > info.penetrationDistance)
        {
            info.penetrationDistance = penetration;
            info.normal = glm::cross(aY, bX);
        }

        penetration = glm::abs(glm::dot(T, aX)*rzy - glm::dot(T, aZ)*rxy) - (glm::abs(cubeA->xSize*rzy)+glm::abs(cubeA->zSize*rxy)+glm::abs(cubeB->xSize*ryz)+glm::abs(cubeB->zSize*ryx));
        if(penetration>0)
        {
            return false;
        }
        if(penetration > info.penetrationDistance)
        {
            info.penetrationDistance = penetration;
            info.normal = glm::cross(aY, bY);
        }

        penetration = glm::abs(glm::dot(T, aX)*rzz - glm::dot(T, aZ)*rxz) - (glm::abs(cubeA->xSize*rzz)+glm::abs(cubeA->zSize*rxz)+glm::abs(cubeB->xSize*ryy)+glm::abs(cubeB->ySize*ryx));
        if(penetration>0)
        {
            return false;
        }
        if(penetration > info.penetrationDistance)
        {
            info.penetrationDistance = penetration;
            info.normal = glm::cross(aY, bZ);
        }

        penetration = glm::abs(glm::dot(T, aY)*rxx - glm::dot(T, aX)*ryx) - (glm::abs(cubeA->xSize*ryx)+glm::abs(cubeA->ySize*rxx)+glm::abs(cubeB->ySize*rzz)+glm::abs(cubeB->zSize*rzy));
        if(penetration>0)
        {
            return false;
        }
        if(penetration > info.penetrationDistance)
        {
            info.penetrationDistance = penetration;
            info.normal = glm::cross(aZ, bX);
        }

        penetration = glm::abs(glm::dot(T, aY)*rxy - glm::dot(T, aX)*ryy) - (glm::abs(cubeA->xSize*ryy)+glm::abs(cubeA->ySize*rxy)+glm::abs(cubeB->xSize*rzz)+glm::abs(cubeB->zSize*rzx));
        if(penetration>0)
        {
            return false;
        }
        if(penetration > info.penetrationDistance)
        {
            info.penetrationDistance = penetration;
            info.normal = glm::cross(aZ, bY);
        }

        penetration = glm::abs(glm::dot(T, aY)*rxz - glm::dot(T, aX)*ryz) - (glm::abs(cubeA->xSize*ryz)+glm::abs(cubeA->ySize*rxz)+glm::abs(cubeB->xSize*rzy)+glm::abs(cubeB->ySize*rzx));
        if(penetration>0)
        {
            return false;
        }
        if(penetration > info.penetrationDistance)
        {
            info.penetrationDistance = penetration;
            info.normal = glm::cross(aZ, bZ);
        }
        qDebug() << "distance: " << info.penetrationDistance;
        cubeA->collisionDetected = true;
        cubeB->collisionDetected = true;
        return true;


    }

    bool detectSphereSphereCollision(SphereCollider* sphere, SphereCollider* other)
    {
        glm::vec3 dp = other->rb->position-sphere->rb->position;
        float lSquared = glm::length2(dp);
        float minDist = other->radius+sphere->radius;
        bool result = lSquared<=minDist*minDist;
        sphere->collisionDetected = result;
        other->collisionDetected = result;
        return result;
    }

    void sphereSphereCollisionResponse(float dt, SphereCollider* sphere, SphereCollider* other)
    {

        glm::vec3 dp = other->rb->position-sphere->rb->position;
        glm::vec3 relativeMomentum = sphere->rb->linearMomentum-other->rb->linearMomentum;
        dp = glm::normalize(dp);
        float mag = glm::dot(dp, relativeMomentum);
        if(mag>0)
        {
            glm::vec3 relativeMomentumNorm = mag*dp;
            other->rb->addForce((1.0f/dt)*relativeMomentumNorm);
        }
    }
    void spherePlaneCollision(float dt, SphereCollider* sphere)
    {

        if(Raycast(sphere->rb->position, glm::vec3(0,-1,0), rcd))
        {
            if(rcd.length<=sphere->radius)
            {
                float velNorm = glm::dot(glm::vec3(0,-1,0), sphere->rb->velocity);
                float sMax = restitutionSlope*-gravity.y+restitutionIntersect;
                if(velNorm<sMax)
                {
                    sphere->rb->position = glm::vec3(0,1,0)*sphere->radius+rcd.point;
                    sphere->rb->linearMomentum = glm::cross(glm::cross(glm::vec3(0,1,0), sphere->rb->linearMomentum), glm::vec3(0,1,0));
                    sphere->rb->setAngularVelocity(glm::cross(glm::vec3(0,1,0),sphere->rb->velocity/sphere->radius));
                    sphere->rb->addForce(-friction*sphere->rb->velocity);
                }
                else
                {
                    sphere->rb->position = glm::vec3(sphere->rb->position.x, 0.05f+sphere->radius, sphere->rb->position.z);
                    //sphere->addForce(glm::vec3(0, -2.0f*(sphere->velocity.y)/dt, 0));
                    glm::vec3 pNorm = glm::dot(glm::vec3(0,1,0), sphere->rb->linearMomentum)*sphere->rb->elasticity*glm::vec3(0,1,0);
                    glm::vec3 pPerp = glm::cross(glm::cross(glm::vec3(0,1,0), sphere->rb->linearMomentum), glm::vec3(0,1,0));
                    sphere->rb->linearMomentum = pPerp-pNorm;
                    sphere->rb->addTorque(glm::cross(glm::vec3(0,1,0),friction/dt*pPerp));

                }
            }
        }
    }
    void updateQuantities(float dt)
    {
        for(const auto& collider: colliders)
        {
            if(collider->rb!=nullptr)
                collider->rb->stepQuantities(dt);
        }
    }
    void stepWorld(float dt)
    {
        updateQuantities(dt);
        checkForCollisions(dt);
    }
};



int main(int argc, char *argv[])
{
    QGuiApplication app(argc, argv);

    QSurfaceFormat format;
    format.setSamples(4);
    format.setDepthBufferSize(24);
    format.setMajorVersion(4);
    format.setMinorVersion(5);
    format.setSwapInterval(0);
    format.setSwapBehavior(QSurfaceFormat::SwapBehavior::DefaultSwapBehavior);
    format.setProfile(QSurfaceFormat::CoreProfile);

    MainWindow window;
    window.setTitle("Animation Project");
    window.setFormat(format);
    window.setSurfaceType(QWindow::OpenGLSurface);
    window.resize(1280, 720);
    window.setKeyboardGrabEnabled(true);
    window.show();
    CloseEventFilter closeFilter(&window);
    window.installEventFilter(&closeFilter);


    QOpenGLContext* context = new QOpenGLContext(&window);
    context->setFormat(window.requestedFormat());
    context->create();
    context->makeCurrent(&window);

    //app.processEvents();
    QOpenGLPaintDevice* paintDevice = new QOpenGLPaintDevice;
    paintDevice->setSize(window.size() * window.devicePixelRatio());
    paintDevice->setDevicePixelRatio(window.devicePixelRatio());

    //painter->setWorldMatrixEnabled(false);

    openglFunctions = context->versionFunctions<QOpenGLFunctions_4_5_Core>();
    if(!openglFunctions)
    {
        qDebug("Could not obtain required version of opengl");
        app.exit();
    }
    openglFunctions->initializeOpenGLFunctions();



    window.openglInitialized = true;
    openglFunctions->glViewport(0, 0, window.width() * window.devicePixelRatio(), window.height() * window.devicePixelRatio());
    openglFunctions->glEnable(GL_DEPTH_TEST);
    openglFunctions->glEnable(GL_CULL_FACE);
    openglFunctions->glEnable(GL_LINE_SMOOTH);
    openglFunctions->glEnable(GL_LINE_WIDTH);
    openglFunctions->glLineWidth(2.5f);
    openglFunctions->glDisable(GL_LIGHTING);


    Shader modelShaderObj("model.vert", "model.frag");
    Shader gridShaderObj("grid.vert", "grid.frag");
    modelShader = &modelShaderObj;
    gridShader = &gridShaderObj;

    glm::vec3 euler(0,0,0);
    glm::mat4 trans(1.0f);
    modelShader->insertUniform("model");
    modelShader->insertUniform("view");
    modelShader->insertUniform("projection");
    modelShader->insertUniform("color");
    modelShader->insertUniform("lightDir");
    gridShader->insertUniform("model");
    gridShader->insertUniform("view");
    gridShader->insertUniform("projection");
    gridShader->insertUniform("color");
    //modelShader.setVec3("color", glm::vec3(1,1,1));
    glm::mat4 projection = glm::perspective((float)PI*0.33f, (float)window.width()/window.height(), 0.1f, 100.0f);

    Camera cam(glm::vec3(0,2,5));
    cam.updateView();

    modelShader->setVec3("lightDir", glm::vec3(-0.5f, -1.0f, -0.75));
    modelShader->setMat4("model", trans);
    modelShader->setMat4("view", cam.view);
    modelShader->setMat4("projection", projection);
    gridShader->setMat4("model", trans);
    gridShader->setMat4("view", cam.view);
    gridShader->setMat4("projection", projection);

    Mesh::initializeStaticArrays();
    lineMesh = Mesh::createLine();
    lineMesh.setColor(glm::vec3(0,0,1));
    Mesh pointMesh = Mesh::createSphere();
    pointMesh.setColor(glm::vec3(0.3,0.45,0.3));
    Entity point(pointMesh);
    point.setScale(glm::vec3(0.2,0.2,0.2));

    Entity plane = createGridedPlaneEntity(10);

    Entity unitDirs = createUnitDirs();
    Entity cube = createBoundedCubeEntity();
    Entity cone = createBoundedConeEntity();
    Entity sphere = createBoundedSphereEntity();
    Entity cylinder = createBoundedCylinderEntity();
    Entity capsule = createBoundedCapsuleEntity();

    //Entity sphere = createBoundedSphereEntity();
    float mass = 1.0f;
    float radius = 1.0f;
    float inertia = (2.0f/5.0f)*mass*radius*radius;
    UniformRigidBody rb(mass, inertia);
    UniformRigidBody otherRb(mass, inertia);
    // SphereBody otherRb(mass, 0.5f);
    CubeCollider collider(glm::vec3(3.0f,0.5f,1.5f));
    CubeCollider otherCollider(glm::vec3(0.5f,0.5f,0.5f));
    collider.rb = &rb;
    otherCollider.rb = &otherRb;
    std::vector<Collider*> colliders = {&collider, &otherCollider};
    PhysicsWorld world(&colliders, glm::vec3(0, 0.0f, 0));

    PlaneCollider p1(glm::vec3(-10, 0, -10), glm::vec3(-10, 0, 10), glm::vec3(10, 0, 10));
    PlaneCollider p2(glm::vec3(-10, 0, -10), glm::vec3(10, 0, 10), glm::vec3(10, 0, -10));
    world.colliders.push_back(&p1);
    world.colliders.push_back(&p2);


    rb.position = glm::vec3(0, 2, 0);
    otherRb.position = glm::vec3(0,2,-5);

    QElapsedTimer elapsedTimer;
    elapsedTimer.start();
    long time = elapsedTimer.nsecsElapsed();
    while(window.shouldRun())
    {

        long timeNow = elapsedTimer.nsecsElapsed();
        dt = (timeNow-time)/1000000000.0f;
        time = timeNow;

        openglFunctions->glEnable(GL_DEPTH_TEST);
        openglFunctions->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        if(window.windowResized())
        {
            paintDevice->setSize(window.size() * window.devicePixelRatio());
            paintDevice->setDevicePixelRatio(window.devicePixelRatio());
            projection = glm::perspective((float)PI*0.33f, (float)window.width()/window.height(), 0.1f, 100.0f);
            modelShader->setMat4("projection", projection);
            gridShader->setMat4("projection", projection);
        }

        if(window.getKey(Qt::Key_A))
        {
            cam.translateRight(-dt);
        }
        if(window.getKey(Qt::Key_D))
        {
            cam.translateRight(dt);
        }
        if(window.getKey(Qt::Key_W))
        {
            cam.translateFwd(-dt);
        }
        if(window.getKey(Qt::Key_S))
        {
            cam.translateFwd(dt);
        }
        if(window.getMouse(Qt::MouseButton::LeftButton))
        {
            QPointF deltaPos = QCursor::pos()-window.mousePos;
            window.mousePos = QCursor::pos();
            cam.smoothRotateYaw(-dt*deltaPos.x());
            cam.smoothRotatePitch(-dt*deltaPos.y());
            //            cam.rotateYaw(-(float)dt*deltaPos.x());
            //            cam.rotatePitch(-(float)dt*deltaPos.y());
        }
        rb.setVelocity(glm::vec3(0,0,0));
        if(window.getKey(Qt::Key_Right))
        {
            rb.setVelocity(1.0f*cam.getRight());
        }
        if(window.getKey(Qt::Key_Left))
        {
            rb.setVelocity(-1.0f*cam.getRight());
        }
        if(window.getKey(Qt::Key_Up))
        {
            rb.setVelocity(glm::cross(glm::vec3(0,1,0), cam.getRight()));
        }
        if(window.getKey(Qt::Key_Down))
        {
            rb.setVelocity(glm::cross(glm::vec3(0,-1,0), cam.getRight()));
        }
        if(window.getGetDown(Qt::Key_Space))
        {
            rb.addForce(glm::vec3(0,800,0));
        }
        if(window.getGetDown(Qt::Key_R))
        {
            rb.setVelocity(glm::vec3(0,0,0));
            rb.position = glm::vec3(0,5,0);
        }
        if(window.getGetDown(Qt::Key_1))
        {
            rb.setVelocity(glm::vec3(0,0,0));
        }
        cam.smoothUpdateView();
        //cam.updateView();
        modelShader->setMat4("view", cam.view);
        gridShader->setMat4("view", cam.view);

        //unitDirs.rotate(glm::quat(glm::vec3(0,dt,0)));
        //unitDirs.draw();

        world.stepWorld(dt);

        // cube.meshes[1].setColor(glm::vec3(1,0,0));
        rb.setAngularVelocity(glm::vec3(0,0,1));
        otherRb.setAngularVelocity(glm::vec3(0,1,0));
        if(collider.collisionDetected)
        {
            cube.meshes[1].setColor(glm::vec3(1,0,0));
        }
        else
        {
            cube.meshes[1].setColor(glm::vec3(0,1,0));
        }
        cube.setPosition(rb.position);
        cube.setRotation(rb.rotation);
        cube.setScale(collider.scale);
        cube.draw();

        collider.updateTransformedVerts();
        for(int i =0; i<8; i++)
        {

            point.setPosition(collider.transformedVerts[i]);
            point.draw();
        }
        collider.updateEdges();
        for(int i =0;i<6;i++)
        {
            point.setPosition(collider.edges[i]);
            point.draw();
        }
        cube.setPosition(otherRb.position);
        cube.setRotation(otherRb.rotation);
        cube.setScale(otherCollider.scale);
        cube.draw();


        plane.draw();
        RayCastData rcd;
        if(world.Raycast(rb.position, glm::vec3(0,-1,0), rcd))
            drawLine(rb.position, rcd.point);
        //drawLine(glm::vec3(0, 5.0, 0), glm::vec3(5, -5.0, -5));
        //glm::mat4 ok = glm::translate(trans, glm::vec3(0, 2, 0));



        std::string vector;
        //vector = std::to_string(rb.getLocalYAxis().x) + ", " + std::to_string(rb.getLocalYAxis().y) + ", "+ std::to_string(rb.getLocalYAxis().z);
        openglFunctions->glDisable(GL_DEPTH_TEST);
        QPainter painter(paintDevice);
        painter.setWorldMatrixEnabled(false);
        painter.setPen(Qt::white);
        painter.setFont(QFont("Arial", 12));
        QRectF rect(0.0f,0.0f,paintDevice->size().width(), paintDevice->size().height());
        painter.beginNativePainting();
        //painter.drawText(rect, std::to_string(1.0/dt).c_str());
        //painter.drawText(rect, vector.c_str());
        painter.endNativePainting();

        window.resetInputs();
        app.processEvents();
        openglFunctions->glFinish();
        context->makeCurrent(&window);
        context->swapBuffers(&window);
    }

    app.quit();
    return 0;
}
