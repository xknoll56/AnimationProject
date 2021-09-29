#include "PhysicsWorld.h"
#include <QDebug>

PhysicsWorld::PhysicsWorld(std::vector<Collider*>* colliders, glm::vec3 gravity)
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

PhysicsWorld::PhysicsWorld(std::vector<Collider*>* colliders)
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

bool PhysicsWorld::Raycast(const glm::vec3& start, const glm::vec3& dir, RayCastData& data, Collider* collider)
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

bool PhysicsWorld::Raycast(const glm::vec3& start, const glm::vec3& dir, RayCastData& data)
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


void PhysicsWorld::checkForCollisions(float dt)
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

glm::vec3 PhysicsWorld::closestPointBetweenLines(glm::vec3& p0,  glm::vec3& p1, const glm::vec3& u, const glm::vec3& v)
{
    glm::vec3 uv = glm::cross(v,u);
    float uvSquared = glm::length2(uv);
    float t = -glm::dot(glm::cross(p1-p0, u), uv)/uvSquared;
    float s = -glm::dot(glm::cross(p1-p0, v), uv)/uvSquared;
    p0 += s*u;
    p1 += t*v;
    p0 += p1;
    p0 *= 0.5f;
    return p0;
}

float PhysicsWorld::closestDistanceBetweenLines(glm::vec3& p0,  glm::vec3& p1, const glm::vec3& u, const glm::vec3& v)
{
    glm::vec3 uv = glm::cross(v,u);
    float uvSquared = glm::length2(uv);
    float t = -glm::dot(glm::cross(p1-p0, u), uv)/uvSquared;
    float s = -glm::dot(glm::cross(p1-p0, v), uv)/uvSquared;
    if(glm::abs(t)>0.5f || glm::abs(s)>0.5f)
        return 100.0f;

    return glm::abs(glm::dot(uv, p0-p1)/glm::length(uv));
}



bool PhysicsWorld::detectCubeCubeCollision(float dt, CubeCollider* cubeA, CubeCollider* cubeB)
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
    edgeInfo.points.clear();
    faceInfo.points.clear();
    float penetration = glm::abs(glm::dot(T, aX)) - (cubeA->xSize + glm::abs(cubeB->xSize*rxx) + glm::abs(cubeB->ySize*rxy) + glm::abs(cubeB->zSize*rxz));
    //check for collisions parallel to AX
    if(penetration>0)
    {
        return false;
    }
    faceInfo.penetrationDistance = penetration;
    faceInfo.normal = aX;
    faceInfo.faceCollision = true;
    faceInfo.aDir = CubeCollider::ContactDir::RIGHT;
    faceInfo.bDir = CubeCollider::ContactDir::NONE;

    penetration = glm::abs(glm::dot(T, aY)) - (cubeA->ySize + glm::abs(cubeB->xSize*ryx) + glm::abs(cubeB->ySize*ryy) + glm::abs(cubeB->zSize*ryz));
    //check for collisions parallel to AY
    if(penetration>0)
    {
        return false;
    }
    if(penetration > faceInfo.penetrationDistance && penetration < 0.0)
    {
        faceInfo.penetrationDistance = penetration;
        faceInfo.normal = aY;
        faceInfo.faceCollision = true;
        faceInfo.aDir = CubeCollider::ContactDir::UP;
    }

    penetration = glm::abs(glm::dot(T, aZ)) - (cubeA->zSize + glm::abs(cubeB->xSize*rzx) + glm::abs(cubeB->ySize*rzy) + glm::abs(cubeB->zSize*rzz));
    //check for collisions parallel to AZ
    if(penetration>0)
    {
        return false;
    }
    if(penetration > faceInfo.penetrationDistance && penetration < 0.0)
    {
        faceInfo.penetrationDistance = penetration;
        faceInfo.normal = aZ;
        faceInfo.faceCollision = true;
        faceInfo.aDir = CubeCollider::ContactDir::FORWARD;
    }

    penetration = glm::abs(glm::dot(T, bX)) - (glm::abs(cubeA->xSize*rxx) + glm::abs(cubeA->ySize*ryx) + glm::abs(cubeA->zSize*rzx) + cubeB->xSize);
    //check for collisions parallel to BX
    if(penetration>0)
    {
        return false;
    }
    if(penetration > faceInfo.penetrationDistance && penetration < 0.0)
    {
        faceInfo.penetrationDistance = penetration;
        faceInfo.normal = bX;
        faceInfo.faceCollision = true;
        faceInfo.aDir = CubeCollider::ContactDir::NONE;
        faceInfo.bDir = CubeCollider::ContactDir::RIGHT;
    }

    penetration = glm::abs(glm::dot(T, bY)) - (glm::abs(cubeA->xSize*rxy) + glm::abs(cubeA->ySize*ryy) + glm::abs(cubeA->zSize*rzy) + cubeB->ySize);
    //check for collisions parallel to BY
    if(penetration>0)
    {
        return false;
    }
    if(penetration > faceInfo.penetrationDistance && penetration < 0.0)
    {
        faceInfo.penetrationDistance = penetration;
        faceInfo.normal = bY;
        faceInfo.faceCollision = true;
        faceInfo.aDir = CubeCollider::ContactDir::NONE;
        faceInfo.bDir = CubeCollider::ContactDir::UP;
    }

    penetration = glm::abs(glm::dot(T, bZ)) - (glm::abs(cubeA->xSize*rxz) + glm::abs(cubeA->ySize*ryz) + glm::abs(cubeA->zSize*rzz) + cubeB->zSize);
    //check for collisions parallel to BZ
    if(penetration>0)
    {
        return false;
    }
    if(penetration > faceInfo.penetrationDistance && penetration < 0.0)
    {
        faceInfo.penetrationDistance = penetration;
        faceInfo.normal = bZ;
        faceInfo.faceCollision = true;
        faceInfo.aDir = CubeCollider::ContactDir::NONE;
        faceInfo.bDir = CubeCollider::ContactDir::FORWARD;
    }

    penetration = glm::abs(glm::dot(T, aZ)*ryx - glm::dot(T, aY)*rzx) - (glm::abs(cubeA->ySize*rzx)+glm::abs(cubeA->zSize*ryx)+glm::abs(cubeB->ySize*rxz)+glm::abs(cubeB->zSize*rxy));
    if(penetration>0)
    {
        return false;
    }
    if(penetration < 0.0)
    {
        edgeInfo.penetrationDistance = penetration;
        edgeInfo.normal = glm::cross(aX, bX);
        edgeInfo.faceCollision = false;
        edgeInfo.aDir = CubeCollider::ContactDir::RIGHT;
        edgeInfo.bDir = CubeCollider::ContactDir::RIGHT;

    }

    penetration = glm::abs(glm::dot(T, aZ)*ryy - glm::dot(T, aY)*rzy) - (glm::abs(cubeA->ySize*rzy)+glm::abs(cubeA->zSize*ryy)+glm::abs(cubeB->xSize*rxz)+glm::abs(cubeB->zSize*rxx));
    if(penetration>0)
    {
        return false;
    }
    if(penetration > edgeInfo.penetrationDistance && penetration < 0.0)
    {
        edgeInfo.penetrationDistance = penetration;
        edgeInfo.normal = glm::cross(aX, bY);
        edgeInfo.faceCollision = false;
        edgeInfo.aDir = CubeCollider::ContactDir::RIGHT;
        edgeInfo.bDir = CubeCollider::ContactDir::UP;
    }

    penetration = glm::abs(glm::dot(T, aZ)*ryz - glm::dot(T, aY)*rzz) - (glm::abs(cubeA->ySize*rzz)+glm::abs(cubeA->zSize*ryz)+glm::abs(cubeB->xSize*rxy)+glm::abs(cubeB->ySize*rxx));
    if(penetration>0)
    {
        return false;
    }
    if(penetration > edgeInfo.penetrationDistance && penetration < 0.0)
    {
        edgeInfo.penetrationDistance = penetration;
        edgeInfo.normal = glm::cross(aX, bZ);
        edgeInfo.faceCollision = false;
        edgeInfo.aDir = CubeCollider::ContactDir::RIGHT;
        edgeInfo.bDir = CubeCollider::ContactDir::FORWARD;
    }

    penetration = glm::abs(glm::dot(T, aX)*rzx - glm::dot(T, aZ)*rxx) - (glm::abs(cubeA->xSize*rzx)+glm::abs(cubeA->zSize*rxx)+glm::abs(cubeB->ySize*ryz)+glm::abs(cubeB->zSize*ryy));
    if(penetration>0)
    {
        return false;
    }
    if(penetration > edgeInfo.penetrationDistance && penetration < 0.0)
    {
        edgeInfo.penetrationDistance = penetration;
        edgeInfo.normal = glm::cross(aY, bX);
        edgeInfo.faceCollision = false;
        edgeInfo.aDir = CubeCollider::ContactDir::UP;
        edgeInfo.bDir = CubeCollider::ContactDir::RIGHT;
    }

    penetration = glm::abs(glm::dot(T, aX)*rzy - glm::dot(T, aZ)*rxy) - (glm::abs(cubeA->xSize*rzy)+glm::abs(cubeA->zSize*rxy)+glm::abs(cubeB->xSize*ryz)+glm::abs(cubeB->zSize*ryx));
    if(penetration>0)
    {
        return false;
    }
    if(penetration > edgeInfo.penetrationDistance && penetration < 0.0)
    {
        edgeInfo.penetrationDistance = penetration;
        edgeInfo.normal = glm::cross(aY, bY);
        edgeInfo.faceCollision = false;
        edgeInfo.aDir = CubeCollider::ContactDir::UP;
        edgeInfo.bDir = CubeCollider::ContactDir::UP;
    }

    penetration = glm::abs(glm::dot(T, aX)*rzz - glm::dot(T, aZ)*rxz) - (glm::abs(cubeA->xSize*rzz)+glm::abs(cubeA->zSize*rxz)+glm::abs(cubeB->xSize*ryy)+glm::abs(cubeB->ySize*ryx));
    if(penetration>0)
    {
        return false;
    }
    if(penetration > edgeInfo.penetrationDistance && penetration < 0.0)
    {
        edgeInfo.penetrationDistance = penetration;
        edgeInfo.normal = glm::cross(aY, bZ);
        edgeInfo.faceCollision = false;
        edgeInfo.aDir = CubeCollider::ContactDir::UP;
        edgeInfo.bDir = CubeCollider::ContactDir::FORWARD;
    }

    penetration = glm::abs(glm::dot(T, aY)*rxx - glm::dot(T, aX)*ryx) - (glm::abs(cubeA->xSize*ryx)+glm::abs(cubeA->ySize*rxx)+glm::abs(cubeB->ySize*rzz)+glm::abs(cubeB->zSize*rzy));
    if(penetration>0)
    {
        return false;
    }
    if(penetration > edgeInfo.penetrationDistance && penetration < 0.0)
    {
        edgeInfo.penetrationDistance = penetration;
        edgeInfo.normal = glm::cross(aZ, bX);
        edgeInfo.faceCollision = false;
        edgeInfo.aDir = CubeCollider::ContactDir::FORWARD;
        edgeInfo.bDir = CubeCollider::ContactDir::RIGHT;
    }

    penetration = glm::abs(glm::dot(T, aY)*rxy - glm::dot(T, aX)*ryy) - (glm::abs(cubeA->xSize*ryy)+glm::abs(cubeA->ySize*rxy)+glm::abs(cubeB->xSize*rzz)+glm::abs(cubeB->zSize*rzx));
    if(penetration>0)
    {
        return false;
    }
    if(penetration > edgeInfo.penetrationDistance && penetration < 0.0)
    {
        edgeInfo.penetrationDistance = penetration;
        edgeInfo.normal = glm::cross(aZ, bY);
        edgeInfo.faceCollision = false;
        edgeInfo.aDir = CubeCollider::ContactDir::FORWARD;
        edgeInfo.bDir = CubeCollider::ContactDir::UP;
    }

    penetration = glm::abs(glm::dot(T, aY)*rxz - glm::dot(T, aX)*ryz) - (glm::abs(cubeA->xSize*ryz)+glm::abs(cubeA->ySize*rxz)+glm::abs(cubeB->xSize*rzy)+glm::abs(cubeB->ySize*rzx));
    if(penetration>0)
    {
        return false;
    }
    if(penetration > edgeInfo.penetrationDistance && penetration < 0.0)
    {
        edgeInfo.penetrationDistance = penetration;
        edgeInfo.normal = glm::cross(aZ, bZ);
        edgeInfo.faceCollision = false;
        edgeInfo.aDir = CubeCollider::ContactDir::FORWARD;
        edgeInfo.bDir = CubeCollider::ContactDir::FORWARD;
    }

    if(faceInfo.penetrationDistance>edgeInfo.penetrationDistance || edgeInfo.penetrationDistance>-0.0001f)
    {

        if(faceInfo.aDir==CubeCollider::ContactDir::NONE)
        {
            //The plane of reflection is from B to A
            faceInfo.normal = glm::sign(glm::dot(faceInfo.normal,-T))*faceInfo.normal;
            cubeA->updateContactVerts();
            faceInfo.points.push_back(cubeA->rb->position+cubeA->getClosestVert(faceInfo.normal));
            faceInfo.normal = -faceInfo.normal;
        }
        else
        {
            //The plane of reflection is from A to B
            faceInfo.normal = glm::sign(glm::dot(faceInfo.normal,T))*faceInfo.normal;
            cubeB->updateContactVerts();
            //edge test
            float e1Mag = glm::dot(cubeB->rb->getLocalXAxis(), faceInfo.normal);
            float e2Mag = glm::dot(cubeB->rb->getLocalYAxis(), faceInfo.normal);
            float e3Mag = glm::dot(cubeB->rb->getLocalZAxis(), faceInfo.normal);
            if(!(glm::epsilonEqual(0.0f, e1Mag, 0.00001f) || glm::epsilonEqual(0.0f, e2Mag, 0.00001f) || glm::epsilonEqual(0.0f, e3Mag, 0.00001f)))
                faceInfo.points.push_back(cubeB->rb->position+cubeB->getClosestVert(faceInfo.normal));

        }

        glm::vec3 vRel = -cubeB->rb->velocity+cubeA->rb->velocity;
        float mag = abs(glm::dot(vRel, faceInfo.normal));

        glm::vec3 fParallel = mag*faceInfo.normal/dt;
        glm::vec3 fPerp = glm::cross(vRel, faceInfo.normal)/dt;
        cubeB->rb->addForce(fParallel);
        for(int i =0;i<faceInfo.points.size();i++)
            cubeB->rb->addTorque(glm::cross(faceInfo.points[i]-cubeB->rb->position, fPerp));
        // }
    }
    else
    {
        edgeInfo.normal = glm::sign(glm::dot(edgeInfo.normal,-T))*edgeInfo.normal;
        cubeA->updateContactEdges();
        glm::vec3 edgeADir = cubeA->rb->getLocalXAxis();
        if(edgeInfo.aDir == CubeCollider::ContactDir::UP)
            edgeADir = cubeA->rb->getLocalYAxis();
        else if(edgeInfo.aDir == CubeCollider::ContactDir::FORWARD)
            edgeADir = cubeA->rb->getLocalZAxis();
        glm::vec3 p0, pA, e0;
        pA = cubeA->rb->position + cubeA->contactEdgeBuffer[0];


        glm::vec3 edgeBDir = cubeB->rb->getLocalXAxis();
        if(edgeInfo.bDir == CubeCollider::ContactDir::UP)
            edgeBDir = cubeB->rb->getLocalYAxis();
        else if(edgeInfo.bDir == CubeCollider::ContactDir::FORWARD)
            edgeBDir = cubeB->rb->getLocalZAxis();
        cubeB->updateContactEdges();
        glm::vec3 p1, pB, e1;
        pB = cubeB->rb->position + cubeB->contactEdgeBuffer[0];

        glm::vec3 point = pA;
        float dist = 0.15f;
        for(int i = 0;i<12;i++)
        {
            for(int j =0;j<12;j++)
            {
                pA = cubeA->rb->position + cubeA->contactEdgeBuffer[i];
                pB = cubeB->rb->position + cubeB->contactEdgeBuffer[j];
                edgeADir = cubeA->getContactDirNormalByIndex(i);
                edgeBDir = cubeB->getContactDirNormalByIndex(j);
                float check = closestDistanceBetweenLines(pA, pB, edgeADir, edgeBDir);
                if(check < dist)
                {
                    //dist = check;
                    //                    p0 = pA;
                    //                    p1 = pB;
                    //                    e0 = edgeADir;
                    //                    e1 = edgeBDir;
                    edgeInfo.points.push_back(closestPointBetweenLines(pA, pB, edgeADir, edgeBDir));
                }
            }
        }

        edgeInfo.normal = -edgeInfo.normal;

//        glm::vec3 vRel = -cubeB->rb->velocity+cubeA->rb->velocity;
//        float mag = glm::abs(glm::dot(vRel, edgeInfo.normal));

//        glm::vec3 vParallel = mag*edgeInfo.normal;
//        glm::vec3 vPerp = glm::cross(vRel, edgeInfo.normal);
//        cubeB->rb->addForce(vParallel/dt);

//        cubeB->rb->addForce(edgeInfo.normal/dt);

    }
    cubeA->collisionDetected = true;
    cubeB->collisionDetected = true;
    return true;


}

bool PhysicsWorld::detectSphereSphereCollision(SphereCollider* sphere, SphereCollider* other)
{
    glm::vec3 dp = other->rb->position-sphere->rb->position;
    float lSquared = glm::length2(dp);
    float minDist = other->radius+sphere->radius;
    bool result = lSquared<=minDist*minDist;
    sphere->collisionDetected = result;
    other->collisionDetected = result;
    return result;
}

void PhysicsWorld::sphereSphereCollisionResponse(float dt, SphereCollider* sphere, SphereCollider* other)
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
void PhysicsWorld::spherePlaneCollision(float dt, SphereCollider* sphere)
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
void PhysicsWorld::updateQuantities(float dt)
{
    for(const auto& collider: colliders)
    {
        if(collider->rb!=nullptr)
            collider->rb->stepQuantities(dt);
    }
}
void PhysicsWorld::stepWorld(float dt)
{
    updateQuantities(dt);
    checkForCollisions(dt);
}
