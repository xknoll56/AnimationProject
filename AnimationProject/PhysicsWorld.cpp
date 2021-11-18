#include "PhysicsWorld.h"


PhysicsWorld::PhysicsWorld(std::vector<Collider*>* colliders, glm::vec3 gravity)
{
    this->gravity = gravity;
    this->colliders.reserve(colliders->size());
    unsigned int i = 0;
    for(auto& collider: *colliders)
    {
        collider->id = i++;
        this->colliders.push_back(collider);
    }
    for(auto& collider: *colliders)
    {
        if(collider->rb!=nullptr)
            collider->rb->gravitionalForce += collider->rb->mass*gravity;
    }

}

PhysicsWorld::PhysicsWorld(std::vector<Collider*>* colliders)
{
    gravity = glm::vec3(0, -9.81f, 0);
    this->colliders.reserve(colliders->size());
    unsigned int i = 0;
    for(auto& collider: *colliders)
    {
        collider->id = i++;
        this->colliders.push_back(collider);
    }
    for(auto& collider: *colliders)
    {
        if(collider->rb!=nullptr)
            collider->rb->gravitionalForce += collider->rb->mass*gravity;
    }

}

PhysicsWorld::PhysicsWorld()
{
    gravity = glm::vec3(0, -9.81f, 0);
}

void PhysicsWorld::setColliders(std::vector<Collider *> *colliders)
{
    this->colliders.reserve(colliders->size());
    unsigned int i = 0;
    for(auto& collider: *colliders)
    {
        collider->id = i++;
        this->colliders.push_back(collider);
    }
    for(auto& collider: *colliders)
    {
        if(collider->rb!=nullptr)
            collider->rb->gravitionalForce += collider->rb->mass*gravity;
    }
}

bool PhysicsWorld::contactHandled(Collider* a, Collider* b)
{
    for(ContactInfo& contact: contacts)
    {
        if((contact.a->id == a->id && contact.b->id == b->id) ||(contact.b->id == a->id && contact.a->id == b->id))
            return true;
    }

    return false;
}


void PhysicsWorld::checkForCollisions(float dt)
{
    contacts.clear();
    for(auto& collider: colliders)
    {
        switch(collider->type)
        {
        case ColliderType::SPHERE:
        {
            SphereCollider* sphere = dynamic_cast<SphereCollider*>(collider);
            //spherePlaneCollision(dt, sphere);
            for(auto& other: colliders)
            {
                if(other!=collider)
                {
                    switch(other->type)
                    {
                    case ColliderType::SPHERE:
                    {
                        SphereCollider* otherSphere = dynamic_cast<SphereCollider*>(other);
                        if(detectSphereSphereCollision(sphere, otherSphere))
                            sphereSphereCollisionResponse(dt, sphere, otherSphere);
                        break;
                    }
                    case ColliderType::CUBE:
                    {
                        CubeCollider* otherCube = dynamic_cast<CubeCollider*>(other);
                        ContactInfo info;
                        if(!contactHandled(otherCube, sphere))
                        {
                            if(detectCubeSphereCollision( dt, otherCube, sphere, info))
                            {

                            }
                        }
                        break;
                    }
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
                    {
                        CubeCollider* otherCube = dynamic_cast<CubeCollider*>(other);
                        if(!contactHandled(cube, otherCube))
                        {
                            ContactInfo info;
                            if(detectCubeCubeCollision(dt, cube, otherCube, info))
                                determineCubeCubeContactPoints(info, cube, otherCube);
                        }
                        break;
                    }
                    case ColliderType::SPHERE:
                    {
                        SphereCollider* sphere = dynamic_cast<SphereCollider*>(other);
                        ContactInfo info;
                        if(!contactHandled(cube, sphere))
                        {
                            if(detectCubeSphereCollision( dt, cube, sphere, info))
                            {
                                //qDebug() << "collision";
                            }
                        }
                        break;
                    }
                    }
                }
            }
            break;
        }
        }
    }

    CollisionResponse(dt);
}

void PhysicsWorld::CollisionResponse(float dt)
{
    if(enableResponse)
    {
        for(ContactInfo& info: contacts)
        {
            if(info.a->type == ColliderType::CUBE && info.b->type == ColliderType::CUBE)
            {
                CubeCollider* cube = dynamic_cast<CubeCollider*>(info.a);
                CubeCollider* otherCube = dynamic_cast<CubeCollider*>(info.b);
                if(cube && otherCube)
                {
                    if(cube->rb->isStatic() && !otherCube->rb->isStatic())
                    {
                        cubeCubeCollisionResponseDynamicVsStatic(info, -info.normal, dt, otherCube, cube);
                    }
                    else if(otherCube->rb->isStatic() && !cube->rb->isStatic())
                    {
                        cubeCubeCollisionResponseDynamicVsStatic(info, info.normal, dt, cube, otherCube);
                    }
                    else if(!cube->rb->isStatic() && !otherCube->rb->isStatic())
                        cubeCubeCollisionResponse(info, dt, cube, otherCube);
                }
            }
            else if(info.a->type == ColliderType::CUBE && info.b->type == ColliderType::SPHERE)
            {
                CubeCollider* cube = dynamic_cast<CubeCollider*>(info.a);
                SphereCollider* sphere = dynamic_cast<SphereCollider*>(info.b);
                if(cube && sphere)
                {
                    if(cube->rb->isStatic())
                    {
                        cubeSphereCollisionResponseStaticVsDynamic(info, dt, cube, sphere);
                    }
                    else
                        cubeSphereCollisionResponseDynamicVsDynamic(info, dt, cube, sphere);
                }
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
    return 0.5f*(p0+s*u+p1+t*v);
}

float PhysicsWorld::closestDistanceBetweenLines(glm::vec3& p0,  glm::vec3& p1, const glm::vec3& u, const glm::vec3& v, float s0, float s1)
{
    glm::vec3 uv = glm::cross(v,u);
    float uvSquared = glm::length2(uv);
    float t = -glm::dot(glm::cross(p1-p0, u), uv)/uvSquared;
    float s = -glm::dot(glm::cross(p1-p0, v), uv)/uvSquared;
    if(glm::abs(t)>s1 || glm::abs(s)>s0 || glm::epsilonEqual(t, 0.0f, 0.01f) || glm::epsilonEqual(s, 0.0f, 0.01f))
        return std::numeric_limits<float>::max();

    return glm::abs(glm::dot(uv, p0-p1)/glm::length(uv));
}

bool PhysicsWorld::closestPointsDoIntersect(glm::vec3& p0,  glm::vec3& p1,  const glm::vec3& u, const glm::vec3& v, float s0, float s1, float max1, float max2)
{
    glm::vec3 uv = glm::cross(v,u);
    float uvSquared = glm::length2(uv);
    float t = -glm::dot(glm::cross(p1-p0, u), uv)/uvSquared;
    float s = -glm::dot(glm::cross(p1-p0, v), uv)/uvSquared;
    float dist = glm::abs(glm::dot(uv, p0-p1)/glm::length(uv));
    if(glm::abs(t)>s1 || glm::abs(s)>s0  || std::isnan(t) || std::isnan(s))
        return false;

    return true;
}


bool PhysicsWorld::detectCubeSphereCollision(float dt, CubeCollider* cube, SphereCollider* sphere, ContactInfo& contactInfo)
{
    cube->collisionDetected = false;
    sphere->collisionDetected = false;

    contactInfo.a = cube;
    contactInfo.b = sphere;
    contactInfo.faceCollision = false;

    glm::vec3 aX = cube->rb->getLocalXAxis();
    glm::vec3 aY = cube->rb->getLocalYAxis();
    glm::vec3 aZ = cube->rb->getLocalZAxis();

    glm::vec3 T = cube->rb->position-sphere->rb->position;

    float penetrationY, penetrationX, penetrationZ;

    float penetration = glm::abs(glm::dot(aX, T)) - cube->xSize - sphere->radius;
    if(penetration > 0)
    {

        return false;
    }
    contactInfo.aDir = CubeCollider::ContactDir::RIGHT;
    contactInfo.penetrationDistance = penetration;
    penetrationX = glm::abs(penetration);

    penetration = glm::abs(glm::dot(aY, T)) - cube->ySize - sphere->radius;
    if(penetration > 0)
    {
        return false;
    }

    penetrationY = glm::abs(penetration);
    if(penetration > contactInfo.penetrationDistance)
    {
        contactInfo.aDir = CubeCollider::ContactDir::UP;
        contactInfo.penetrationDistance = penetration;
    }



    penetration = glm::abs(glm::dot(aZ, T)) - cube->zSize - sphere->radius;
    if(penetration > 0)
    {
        return false;
    }
    penetrationZ = penetration;
    if(penetration > contactInfo.penetrationDistance)
    {
        contactInfo.aDir = CubeCollider::ContactDir::FORWARD;
        contactInfo.penetrationDistance = penetration;
    }


    if(cube->rb->atRest)
        cube->rb->atRest = false;

    glm::vec3 normal, normalX, normalY, normalZ;
    normalX = glm::sign(glm::dot(aX, T))*aX;
    normalY = glm::sign(glm::dot(aY, T))*aY;
    normalZ = glm::sign(glm::dot(aZ, T))*aZ;
    float size;
    glm::vec3 castDir;
    switch(contactInfo.aDir)
    {
    case(CubeCollider::ContactDir::RIGHT):
        normal = normalX;
        size = cube->xSize;
        castDir = glm::normalize(-normalY-normalZ);
        if(cubeRaycast(sphere->rb->position, normalX, rcd, cube))
        {
            if(rcd.length<sphere->radius)
            {
                contactInfo.points.push_back(rcd.point);
                contactInfo.normal = -normalX;
                contactInfo.faceCollision = true;
                contacts.push_back(contactInfo);
                return true;
            }
        }
        break;
    case(CubeCollider::ContactDir::UP):
        normal = normalY;
        size = cube->ySize;
        castDir = glm::normalize(-normalX-normalZ);
        if(cubeRaycast(sphere->rb->position, normalY, rcd, cube))
        {
            if(rcd.length<sphere->radius)
            {
                contactInfo.points.push_back(rcd.point);
                contactInfo.normal = -normalY;
                contactInfo.faceCollision = true;
                contacts.push_back(contactInfo);
                return true;
            }
        }
        break;
    case(CubeCollider::ContactDir::FORWARD):
        if(cubeRaycast(sphere->rb->position, normalZ, rcd, cube))
        {
            if(rcd.length<sphere->radius)
            {
                contactInfo.points.push_back(rcd.point);
                contactInfo.normal = -normalZ;
                contactInfo.faceCollision = true;
                contacts.push_back(contactInfo);
                return true;
            }
        }
        break;
    }

    //Thats not all, need to do the edges...
    std::vector<glm::vec3> closestVerts = cube->getClosestVerts(T);
    std::vector<CubeCollider::EdgeIndices> edges = cube->getEdgesFromVertexIndices();

    RayCastData rcd2;
    for(CubeCollider::EdgeIndices edge: edges)
    {
        glm::vec3 point1 = edge.midPoint-edge.dir*edge.length;
        glm::vec3 dir1 = edge.dir;

        glm::vec3 point2 = edge.midPoint+edge.dir*edge.length;
        glm::vec3 dir2 = -edge.dir;
        if(sphereRaycast(point1, dir1, rcd, sphere) && sphereRaycast(point2, dir2, rcd2, sphere))
        {
            glm::vec3 contactPoint = (rcd.point+rcd2.point)*0.5f;
            contactInfo.points.push_back(contactPoint);
            contactInfo.normal = glm::normalize(sphere->rb->position-contactPoint);
            contactInfo.penetrationDistance = glm::length(contactPoint-sphere->rb->position);
            contactInfo.faceCollision = false;
            contacts.push_back(contactInfo);
            return true;
        }
    }


    if(glm::abs(glm::dot(aX, T))<cube->xSize && glm::abs(glm::dot(aZ, T))<cube->zSize && glm::abs(glm::dot(aY, T))<cube->ySize)
    {
        contactInfo.points.push_back(sphere->rb->position);
        contactInfo.normal = -normal;
        contactInfo.faceCollision = true;
        contacts.push_back(contactInfo);
        return true;
    }





    return false;

}


bool PhysicsWorld::detectCubeCubeCollision(float dt, CubeCollider* cubeA, CubeCollider* cubeB, ContactInfo& contactInfo)
{
    if(!cubeA->rb->dynamic && !cubeB->rb->dynamic)
        return false;

    glm::vec3 aX = cubeA->rb->getLocalXAxis();
    glm::vec3 aY = cubeA->rb->getLocalYAxis();
    glm::vec3 aZ = cubeA->rb->getLocalZAxis();

    glm::vec3 bX = cubeB->rb->getLocalXAxis();
    glm::vec3 bY = cubeB->rb->getLocalYAxis();
    glm::vec3 bZ = cubeB->rb->getLocalZAxis();

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
    cubeA->rb->applyGravity = true;
    cubeB->rb->applyGravity = true;
    bool faceToFace = false;
    ContactInfo faceInfo;
    ContactInfo edgeInfo;
    float t1 = 0, t2 = 0;

    float lowestAPenetration = -std::numeric_limits<float>().max();
    float lowestBPenetration = -std::numeric_limits<float>().max();

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
    t1 += penetration;

    penetration = glm::abs(glm::dot(T, aY)) - (cubeA->ySize + glm::abs(cubeB->xSize*ryx) + glm::abs(cubeB->ySize*ryy) + glm::abs(cubeB->zSize*ryz));
    //check for collisions parallel to AY
    if(penetration>0)
    {
        return false;
    }
    if(penetration > faceInfo.penetrationDistance)
    {
        faceInfo.penetrationDistance = penetration;
        faceInfo.normal = aY;
        faceInfo.faceCollision = true;
        faceInfo.aDir = CubeCollider::ContactDir::UP;
    }
    t1 += penetration;

    penetration = glm::abs(glm::dot(T, aZ)) - (cubeA->zSize + glm::abs(cubeB->xSize*rzx) + glm::abs(cubeB->ySize*rzy) + glm::abs(cubeB->zSize*rzz));
    //check for collisions parallel to AZ
    if(penetration>0)
    {
        return false;
    }
    if(penetration > faceInfo.penetrationDistance )
    {
        faceInfo.penetrationDistance = penetration;
        faceInfo.normal = aZ;
        faceInfo.faceCollision = true;
        faceInfo.aDir = CubeCollider::ContactDir::FORWARD;
    }
    t1 += penetration;

    penetration = glm::abs(glm::dot(T, bX)) - (glm::abs(cubeA->xSize*rxx) + glm::abs(cubeA->ySize*ryx) + glm::abs(cubeA->zSize*rzx) + cubeB->xSize);
    //check for collisions parallel to BX
    if(penetration>0)
    {
        return false;
    }
    if(penetration > lowestBPenetration)
    {
        lowestBPenetration = penetration;

        faceInfo.faceCollision = true;
        faceInfo.bDir = CubeCollider::ContactDir::RIGHT;
        if(penetration>faceInfo.penetrationDistance)
        {
            faceInfo.penetrationDistance = penetration;
            faceInfo.normal = bX;
        }
    }

    penetration = glm::abs(glm::dot(T, bY)) - (glm::abs(cubeA->xSize*rxy) + glm::abs(cubeA->ySize*ryy) + glm::abs(cubeA->zSize*rzy) + cubeB->ySize);
    //check for collisions parallel to BY
    if(penetration>0)
    {
        return false;
    }
    if(penetration > lowestBPenetration)
    {
        lowestBPenetration = penetration;

        faceInfo.faceCollision = true;
        faceInfo.bDir = CubeCollider::ContactDir::UP;
        if(penetration>faceInfo.penetrationDistance)
        {
            faceInfo.penetrationDistance = penetration;
            faceInfo.normal = bY;
        }
    }

    penetration = glm::abs(glm::dot(T, bZ)) - (glm::abs(cubeA->xSize*rxz) + glm::abs(cubeA->ySize*ryz) + glm::abs(cubeA->zSize*rzz) + cubeB->zSize);
    //check for collisions parallel to BZ
    if(penetration>0)
    {
        return false;
    }
    if(penetration > lowestBPenetration)
    {
        lowestBPenetration = penetration;

        faceInfo.faceCollision = true;
        faceInfo.bDir = CubeCollider::ContactDir::FORWARD;
        if(penetration>faceInfo.penetrationDistance)
        {
            faceInfo.penetrationDistance = penetration;
            faceInfo.normal = bZ;
        }
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
    t2 += penetration;

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
    t2+=penetration;

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
    t2+=penetration;

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
    t2+=penetration;

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
    t2+=penetration;

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
    t2+=penetration;

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
    t2+=penetration;

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
    t2+=penetration;

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
    t2+=penetration;



    if((cubeA->rb->atRest && !cubeB->rb->atRest) && cubeB->rb->dynamic)
        cubeA->rb->atRest =false;
    if((!cubeA->rb->atRest && cubeB->rb->atRest) && cubeA->rb->dynamic)
        cubeB->rb->atRest =false;

    faceInfo.normal = glm::sign(glm::dot(T, faceInfo.normal))*faceInfo.normal;
    faceInfo.normal = glm::normalize(faceInfo.normal);
    std::vector<glm::vec3> closestVertsA = cubeA->getClosestVerts(-faceInfo.normal);
    std::vector<glm::vec3> closestVertsB = cubeB->getClosestVerts(faceInfo.normal);
    bool facecollision = false;

    facecollision = isCubeCubePetrusion(-faceInfo.normal, closestVertsA, cubeB, faceInfo.bDir)||isCubeCubePetrusion(faceInfo.normal, closestVertsB, cubeA, faceInfo.aDir);
    // }
    //    else if(faceToFace)
    //    {
    //        //        facecollision = isCubeCubePetrusion(-faceInfo.normal, closestVertsA, cubeB, faceInfo.bDir)||
    //        //                isCubeCubePetrusion(faceInfo.normal, closestVertsB, cubeA, faceInfo.aDir);
    //        facecollision = true;

    //    }
    if(facecollision)
    {
        contactInfo = faceInfo;
    }
    else
    {
        //edgeInfo.normal = faceInfo.normal;
        contactInfo = edgeInfo;

    }

    contactInfo.a = cubeA;
    contactInfo.b = cubeB;


    cubeA->collisionDetected = true;
    cubeB->collisionDetected = true;
    return true;


}

void PhysicsWorld::determineCubeCubeContactPoints(ContactInfo& info, CubeCollider* cubeA, CubeCollider* cubeB)
{
    glm::vec3 T = cubeB->rb->position - cubeA->rb->position;
    info.normal = glm::sign(glm::dot(T, info.normal))*info.normal;
    info.normal = glm::normalize(info.normal);
    std::vector<glm::vec3> closestVertsA = cubeA->getClosestVerts(-info.normal);
    std::vector<glm::vec3> closestVertsB = cubeB->getClosestVerts(info.normal);


    std::vector<CubeCollider::EdgeIndices> aEdges = cubeA->getEdgesFromVertexIndices();
    std::vector<CubeCollider::EdgeIndices> bEdges = cubeB->getEdgesFromVertexIndices();
    float minDist = std::numeric_limits<float>().max();
    RayCastData data;
    for(CubeCollider::EdgeIndices ea: aEdges)
    {
        for(CubeCollider::EdgeIndices eb: bEdges)
        {
            //            glm::vec3 tangentialA = ea.dir*ea.length - glm::dot(ea.dir*ea.length, info.normal)*info.normal;
            //            float aLen = glm::length(tangentialA);
            //            glm::vec3 tangentialB = eb.dir*eb.length - glm::dot(eb.dir*eb.length, info.normal)*info.normal;
            //            float bLen = glm::length(tangentialB);
            //            tangentialA /= aLen;
            //            tangentialB /= bLen;

            if(closestPointsDoIntersect(ea.midPoint, eb.midPoint, ea.dir, eb.dir, ea.length, eb.length, ea.normalLength, eb.normalLength))
            {
                glm::vec3 testPoint = closestPointBetweenLines(ea.midPoint, eb.midPoint, ea.dir, eb.dir);
                glm::vec3 pa1 = ea.midPoint+ea.dir*ea.length;
                glm::vec3 pa2 = ea.midPoint-ea.dir*ea.length;
                glm::vec3 pb1 = eb.midPoint+eb.dir*eb.length;
                glm::vec3 pb2 = eb.midPoint-eb.dir*eb.length;
                if((cubeRaycast(pa1, -ea.dir, data, cubeB) || cubeRaycast(pa2, ea.dir, data, cubeB))&&(cubeRaycast(pb1, -eb.dir, data, cubeA) || cubeRaycast(pb2, eb.dir, data, cubeA)))
                {
                    float dist = closestDistanceBetweenLines(ea.midPoint, eb.midPoint, ea.dir, eb.dir, ea.length, eb.length);
                    if(dist<info.penetrationDistance && !info.faceCollision)
                        info.penetrationDistance = dist;
                    info.points.push_back(testPoint);
                    info.edgePoints++;
                }
            }
        }
    }


    if(info.faceCollision)
    {
        determineCubeCubePetrusionVerts(info, -info.normal, closestVertsA, cubeB, info.bDir, true);
        determineCubeCubePetrusionVerts(info, info.normal, closestVertsB, cubeA, info.aDir, true);
    }

    contacts.push_back(info);
}

bool PhysicsWorld::isCubeCubePetrusion(const glm::vec3& normal, const std::vector<glm::vec3>& points, CubeCollider* toCube, CubeCollider::ContactDir dir)
{
    glm::vec3 p0 = toCube->rb->position + toCube->rb->getLocalXAxis()*toCube->xSize;
    glm::vec3 adj1 = toCube->rb->getLocalYAxis();
    float maxDist1 = toCube->ySize;
    glm::vec3 adj2 = toCube->rb->getLocalZAxis();
    float maxDist2 = toCube->zSize;
    float tolerance = 0.005f;
    switch(dir)
    {
    case CubeCollider::ContactDir::RIGHT:
        if(glm::dot(normal, toCube->rb->getLocalXAxis())<0)
            p0 = toCube->rb->position - toCube->rb->getLocalXAxis()*toCube->xSize;
        break;
    case CubeCollider::ContactDir::UP:
        p0 = toCube->rb->position + toCube->rb->getLocalYAxis()*toCube->ySize;
        if(glm::dot(normal, toCube->rb->getLocalYAxis())<0)
            p0 = toCube->rb->position - toCube->rb->getLocalYAxis()*toCube->ySize;
        adj1 = toCube->rb->getLocalXAxis();
        maxDist1 = toCube->xSize;
        adj2 = toCube->rb->getLocalZAxis();
        maxDist2 = toCube->zSize;
        break;
    case CubeCollider::ContactDir::FORWARD:
        p0 = toCube->rb->position + toCube->rb->getLocalZAxis()*toCube->zSize;
        if(glm::dot(normal, toCube->rb->getLocalZAxis())<0)
            p0 = toCube->rb->position - toCube->rb->getLocalZAxis()*toCube->zSize;
        adj1 = toCube->rb->getLocalXAxis();
        maxDist1 = toCube->xSize;
        adj2 = toCube->rb->getLocalYAxis();
        maxDist2 = toCube->ySize;
        break;
    }

    for(glm::vec3 point: points)
    {
        float d = glm::dot(p0-point, normal)/glm::dot(normal,normal);
        if(d>=-tolerance)
        {
            glm::vec3 intersectionPoint = point+d*normal;
            float dist1 = glm::abs(glm::dot(intersectionPoint-p0, adj1));
            float dist2 = glm::abs(glm::dot(intersectionPoint-p0, adj2));
            if(dist1<=maxDist1+tolerance && dist2<=maxDist2+tolerance)
                return true;
        }
    }
    return false;
}

bool PhysicsWorld::sphereRaycast(const glm::vec3& start, const glm::vec3& dir, RayCastData& dat, SphereCollider* sphere)
{
    glm::vec3 right(1,0,0), up(0,1,0);
    if(glm::all(glm::equal(dir, glm::vec3(0,1,0)))  || glm::all(glm::equal(dir, glm::vec3(0,-1,0))))
    {
        up = glm::vec3(0,0,1);
    }
    else
    {
        right = glm::normalize(glm::cross(up, dir));
        up = glm::cross(right, dir);
    }

    bool hits = false;
    glm::vec3 p0 = sphere->rb->position;
    glm::vec3 normal = -dir;
    float dist = glm::dot((p0-start),normal)/glm::dot(dir, normal);

    if(dist>=0.0f)
    {
        glm::vec3 intersection = start+dir*dist;
        glm::vec3 radius = intersection-p0;
        float l2 = glm::length2(radius);
        if(l2<=sphere->radius*sphere->radius)
        {
            hits = true;
            glm::vec3 rightComp = glm::dot(right, radius)*right;
            glm::vec3 upComp = glm::dot(up, radius)*up;
            float length = glm::length(rightComp+upComp);
            float theta = glm::asin(length/sphere->radius);
            glm::vec3 dirComp = normal*length/glm::tan(theta);
            if(length==0.0f)
                dirComp = normal*sphere->radius;
            glm::vec3 point = rightComp+upComp+dirComp+p0;
            dat.length = glm::length(point-start);
            dat.normal = rightComp+upComp+dirComp;
            dat.point = point;
            dat.collider = sphere;
        }
    }

    return hits;
}

bool PhysicsWorld::cubeRaycast(const glm::vec3& start, const glm::vec3& dir, RayCastData& dat, CubeCollider* cube)
{
    float minDist = std::numeric_limits<float>().max();
    //normal of the plane
    glm::vec3 normal;
    //initial points on plane
    glm::vec3 p0;
    //The adjacent directions to check if its on the plane
    glm::vec3 adj1;
    float maxDist1;
    glm::vec3 adj2;
    float maxDist2;
    bool doesHit = false;

    //start with the left and right faces
    for(int i = 0;i<6;i++)
    {
        switch(i)
        {
        case 0:
            normal = -cube->rb->getLocalXAxis();
            p0 = cube->rb->position-cube->xSize*cube->rb->getLocalXAxis();
            adj1 = cube->rb->getLocalYAxis();
            adj2 = cube->rb->getLocalZAxis();
            maxDist1 = cube->ySize;
            maxDist2 = cube->zSize;
            break;
        case 1:
            normal = cube->rb->getLocalXAxis();
            p0 = cube->rb->position+cube->xSize*cube->rb->getLocalXAxis();
            adj1 = cube->rb->getLocalYAxis();
            adj2 = cube->rb->getLocalZAxis();
            maxDist1 = cube->ySize;
            maxDist2 = cube->zSize;
            break;
        case 2:
            normal = -cube->rb->getLocalYAxis();
            p0 = cube->rb->position-cube->ySize*cube->rb->getLocalYAxis();
            adj1 = cube->rb->getLocalXAxis();
            adj2 = cube->rb->getLocalZAxis();
            maxDist1 = cube->xSize;
            maxDist2 = cube->zSize;
            break;
        case 3:
            normal = cube->rb->getLocalYAxis();
            p0 = cube->rb->position+cube->ySize*cube->rb->getLocalYAxis();
            adj1 = cube->rb->getLocalXAxis();
            adj2 = cube->rb->getLocalZAxis();
            maxDist1 = cube->xSize;
            maxDist2 = cube->zSize;
            break;
        case 4:
            normal = -cube->rb->getLocalZAxis();
            p0 = cube->rb->position-cube->zSize*cube->rb->getLocalZAxis();
            adj1 = cube->rb->getLocalYAxis();
            adj2 = cube->rb->getLocalXAxis();
            maxDist1 = cube->ySize;
            maxDist2 = cube->xSize;
            break;
        case 5:
            normal = cube->rb->getLocalZAxis();
            p0 = cube->rb->position+cube->zSize*cube->rb->getLocalZAxis();
            adj1 = cube->rb->getLocalYAxis();
            adj2 = cube->rb->getLocalXAxis();
            maxDist1 = cube->ySize;
            maxDist2 = cube->xSize;
            break;
        }

        float dist = glm::dot((p0-start),normal)/glm::dot(dir, normal);
        if(dist>=0.0f)
        {
            glm::vec3 intersection = start + dir*dist;
            float dist1 = glm::abs(glm::dot(intersection-p0, adj1));
            float dist2 = glm::abs(glm::dot(intersection-p0, adj2));
            if(dist1<=maxDist1 && dist2<=maxDist2)
            {
                //cast hits, check if its the minimum
                if(dist<minDist)
                {
                    minDist = dist;
                    dat.normal = normal;
                    dat.point = intersection;
                    dat.length = dist;
                    doesHit = true;
                    dat.collider = cube;
                }
            }

        }
    }

    return doesHit;
}

void PhysicsWorld::determineCubeCubePetrusionVerts(ContactInfo& info, const glm::vec3& normal, const std::vector<glm::vec3>& points, CubeCollider* toCube, CubeCollider::ContactDir dir, bool adjustPenetration)
{
    glm::vec3 p0 = toCube->rb->position + toCube->rb->getLocalXAxis()*toCube->xSize;
    glm::vec3 adj1 = toCube->rb->getLocalYAxis();
    float maxDist1 = toCube->ySize;
    glm::vec3 adj2 = toCube->rb->getLocalZAxis();
    float maxDist2 = toCube->zSize;
    switch(dir)
    {
    case CubeCollider::ContactDir::RIGHT:
        if(glm::dot(normal, toCube->rb->getLocalXAxis())<0)
            p0 = toCube->rb->position - toCube->rb->getLocalXAxis()*toCube->xSize;
        break;
    case CubeCollider::ContactDir::UP:
        p0 = toCube->rb->position + toCube->rb->getLocalYAxis()*toCube->ySize;
        if(glm::dot(normal, toCube->rb->getLocalYAxis())<0)
            p0 = toCube->rb->position - toCube->rb->getLocalYAxis()*toCube->ySize;
        adj1 = toCube->rb->getLocalXAxis();
        maxDist1 = toCube->xSize;
        adj2 = toCube->rb->getLocalZAxis();
        maxDist2 = toCube->zSize;
        break;
    case CubeCollider::ContactDir::FORWARD:
        p0 = toCube->rb->position + toCube->rb->getLocalZAxis()*toCube->zSize;
        if(glm::dot(normal, toCube->rb->getLocalZAxis())<0)
            p0 = toCube->rb->position - toCube->rb->getLocalZAxis()*toCube->zSize;
        adj1 = toCube->rb->getLocalXAxis();
        maxDist1 = toCube->xSize;
        adj2 = toCube->rb->getLocalYAxis();
        maxDist2 = toCube->ySize;
        break;
    }

    for(glm::vec3 point: points)
    {
        float d = glm::dot(p0-point, normal)/glm::dot(normal,normal);
        //This now becomes the new penetration distance
        //qDebug() << "Penetration distance: " << d;

        if(d>=0.0f)
        {
            glm::vec3 intersectionPoint = point+d*normal;
            float dist1 = glm::abs(glm::dot(intersectionPoint-p0, adj1));
            float dist2 = glm::abs(glm::dot(intersectionPoint-p0, adj2));
            if(dist1<=maxDist1 && dist2<=maxDist2)
            {
                info.points.push_back(intersectionPoint);
                if(adjustPenetration && d>=0.0f  && d>=-info.penetrationDistance)
                {
                    info.penetrationDistance = -d;
                    info.vertexPoints = info.points.size()-1;
                }

            }
        }
    }
}

void PhysicsWorld::cubeCubeCollisionResponse(ContactInfo& info, float dt, CubeCollider* cubeA, CubeCollider* cubeB)
{

    cubeA->rb->position += 0.5f*info.normal*info.penetrationDistance;
    cubeB->rb->position -= 0.5f*info.normal*info.penetrationDistance;


    for(int i =0;i<info.points.size();i++)
    {
        float epsilon = 0.5f;
        glm::vec3 ra = info.points[i]-cubeA->rb->position;
        glm::vec3 rb = info.points[i]-cubeB->rb->position;
        glm::vec3 va = cubeA->rb->velocity + glm::cross(cubeA->rb->angularVelocity, ra);
        glm::vec3 vb = cubeB->rb->velocity + glm::cross(cubeB->rb->angularVelocity, rb);
        float vRel = glm::dot(info.normal, va-vb);

        //glm::vec3 vPerp = glm::normalize(glm::cross(info.normal, va-vb));
        //qDebug() << "vPerp x:" << vPerp.x << " y: " << vPerp.y << " z: " << vPerp.z;
        float numerator = -(1-epsilon)*vRel;

        float t1 = cubeA->rb->massInv;
        float t2 = cubeB->rb->massInv;
        float t3 = glm::dot(info.normal, glm::cross(glm::cross(cubeA->rb->inertiaInv*ra, info.normal), ra));
        float t4 = glm::dot(info.normal, glm::cross(glm::cross(cubeB->rb->inertiaInv*rb, info.normal), rb));

        float j = numerator/(t1+t2+t3+t4);
        glm::vec3 force = j*info.normal/(dt*info.points.size());
        float angularRel = glm::length(cubeA->rb->angularVelocity-cubeB->rb->angularVelocity);


        if(cubeA->rb->dynamic)
        {
            glm::vec3 perpVelNormal = glm::normalize(cubeA->rb->velocity - va*info.normal);
            if(!cubeB->rb->dynamic){
                glm::vec3 fric = friction*perpVelNormal*cubeA->rb->mass*glm::dot(gravity, info.normal);
                force-=fric;
            }
            cubeA->rb->addForce(force, *cubeB->rb);
            cubeA->rb->addTorque(glm::cross(ra, force));
        }
        if(cubeB->rb->dynamic)
        {
            glm::vec3 perpVelNormal = glm::normalize(cubeB->rb->velocity + vb*info.normal);
            if(!cubeA->rb->dynamic)
            {
                glm::vec3 fric = friction*perpVelNormal*cubeB->rb->mass*glm::dot(gravity, info.normal);
                force-=fric;
            }
            cubeB->rb->addForce(-force, *cubeA->rb);
            cubeB->rb->addTorque(-glm::cross(rb, force));
        }
        //qDebug() << "not resting";
    }

}

glm::vec3 landingHorizontalSpeed;
glm::vec3 frictionalSpeed;
void PhysicsWorld::cubeCubeCollisionResponseDynamicVsStatic(ContactInfo& info, const glm::vec3& norm, float dt, CubeCollider* dynamicCube, CubeCollider* staticCube)
{

    if(info.faceCollision)
        dynamicCube->rb->position += norm*info.penetrationDistance;

    if(!dynamicCube->rb->restingContact)
    {
        // qDebug() << "not stabalizing";
        // if(info.points.size()==0)
        //dynamicCube->rb->restingContact = true;
        for(int i =0;i<info.points.size();i++)
        {
            float epsilon = 0.5f;
            glm::vec3 ra = info.points[i]-dynamicCube->rb->position;
            glm::vec3 rb = info.points[i]-staticCube->rb->position;
            glm::vec3 va = dynamicCube->rb->velocity + glm::cross(dynamicCube->rb->angularVelocity, ra);
            float vRel = glm::dot(info.normal, va);

            float numerator = -(1-epsilon)*vRel;

            float t1 = dynamicCube->rb->massInv;
            float t2 = staticCube->rb->massInv;
            float t3 = glm::dot(info.normal, glm::cross(glm::cross(dynamicCube->rb->inertiaInv*ra, info.normal), ra));
            float t4 = glm::dot(info.normal, glm::cross(glm::cross(staticCube->rb->inertiaInv*rb, info.normal), rb));

            float j = numerator/(t1+t2+t3+t4);
            glm::vec3 force = j*info.normal/(dt*info.points.size());
            float angularRel = glm::length(dynamicCube->rb->angularVelocity-staticCube->rb->angularVelocity);


            // qDebug() << "surface slope: " << glm::abs(glm::dot(glm::normalize(gravity), info.normal));
            if(glm::abs(j)<0.5f && glm::abs(glm::dot(glm::normalize(gravity), info.normal))>0.7f)
            {
                dynamicCube->rb->restingContact = true;
                landingHorizontalSpeed = dynamicCube->rb->velocity-glm::dot(dynamicCube->rb->velocity, norm)*norm;

            }

            dynamicCube->rb->addForce(force, *dynamicCube->rb);
            dynamicCube->rb->addTorque(glm::cross(ra, force));
        }
    }
    else
    {

        float angularSpeed = glm::length(dynamicCube->rb->angularVelocity);
        float speed = glm::length(dynamicCube->rb->velocity);

        if(speed<1.0f && angularSpeed<1.0f)
        {
            dynamicCube->rb->restingContact = false;
            dynamicCube->rb->atRest = true;
            bool reverseDir;
            CubeCollider::ContactDir upDir = dynamicCube->GetFaceClosestToNormal(info.normal, reverseDir);
            glm::vec3 up = info.normal;
            if(reverseDir)
                up = -up;
            glm::quat toRotation = dynamicCube->toRotation(upDir, up);

            dynamicCube->rb->rotation = toRotation;
            //dynamicCube->rb->rotation = glm::slerp(dynamicCube->rb->rotation, toRotation, 0.1f);

        }
        else if(info.points.size())
        {
            //if(glm::abs(glm::dot(glm::normalize(gravity), info.normal))<=0.7f)
            {
                // dynamicCube->rb->restingContact = false;
                // return;
            }
            glm::vec3 totalVelocity(0,0,0);
            for(int i =0;i<info.points.size();i++)
            {
                float epsilon = 0.5f;
                glm::vec3 ra = info.points[i]-dynamicCube->rb->position;
                glm::vec3 rb = info.points[i]-staticCube->rb->position;
                glm::vec3 va = dynamicCube->rb->velocity + glm::cross(dynamicCube->rb->angularVelocity, ra);
                glm::vec3 vn = glm::dot(va, info.normal)*info.normal;
                glm::vec3 vt = va-vn;
                float vRel = glm::dot(info.normal, va);

                float numerator = -(1-epsilon)*vRel;

                float t1 = dynamicCube->rb->massInv;
                float t2 = staticCube->rb->massInv;
                float t3 = glm::dot(info.normal, glm::cross(glm::cross(dynamicCube->rb->inertiaInv*ra, info.normal), ra));
                float t4 = glm::dot(info.normal, glm::cross(glm::cross(staticCube->rb->inertiaInv*rb, info.normal), rb));

                float j = numerator/(t1+t2+t3+t4);
                glm::vec3 normalForce = j*info.normal/(dt*info.points.size());
                glm::vec3 tangentialForce = (glm::length(vt)/glm::length(vn))*glm::abs(j)*glm::normalize(vt)/(dt*info.points.size());
                // Utilities::PrintVec3(tangentialForce);
                float angularRel = glm::length(dynamicCube->rb->angularVelocity-staticCube->rb->angularVelocity);

                dynamicCube->rb->addTorque(20.0f/(dynamicCube->xSize+dynamicCube->ySize+dynamicCube->zSize)*glm::cross(dynamicCube->rb->mass*gravity, ra));
                dynamicCube->rb->addTorque(glm::cross(ra, normalForce));
                //add the fritional torque

                glm::vec3 rotationPoint = info.points[i];
                glm::vec3 radius = dynamicCube->rb->position-rotationPoint;



                glm::vec3 velocityFromAngular = glm::cross(dynamicCube->rb->angularVelocity, radius);
                velocityFromAngular -= glm::dot(velocityFromAngular, norm)*norm;

                glm::vec3 torsion = glm::dot(dynamicCube->rb->mass*gravity, norm)*glm::dot(dynamicCube->rb->angularVelocity, norm)*norm;
                dynamicCube->rb->addTorque(-torsion);

                if(glm::length(vt)<0.1f)
                    vt = glm::vec3(0,0,0);
                else
                {
                    vt -= 20.0f*glm::dot(dynamicCube->rb->mass*gravity*friction, norm)*glm::normalize(vt)*dt;
                }
                totalVelocity+=velocityFromAngular+vt;
            }
            totalVelocity/=(float)info.points.size();

            dynamicCube->rb->setVelocity(totalVelocity);
        }
    }

}

void PhysicsWorld::cubeSphereCollisionResponseDynamicVsDynamic(ContactInfo& info, float dt, CubeCollider* cube, SphereCollider* sphere)
{
    // qDebug()<<"dynamic response:";

    //cube->rb->position += info.normal*info.penetrationDistance;
    if(info.faceCollision)
        sphere->rb->position -= info.normal*info.penetrationDistance;

    float epsilon = 0.0f;
    glm::vec3 ra = info.points[0]-cube->rb->position;
    glm::vec3 rb = info.points[0]-sphere->rb->position;
    glm::vec3 va = cube->rb->velocity + glm::cross(cube->rb->angularVelocity, ra);
    glm::vec3 vb = sphere->rb->velocity + glm::cross(sphere->rb->angularVelocity, rb);
    float vRel = glm::dot(info.normal, va-vb);

    float numerator = -(1-epsilon)*vRel;

    float t1 = cube->rb->massInv;
    float t2 = sphere->rb->massInv;
    float t3 = glm::dot(info.normal, glm::cross(glm::cross(cube->rb->inertiaInv*ra, info.normal), ra));
    float t4 = glm::dot(info.normal, glm::cross(glm::cross(sphere->rb->inertiaInv*rb, info.normal), rb));

    float j = numerator/(t1+t2+t3+t4);
    glm::vec3 force = j*info.normal/dt;
    //qDebug() << j;
    float angularRel = glm::length(cube->rb->angularVelocity-sphere->rb->angularVelocity);


    cube->rb->addForce(force, *cube->rb);
    cube->rb->addTorque(glm::cross(ra, force));

    sphere->rb->addForce(-force, *sphere->rb);
    sphere->rb->addTorque(glm::cross(rb, -force));

}

void PhysicsWorld::cubeSphereCollisionResponseStaticVsDynamic(ContactInfo& info, float dt, CubeCollider* cube, SphereCollider* sphere)
{
    glm::vec3 normal = info.normal;
    if(info.faceCollision)
        sphere->rb->position -= normal*info.penetrationDistance;
    glm::vec3 r = info.points[0]-sphere->rb->position;

    glm::vec3 vn = glm::dot(normal, sphere->rb->velocity)*sphere->rb->velocity;
    glm::vec3 vt = sphere->rb->velocity - vn;

    glm::vec3 force = -1.1f*normal*glm::dot(sphere->rb->velocity, normal)*sphere->rb->mass/dt;
    float j = glm::length(force);
    float sMax = 5.0f*restitutionSlope*glm::abs(glm::dot(gravity, normal))+restitutionIntersect;
    float slope = glm::dot(info.normal, glm::vec3(0,1,0));

    if(j>sMax || slope<0.0f)
    {

        sphere->rb->addForce(force);
        glm::vec3 forceT = glm::normalize(vt)*glm::length(vt)/glm::length(vn);
        sphere->rb->setAngularVelocity(glm::cross(normal,vt/sphere->radius));
    }
    else
    {
        if(info.faceCollision)
        {
            sphere->rb->linearMomentum = glm::cross(glm::cross(normal, sphere->rb->linearMomentum), normal);
            sphere->rb->setAngularVelocity(glm::cross(normal,sphere->rb->velocity/sphere->radius));
            sphere->rb->addForce(friction*glm::normalize(sphere->rb->velocity)*glm::dot(gravity, normal));
        }
        else
        {

            if(glm::dot(force, normal)>0)
            {
                sphere->rb->addForce(force);
            }
        }
    }

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

bool PhysicsWorld::raycastAll(const glm::vec3& start, const glm::vec3& dir, RayCastData& dat)
{
    float minDist = std::numeric_limits<float>().max();
    RayCastData tempData;
    bool didHit = false;
    for(Collider* collider: colliders)
    {
        switch(collider->type)
        {
        case ColliderType::CUBE:
        {
            CubeCollider* cube = dynamic_cast<CubeCollider*>(collider);
            if(cubeRaycast(start, dir, tempData, cube))
            {
                didHit = true;
                if(tempData.length<minDist)
                {
                    dat = tempData;
                    minDist = tempData.length;
                }
            }
        }
            break;

        case ColliderType::SPHERE:
        {
            SphereCollider* sphere = dynamic_cast<SphereCollider*>(collider);
            if(sphereRaycast(start, dir, tempData, sphere))
            {
                didHit = true;
                if(tempData.length<minDist)
                {
                    dat = tempData;
                    minDist = tempData.length;
                }
            }
        }
            break;
        }
    }

    return didHit;
}
bool PhysicsWorld::raycastAll(const glm::vec3& start, const glm::vec3& dir, RayCastData& dat, int mask)
{
    return false;
}
bool PhysicsWorld::raycastAll(const glm::vec3& start, const glm::vec3& dir, RayCastData& dat, ColliderType type)
{
    float minDist = std::numeric_limits<float>().max();
    RayCastData tempData;
    bool didHit = false;
    for(Collider* collider: colliders)
    {
        if(collider->type == type)
        {
            switch(type)
            {
            case ColliderType::CUBE:
            {
                CubeCollider* cube = dynamic_cast<CubeCollider*>(collider);
                if(cubeRaycast(start, dir, tempData, cube))
                {
                    didHit = true;
                    if(tempData.length<minDist)
                    {
                        dat = tempData;
                        minDist = tempData.length;
                    }
                }
            }
                break;

            case ColliderType::SPHERE:
            {
                SphereCollider* sphere = dynamic_cast<SphereCollider*>(collider);
                if(sphereRaycast(start, dir, tempData, sphere))
                {
                    didHit = true;
                    if(tempData.length<minDist)
                    {
                        dat = tempData;
                        minDist = tempData.length;
                    }
                }
            }
                break;
            }
        }
    }

    return didHit;
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
    checkForCollisions(dt);
    updateQuantities(dt);
}

void PhysicsWorld::stepWorld(float dt, int inc)
{
    float adjustedDt = dt/inc;
    for(int i = 0; i<inc;i++)
    {
        checkForCollisions(adjustedDt);
        updateQuantities(adjustedDt);
    }
}

bool PhysicsWorld::cubeFlatOnSurface(CubeCollider* cube, glm::vec3& normal, float tolerance)
{
    return true;
}
