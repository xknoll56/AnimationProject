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
    contacts.clear();
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
                        if(!contactHandled(cube, otherCube))
                        {
                            ContactInfo info;
                            if(detectCubeCubeCollision(dt, cube, otherCube, info))
                                determineCubeCubeContactPoints(info, cube, otherCube);
                        }
                        break;
                    }
                }
            }
            break;
        }
        }
    }

    if(enableResponse)
    {
        for(ContactInfo& info: contacts)
        {
            CubeCollider* cube = dynamic_cast<CubeCollider*>(info.a);
            CubeCollider* otherCube = dynamic_cast<CubeCollider*>(info.b);
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

bool PhysicsWorld::closestPointsDoIntersect(glm::vec3& p0,  glm::vec3& p1, const glm::vec3& u, const glm::vec3& v, float s0, float s1)
{
    glm::vec3 uv = glm::cross(v,u);
    float uvSquared = glm::length2(uv);
    float t = -glm::dot(glm::cross(p1-p0, u), uv)/uvSquared;
    float s = -glm::dot(glm::cross(p1-p0, v), uv)/uvSquared;
    if(glm::abs(t)>s1 || glm::abs(s)>s0  || std::isnan(t) || std::isnan(s))
        return false;

    return true;
}



bool PhysicsWorld::detectCubeCubeCollision(float dt, CubeCollider* cubeA, CubeCollider* cubeB, ContactInfo& contactInfo)
{
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
    if(penetration > faceInfo.penetrationDistance)
    {
        faceInfo.penetrationDistance = penetration;
        faceInfo.normal = bX;
        faceInfo.faceCollision = true;
        faceInfo.aDir = CubeCollider::ContactDir::NONE;
        faceInfo.bDir = CubeCollider::ContactDir::RIGHT;
    }
    else if(glm::epsilonEqual(penetration, faceInfo.penetrationDistance, 0.00001f))
    {
        faceInfo.penetrationDistance = penetration;
        faceInfo.normal = bX;
        faceInfo.faceCollision = true;
        faceToFace = true;
        faceInfo.bDir = CubeCollider::ContactDir::RIGHT;
    }
    t1 += penetration;

    penetration = glm::abs(glm::dot(T, bY)) - (glm::abs(cubeA->xSize*rxy) + glm::abs(cubeA->ySize*ryy) + glm::abs(cubeA->zSize*rzy) + cubeB->ySize);
    //check for collisions parallel to BY
    if(penetration>0)
    {
        return false;
    }
    if(penetration > faceInfo.penetrationDistance)
    {
        faceInfo.penetrationDistance = penetration;
        faceInfo.normal = bY;
        faceInfo.faceCollision = true;
        faceInfo.aDir = CubeCollider::ContactDir::NONE;
        faceInfo.bDir = CubeCollider::ContactDir::UP;
    }
    else if(glm::epsilonEqual(penetration, faceInfo.penetrationDistance, 0.00001f))
    {
        faceInfo.penetrationDistance = penetration;
        faceInfo.normal = bY;
        faceInfo.faceCollision = true;
        faceToFace = true;
        faceInfo.bDir = CubeCollider::ContactDir::UP;
    }
    t1 += penetration;

    penetration = glm::abs(glm::dot(T, bZ)) - (glm::abs(cubeA->xSize*rxz) + glm::abs(cubeA->ySize*ryz) + glm::abs(cubeA->zSize*rzz) + cubeB->zSize);
    //check for collisions parallel to BZ
    if(penetration>0)
    {
        return false;
    }
    if(penetration > faceInfo.penetrationDistance)
    {
        faceInfo.penetrationDistance = penetration;
        faceInfo.normal = bZ;
        faceInfo.faceCollision = true;
        faceInfo.aDir = CubeCollider::ContactDir::NONE;
        faceInfo.bDir = CubeCollider::ContactDir::FORWARD;
    }
    else if(glm::epsilonEqual(penetration, faceInfo.penetrationDistance, 0.00001f))
    {
        faceInfo.penetrationDistance = penetration;
        faceInfo.normal = bZ;
        faceInfo.faceCollision = true;
        faceToFace = true;
        faceInfo.bDir = CubeCollider::ContactDir::FORWARD;
    }
    t1 += penetration;

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

    //    qDebug() << "t1: " << t1;
    //    qDebug() << "t2: " << t2;
    //        qDebug() << "face penetration: " << faceInfo.penetrationDistance;
    //        qDebug() << "edge pentration: " << edgeInfo.penetrationDistance;

    faceInfo.normal = glm::sign(glm::dot(T, faceInfo.normal))*faceInfo.normal;
    faceInfo.normal = glm::normalize(faceInfo.normal);
    std::vector<glm::vec3> closestVertsA = cubeA->getClosestVerts(-faceInfo.normal);
    std::vector<glm::vec3> closestVertsB = cubeB->getClosestVerts(faceInfo.normal);
    bool facecollision = false;
    if(faceInfo.aDir == CubeCollider::ContactDir::NONE)
    {
        facecollision = isCubeCubePetrusion(-faceInfo.normal, closestVertsA, cubeB, faceInfo.bDir);
    }
    else if(faceInfo.bDir == CubeCollider::ContactDir::NONE)
    {
        facecollision = isCubeCubePetrusion(faceInfo.normal, closestVertsB, cubeA, faceInfo.aDir);
    }
    else if(faceToFace)
    {
//        facecollision = isCubeCubePetrusion(-faceInfo.normal, closestVertsA, cubeB, faceInfo.bDir)||
//                isCubeCubePetrusion(faceInfo.normal, closestVertsB, cubeA, faceInfo.aDir);
        facecollision = true;

    }
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
    for(CubeCollider::EdgeIndices ea: aEdges)
    {
        for(CubeCollider::EdgeIndices eb: bEdges)
        {
            if(closestPointsDoIntersect(ea.midPoint, eb.midPoint, ea.dir, eb.dir, ea.length, eb.length))
            {
                //testing petrusion correciton
                //info.penetrationDistance = -closestDistanceBetweenLines(ea.midPoint, eb.midPoint, ea.dir, eb.dir, ea.length, eb.length);
                float dist = glm::abs(closestDistanceBetweenLines(ea.midPoint, eb.midPoint, ea.dir, eb.dir, ea.length, eb.length));
                if(dist<minDist && !info.faceCollision)
                {
                    qDebug() << "ea dir dot eb dir";
                    qDebug() << glm::dot(ea.dir, eb.dir);
                    minDist = dist;
                    info.normal = glm::normalize(glm::cross(ea.dir, eb.dir));
                    info.normal = glm::sign(glm::dot(T, info.normal))*info.normal;
                    info.penetrationDistance = -dist;
                    qDebug() << "ea dir: ";
                    Utilities::PrintVec3(ea.dir);
                    qDebug() <<"eb dir: ";
                    Utilities::PrintVec3(eb.dir);
                    qDebug() <<"normal dir: ";
                    Utilities::PrintVec3(info.normal);
                }
                info.points.push_back(closestPointBetweenLines(ea.midPoint, eb.midPoint, ea.dir, eb.dir));
//                if(!info.faceCollision)
//                {
//                    info.normal = glm::normalize(glm::cross(ea.dir, eb.dir));
//                    info.normal = glm::sign(glm::dot(T, info.normal))*info.normal;
//                    Utilities::PrintVec3(info.normal);
//                }
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
        if(d>=0.0f)
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
        if(adjustPenetration && d>=0.0f)
            info.penetrationDistance = -d;
<<<<<<< Updated upstream
        glm::vec3 intersectionPoint = point+d*normal;
        float dist1 = glm::abs(glm::dot(intersectionPoint-p0, adj1));
        float dist2 = glm::abs(glm::dot(intersectionPoint-p0, adj2));
        if(dist1<=maxDist1 && dist2<=maxDist2)
            info.points.push_back(intersectionPoint);
=======
        if(d>=0.0f)
        {
            glm::vec3 intersectionPoint = point+d*normal;
            float dist1 = glm::abs(glm::dot(intersectionPoint-p0, adj1));
            float dist2 = glm::abs(glm::dot(intersectionPoint-p0, adj2));
            if(dist1<=maxDist1 && dist2<=maxDist2)
            {
                info.points.push_back(intersectionPoint);
                info.vertexPoints++;
            }
        }
>>>>>>> Stashed changes
    }
}

void PhysicsWorld::cubeCubeCollisionResponse(ContactInfo& info, float dt, CubeCollider* cubeA, CubeCollider* cubeB)
{

    cubeA->rb->position += 0.5f*info.normal*info.penetrationDistance;
    cubeB->rb->position -= 0.5f*info.normal*info.penetrationDistance;
//    if(info.aDir == CubeCollider::ContactDir::NONE && info.faceToFaceCollision)
//        cubeB->rb->position -= info.normal*info.penetrationDistance;
//    else if(info.bDir == CubeCollider::ContactDir::NONE && info.faceToFaceCollision)
//        cubeA->rb->position += info.normal*info.penetrationDistance;
//    else
//    {
//        cubeA->rb->position += 0.5f*info.normal*info.penetrationDistance;
//        cubeB->rb->position -= 0.5f*info.normal*info.penetrationDistance;
//    }


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
        //qDebug() << "cube b vertical velocity: " << cubeB->rb->velocity.y;
        //qDebug() << "cube a vertical velocity: " << cubeA->rb->velocity.y;
        //qDebug() << vRel;
        //qDebug() << "angular velocity: " << angularRel;

        //            if(glm::abs(vRel)>0.2f )
        //            {

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


void PhysicsWorld::cubeCubeCollisionResponseDynamicVsStatic(ContactInfo& info, const glm::vec3& norm, float dt, CubeCollider* dynamicCube, CubeCollider* staticCube)
{

    dynamicCube->rb->position += norm*info.penetrationDistance;

    if(!dynamicCube->rb->stabilizing)
    {
       // qDebug() << "not stabalizing";
        if(info.points.size()==0)
            dynamicCube->rb->stabilizing = true;
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

            float j = numerator/(t1+t3);
            glm::vec3 force = j*info.normal/(dt*info.points.size());
            float angularRel = glm::length(dynamicCube->rb->angularVelocity-staticCube->rb->angularVelocity);
            //qDebug() << "j: " << j;

//            if(glm::abs(vRel)<2.0f )
//                dynamicCube->rb->stabilizing = true;
            if(glm::abs(vRel)<1.0f )
                dynamicCube->rb->stabilizing = true;

            glm::vec3 perpVelNormal = glm::normalize(dynamicCube->rb->velocity + va*norm);
            if(!glm::isnan(perpVelNormal.x) && !glm::isnan(perpVelNormal.y) && !glm::isnan(perpVelNormal.z)){
                glm::vec3 fric = friction*perpVelNormal*dynamicCube->rb->mass*glm::dot(gravity, norm);
                force-=fric;
            }
            dynamicCube->rb->addForce(force, *dynamicCube->rb);
            dynamicCube->rb->addTorque(glm::cross(ra, force));
        }
    }
    else
    {
        //qDebug() <<"in resting state";

        float angularLen = glm::length(dynamicCube->rb->angularVelocity);

        if(info.points.size()>0)
        {
           // if(info.points.size()>2)
               // qDebug() <<"more then 2: " << info.points.size();
            //qDebug() << "angular speed: " << angularLen;
            glm::vec3 ra = info.points[0]-dynamicCube->rb->position;
            glm::vec3 rotPoint= info.points[0];
            if(info.points.size() == 2)
            {
                rotPoint = 0.5f*(info.points[0]+info.points[1]);
                glm::vec3 ra = rotPoint-dynamicCube->rb->position;
            }
            else if(info.points.size() == 4)
            {
                rotPoint = 0.5f*(info.points[0]+info.points[1] + info.points[2]+info.points[3]);
                glm::vec3 ra = rotPoint-dynamicCube->rb->position;
              //  Utilities::PrintVec3(ra);
            }
            if(info.points.size()>2 && info.faceToFaceCollision)
            {
                dynamicCube->rb->atRest = true;
                dynamicCube->rb->stabilizing = false;
            }
            glm::vec3 perpVel = glm::cross(dynamicCube->rb->angularVelocity, -ra);
            glm::vec3 normVel = glm::dot(perpVel, norm)*norm;
            perpVel -= normVel;
            //Utilities::PrintVec3(perpVel);

            glm::vec3 frictionForce = glm::normalize(perpVel)*glm::dot(friction*gravity, norm);
            dynamicCube->rb->addTorque(5.0f*glm::cross(dynamicCube->rb->mass*gravity, 2.0f*ra));



             if(!glm::isnan(frictionForce.x) && !glm::isnan(frictionForce.y) && !glm::isnan(frictionForce.z))
             {
                 perpVel-=frictionForce*dt;
                 dynamicCube->rb->addTorque(5.0f*glm::cross(frictionForce, -ra));
             }
            dynamicCube->rb->setVelocity(perpVel);
        }
//        else if(glm::abs(glm::dot(info.normal, glm::vec3(0,1,0)))>0.90f)
//            dynamicCube->rb->atRest = true;

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
    checkForCollisions(dt);
    updateQuantities(dt);
}
