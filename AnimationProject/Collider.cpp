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

void SphereCollider::setAABB()
{
    aabb.xSize = aabb.ySize = aabb.zSize = radius;
}

SphereCollider::SphereCollider()
{
    this->radius = 0.5f;
    type = ColliderType::SPHERE;
    scale = 2.0f*glm::vec3(radius, radius, radius);
    setAABB();
}

SphereCollider::SphereCollider(const float radius)
{
    this->radius = radius;
    type = ColliderType::SPHERE;
    scale = 2.0f*glm::vec3(radius, radius, radius);
    setAABB();
}

SphereCollider::SphereCollider(const float radius, UniformRigidBody* const rb)
{
    this->radius = radius;
    type = ColliderType::SPHERE;
    this->rb = rb;
    scale = 2.0f*glm::vec3(radius, radius, radius);
    setAABB();
}

SphereCollider& SphereCollider::operator= (const SphereCollider& other)
{
    this->radius = other.radius;
    this->type = ColliderType::SPHERE;
    scale = 2.0f*glm::vec3(radius, radius, radius);
    return *this;
}

SphereCollider::SphereCollider(const SphereCollider& other): Collider(other)
{
    this->radius = other.radius;
    this->type = ColliderType::SPHERE;
    scale = 2.0f*glm::vec3(radius, radius, radius);
    this->rb = other.rb;
}

void BoxCollider::setAABB()
{
    aabb.xSize = aabb.ySize = aabb.zSize = glm::sqrt(xSize*xSize+ySize*ySize+zSize*zSize);
}

BoxCollider::BoxCollider(const glm::vec3& sizes)
{
    xSize = sizes.x;
    ySize = sizes.y;
    zSize = sizes.z;
    scale = glm::vec3(2*xSize, 2*ySize, 2*zSize);
    type = ColliderType::CUBE;
    initEdges();
    setAABB();
}

BoxCollider::BoxCollider()
{
    xSize = 0.5f;
    ySize = 0.5f;
    zSize = 0.5f;
    scale = glm::vec3(2*xSize, 2*ySize, 2*zSize);
    type = ColliderType::CUBE;
    initEdges();
    setAABB();
}
BoxCollider::BoxCollider(const BoxCollider& other): Collider(other)
{
    xSize = other.xSize;
    ySize = other.ySize;
    zSize = other.zSize;
    scale = glm::vec3(2*xSize, 2*ySize, 2*zSize);
    type = ColliderType::CUBE;
    initEdges();
    setAABB();
}

BoxCollider& BoxCollider::operator= (const BoxCollider& other)
{
    xSize = other.xSize;
    ySize = other.ySize;
    zSize = other.zSize;
    scale = glm::vec3(2*xSize, 2*ySize, 2*zSize);
    type = ColliderType::CUBE;
    initEdges();
    this->aabb = other.aabb;
    return *this;
}

void BoxCollider::initEdges()
{
    edges[0] = {0,1, zSize};
    edges[1] = {1,2, xSize};
    edges[2] = {2,3, zSize};
    edges[3] = {3,0, xSize};

    edges[4] = {0,4, ySize};
    edges[5] = {1,5, ySize};
    edges[6] = {2,6, ySize};
    edges[7] = {3,7, ySize};

    edges[8] = {4,5, zSize};
    edges[9] = {5,6, xSize};
    edges[10] = {6,7, zSize};
    edges[11] = {7,4, xSize};
}

void BoxCollider::updateContactVerts()
{
    glm::vec3 px = xSize*rb->getLocalXAxis();
    glm::vec3 py = ySize*rb->getLocalYAxis();
    glm::vec3 pz = zSize*rb->getLocalZAxis();
    contactVertBuffer[0] = rb->position + -px-py-pz;
    contactVertBuffer[1] = rb->position + -px-py+pz;
    contactVertBuffer[2] = rb->position + +px-py+pz;
    contactVertBuffer[3] = rb->position + +px-py-pz;
    contactVertBuffer[4] = rb->position + -px+py-pz;
    contactVertBuffer[5] = rb->position + -px+py+pz;
    contactVertBuffer[6] = rb->position + +px+py+pz;
    contactVertBuffer[7] = rb->position + +px+py-pz;
}



std::vector<glm::vec3> BoxCollider::getClosestVerts(const glm::vec3& dir)
{
    //todo return all of the edges in the face of the closest dir
    updateContactVerts();
    float min = std::numeric_limits<float>::max();
    //glm::vec3 minVert = contactVertBuffer[0];
    std::vector<glm::vec3> minVerts;
    indices.clear();
    minVerts.reserve(4);
    indices.reserve(4);
    float magX = glm::dot(dir, xSize*rb->getLocalXAxis());
    float magY = glm::dot(dir, ySize*rb->getLocalYAxis());
    float magZ = glm::dot(dir, zSize*rb->getLocalZAxis());


    float max = magX;
    ContactDir cd = ContactDir::RIGHT;
    if(glm::abs(magY)>glm::abs(max))
    {
        cd =ContactDir::UP;
        max = magY;
    }
    if(glm::abs(magZ)>glm::abs(max))
    {
        cd = ContactDir::FORWARD;
        max = magZ;
    }
    switch(cd)
    {
    case ContactDir::RIGHT:
        if(max<0)
        {
            minVerts.push_back(contactVertBuffer[2]);
            minVerts.push_back(contactVertBuffer[3]);
            minVerts.push_back(contactVertBuffer[6]);
            minVerts.push_back(contactVertBuffer[7]);

            indices.push_back(2);
            indices.push_back(3);
            indices.push_back(6);
            indices.push_back(7);
        }
        else
        {
            minVerts.push_back(contactVertBuffer[0]);
            minVerts.push_back(contactVertBuffer[1]);
            minVerts.push_back(contactVertBuffer[4]);
            minVerts.push_back(contactVertBuffer[5]);

            indices.push_back(0);
            indices.push_back(1);
            indices.push_back(4);
            indices.push_back(5);
        }
        break;
    case ContactDir::UP:
        if(max<0)
        {
            minVerts.push_back(contactVertBuffer[4]);
            minVerts.push_back(contactVertBuffer[5]);
            minVerts.push_back(contactVertBuffer[6]);
            minVerts.push_back(contactVertBuffer[7]);

            indices.push_back(4);
            indices.push_back(5);
            indices.push_back(6);
            indices.push_back(7);
        }
        else
        {
            minVerts.push_back(contactVertBuffer[0]);
            minVerts.push_back(contactVertBuffer[1]);
            minVerts.push_back(contactVertBuffer[2]);
            minVerts.push_back(contactVertBuffer[3]);

            indices.push_back(0);
            indices.push_back(1);
            indices.push_back(2);
            indices.push_back(3);
        }
        break;
    case ContactDir::FORWARD:
        if(max<0)
        {
            minVerts.push_back(contactVertBuffer[1]);
            minVerts.push_back(contactVertBuffer[2]);
            minVerts.push_back(contactVertBuffer[5]);
            minVerts.push_back(contactVertBuffer[6]);

            indices.push_back(1);
            indices.push_back(2);
            indices.push_back(5);
            indices.push_back(6);
        }
        else
        {
            minVerts.push_back(contactVertBuffer[0]);
            minVerts.push_back(contactVertBuffer[7]);
            minVerts.push_back(contactVertBuffer[3]);
            minVerts.push_back(contactVertBuffer[4]);

            indices.push_back(0);
            indices.push_back(7);
            indices.push_back(3);
            indices.push_back(4);
        }
        break;
    }
    return minVerts;
}

std::vector<BoxCollider::EdgeIndices> BoxCollider::getClosestEdges(const glm::vec3& dir)
{
    std::vector<BoxCollider::EdgeIndices> eis = getEdgesFromVertexIndices();

    return eis;
}

//TODO- Use a hash table here
std::vector<BoxCollider::EdgeIndices> BoxCollider::getEdgesFromVertexIndices()
{
    std::vector<BoxCollider::EdgeIndices> eis;
    for(int i0: indices)
    {
        for(int i1: indices)
        {
            if(i0!=i1)
            {
                for(int j = 0;j<12;j++)
                {
                    if((edges[j].i0 == i0 && edges[j].i1 == i1) ||(edges[j].i0 == i1 && edges[j].i1 == i0))
                    {
                        bool found = false;
                        for(int i =0;i<eis.size();i++)
                        {
                            if((eis[i].i0 == i0 && eis[i].i1 == i1) ||(eis[i].i0 == i1 && eis[i].i1==i0))
                            {
                                //already contained in edges
                                found = true;
                                break;
                            }
                        }
                        if(found)
                            break;

                        edges[j].midPoint = 0.5f*(contactVertBuffer[edges[j].i0]+contactVertBuffer[edges[j].i1]);
                        if(j>3 && j<8)
                            edges[j].dir = rb->getLocalYAxis();
                        else
                        {
                            if(j%2==0)
                                edges[j].dir = rb->getLocalZAxis();
                            else
                                edges[j].dir = rb->getLocalXAxis();
                        }
                        eis.push_back(edges[j]);
                    }
                }
            }
        }
    }

    for(BoxCollider::EdgeIndices& edge: eis)
    {
        for(BoxCollider::EdgeIndices& otherEdge: eis)
        {
            if(glm::dot(edge.dir, otherEdge.dir)<0.5f)
            {
                edge.normalLength = otherEdge.length;
                break;
            }
        }
    }
    return eis;
}

BoxCollider::ContactDir BoxCollider::GetFaceClosestToNormal(const glm::vec3& normal, bool& negative, glm::vec3& faceDir)
{
     BoxCollider::ContactDir dir = BoxCollider::ContactDir::RIGHT;
     faceDir = rb->getLocalXAxis();
     negative = false;
     float dirMax = glm::dot(normal, rb->getLocalXAxis());
     if(dirMax<0)
     {
         dirMax = -dirMax;
         negative = true;
     }

     float dirTest = glm::dot(normal, rb->getLocalYAxis());
     bool negTest = dirTest<0;
     if(negTest)
         dirTest = -dirTest;

     if(dirTest>dirMax)
     {
        dir = BoxCollider::ContactDir::UP;
        dirMax = dirTest;
        negative = negTest;
        faceDir = rb->getLocalYAxis();
     }

     dirTest = glm::dot(normal, rb->getLocalZAxis());
     negTest = dirTest<0;
     if(negTest)
         dirTest = -dirTest;

     if(dirTest>dirMax)
     {
        dir = BoxCollider::ContactDir::FORWARD;
        dirMax = dirTest;
        negative = negTest;
        faceDir = rb->getLocalZAxis();
     }

     return dir;
}

glm::quat BoxCollider::toRotation(BoxCollider::ContactDir upMostDir, const glm::vec3& up)
{
    glm::mat3 rotationMat;

    switch(upMostDir)
    {
    case BoxCollider::ContactDir::RIGHT:
    {
        rotationMat[0] = up;
        glm::vec3 y = rb->getLocalYAxis()-glm::dot(rb->getLocalYAxis(), up)*up;
        y = glm::normalize(y);
        glm::vec3 z = glm::cross(up, y);
        rotationMat[1] = y;
        rotationMat[2] = z;
    }
        break;
    case BoxCollider::ContactDir::UP:
    {
        rotationMat[1] = up;
        glm::vec3 z = rb->getLocalZAxis()-glm::dot(rb->getLocalZAxis(), up)*up;
        z = glm::normalize(z);
        glm::vec3 x = glm::cross(up,z);
        rotationMat[2] = z;
        rotationMat[0] = x;
    }
        break;
    case BoxCollider::ContactDir::FORWARD:
    {
        rotationMat[2] = up;
        glm::vec3 x = rb->getLocalXAxis()-glm::dot(rb->getLocalXAxis(), up)*up;
        x = glm::normalize(x);
        glm::vec3 y = glm::cross(up, x);
        rotationMat[0] = x;
        rotationMat[1] = y;
    }
        break;
    }

    return glm::toQuat(rotationMat);
}

QuadCollider::QuadCollider(const float xSize, const float zSize)
{
    this->xSize = xSize;
    this->zSize = zSize;
    type = ColliderType::QUAD;
}
