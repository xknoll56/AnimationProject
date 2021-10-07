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

SphereCollider::SphereCollider(const float radius)
{
    this->radius = radius;
    type = ColliderType::SPHERE;
}

SphereCollider::SphereCollider(const float radius, UniformRigidBody* const rb)
{
    this->radius = radius;
    type = ColliderType::SPHERE;
    this->rb = rb;
}

CubeCollider::CubeCollider(const glm::vec3& sizes)
{
    xSize = sizes.x;
    ySize = sizes.y;
    zSize = sizes.z;
    scale = glm::vec3(2*xSize, 2*ySize, 2*zSize);
    type = ColliderType::CUBE;
    initEdges();
}

CubeCollider::CubeCollider()
{
    xSize = 0.5f;
    ySize = 0.5f;
    zSize = 0.5f;
    scale = glm::vec3(2*xSize, 2*ySize, 2*zSize);
    type = ColliderType::CUBE;
    initEdges();
}
CubeCollider::CubeCollider(const CubeCollider& other)
{
    xSize = other.xSize;
    ySize = other.ySize;
    zSize = other.zSize;
    scale = glm::vec3(2*xSize, 2*ySize, 2*zSize);
    type = ColliderType::CUBE;
    initEdges();
}

CubeCollider& CubeCollider::operator= (const CubeCollider& other)
{
    xSize = other.xSize;
    ySize = other.ySize;
    zSize = other.zSize;
    scale = glm::vec3(2*xSize, 2*ySize, 2*zSize);
    type = ColliderType::CUBE;
    initEdges();
    return *this;
}

CubeCollider::CubeCollider(const glm::vec3& sizes, UniformRigidBody* const rb)
{
    xSize = sizes.x;
    ySize = sizes.y;
    zSize = sizes.z;
    this->rb = rb;
    type = ColliderType::CUBE;
    scale = glm::vec3(2*xSize, 2*ySize, 2*zSize);
    initEdges();

}

void CubeCollider::initEdges()
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

void CubeCollider::updateContactVerts()
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



std::vector<glm::vec3> CubeCollider::getClosestVerts(const glm::vec3& dir)
{
    updateContactVerts();
    float min = std::numeric_limits<float>::max();
    //glm::vec3 minVert = contactVertBuffer[0];
    std::vector<glm::vec3> minVerts;
    indices.clear();
    minVerts.reserve(4);
    indices.reserve(4);
    for(int i = 0; i<8;i++)
    {
        float test = glm::dot(dir, contactVertBuffer[i]);
        if(test<min)
        {
            min = test;
        }
    }
    for(int i = 0; i<8;i++)
    {
        float test = glm::dot(dir, contactVertBuffer[i]);
        if(glm::epsilonEqual(test, min, 0.0001f))
        {
            minVerts.push_back(contactVertBuffer[i]);
            indices.push_back(i);
        }
    }
    return minVerts;
}

std::vector<CubeCollider::EdgeIndices> CubeCollider::getEdgesFromVertexIndices()
{
    std::vector<CubeCollider::EdgeIndices> eis;
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
    return eis;
}

QuadCollider::QuadCollider(const float xSize, const float zSize)
{
    this->xSize = xSize;
    this->zSize = zSize;
    type = ColliderType::QUAD;
}
