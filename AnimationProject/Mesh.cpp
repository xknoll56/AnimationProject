#include "Mesh.h"
#include <glm/gtc/constants.hpp>
extern QOpenGLFunctions_4_5_Core* openglFunctions;
#define PI 3.14159265359f

static PrimitiveBufferData cubePBD;
static PrimitiveBufferData cubeBoundsPBD;
static PrimitiveBufferData cylinderPBD;
static PrimitiveBufferData cylinderBoundsPBD;
static PrimitiveBufferData spherePBD;
static PrimitiveBufferData sphereBoundsPBD;
static PrimitiveBufferData planePBD;
static PrimitiveBufferData planeBoundsPBD;
static PrimitiveBufferData conePBD;
static PrimitiveBufferData coneBoundsPBD;

//this constructor will contain the vertices packed in with the normals
Mesh::Mesh(std::vector<glm::vec3> verts, GLenum type)
{
    this->type = type;
    openglFunctions->glGenVertexArrays(1, &vao);
    openglFunctions->glGenBuffers(1, &vbo);
    // bind the Vertex Array Object first, then bind and set vertex buffer(s), and then configure vertex attributes(s).
    openglFunctions->glBindVertexArray(vao);

    if(type == GL_TRIANGLES)
    {
        openglFunctions->glBindBuffer(GL_ARRAY_BUFFER, vbo);
        openglFunctions->glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3)*verts.size(), &verts[0], GL_STATIC_DRAW);

        openglFunctions->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 2 * sizeof(glm::vec3), (void*)0);
        openglFunctions->glEnableVertexAttribArray(0);

        openglFunctions->glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 2 * sizeof(glm::vec3), (void*)(sizeof(glm::vec3)));
        openglFunctions->glEnableVertexAttribArray(1);
        numVerts = verts.size()/2;
        color = glm::vec3(1,1,1);
    }
    else
    {
        openglFunctions->glBindBuffer(GL_ARRAY_BUFFER, vbo);
        openglFunctions->glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3)*verts.size(), &verts[0], GL_STATIC_DRAW);

        openglFunctions->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
        openglFunctions->glEnableVertexAttribArray(0);

        numVerts = verts.size();
        color = glm::vec3(1,1,1);
    }
}

Mesh::Mesh(std::vector<float> verts, GLenum type)
{
    this->type = type;
    openglFunctions->glGenVertexArrays(1, &vao);
    openglFunctions->glGenBuffers(1, &vbo);
    // bind the Vertex Array Object first, then bind and set vertex buffer(s), and then configure vertex attributes(s).
    openglFunctions->glBindVertexArray(vao);

    if(type == GL_TRIANGLES)
    {
        openglFunctions->glBindBuffer(GL_ARRAY_BUFFER, vbo);
        openglFunctions->glBufferData(GL_ARRAY_BUFFER, sizeof(float)*verts.size(), &verts[0], GL_STATIC_DRAW);

        openglFunctions->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
        openglFunctions->glEnableVertexAttribArray(0);

        openglFunctions->glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3*sizeof(float)));
        openglFunctions->glEnableVertexAttribArray(1);
        numVerts = verts.size()/6;
        color = glm::vec3(1,1,1);
    }
    else
    {
        openglFunctions->glBindBuffer(GL_ARRAY_BUFFER, vbo);
        openglFunctions->glBufferData(GL_ARRAY_BUFFER, sizeof(float)*verts.size(), &verts[0], GL_STATIC_DRAW);

        openglFunctions->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        openglFunctions->glEnableVertexAttribArray(0);
        numVerts = verts.size()/3;
        color = glm::vec3(1,1,1);
    }


}

Mesh::Mesh(const Mesh& mesh)
{
    vao = mesh.vao;
    vbo = mesh.vbo;
    nbo = mesh.nbo;
    ebo = mesh.ebo;
    numVerts = mesh.numVerts;
    color = mesh.color;
    type = mesh.type;
}

Mesh Mesh::createCube()
{
    return Mesh(cubePBD);
}

Mesh Mesh::createPlane()
{
    return Mesh(planePBD);
}

Mesh Mesh::createBoundingBox()
{
    return Mesh(cubeBoundsPBD);
}

Mesh Mesh::createBoundingSphere()
{
    return Mesh(sphereBoundsPBD);
}

Mesh Mesh::createBoundingCylinder()
{
    return Mesh(cylinderBoundsPBD);
}

Mesh Mesh::createBoundingPlane()
{
    return Mesh(planeBoundsPBD);
}
Mesh Mesh::createCone()
{
    return Mesh(conePBD);
}

Mesh Mesh::createBoundingCone()
{
    return Mesh(coneBoundsPBD);
}

Mesh Mesh::createGrid(int size)
{
    std::vector<glm::vec3> gridVerts;
    for(int i= -size; i<=size;i++)
    {
        gridVerts.push_back(glm::vec3(i, 0.05f, -size));
        gridVerts.push_back(glm::vec3(i, 0.05f, size));

        gridVerts.push_back(glm::vec3(-size, 0.05f, i));
        gridVerts.push_back(glm::vec3(size, 0.05f, i));
    }
    return Mesh(gridVerts, GL_LINES);
}

Mesh Mesh::createCylinder()
{
    return Mesh(cylinderPBD);
}

Mesh Mesh::createSphere()
{
    return Mesh(spherePBD);
}

PrimitiveBufferData Mesh::extractPrimitiveBufferData(const Mesh& mesh)
{
    PrimitiveBufferData pbd;
    pbd.vao = mesh.vao;
    pbd.type = mesh.type;
    pbd.numVerts = mesh.numVerts;
    pbd.vbo = mesh.vbo;
    return pbd;
}

Mesh::Mesh(const PrimitiveBufferData& pbd)
{
    vao = pbd.vao;
    type = pbd.type;
    numVerts = pbd.numVerts;
    vbo = pbd.vbo;
    if(type == GL_TRIANGLES)
        color = glm::vec3(1,1,1);
    else if(type == GL_LINES)
        color = glm::vec3(0,1,0);
}

void Mesh::initializeStaticArrays()
{
    int divs = SEGMENTS;
    std::vector<glm::vec3> verts;
    for(int i = 0; i<divs; i++)
    {
        float theta0 = (float)i*2.0f*PI/divs;
        float theta1 = (float)(i+1)*2.0f*PI/divs;
        glm::vec3 v1 = glm::vec3(0.5f*glm::cos(theta0), 1.0f, 0.5f*glm::sin(theta0));
        glm::vec3 v2 = glm::vec3(0, 1.0f, 0);
        glm::vec3 v3 = glm::vec3(0.5f*glm::cos(theta1), 1.0f, 0.5f*glm::sin(theta1));

        glm::vec3 normal = glm::cross(v2-v1, v3-v1);
        normal = glm::normalize(normal);

        verts.push_back(v1);
        verts.push_back(normal);
        verts.push_back(v2);
        verts.push_back(normal);
        verts.push_back(v3);
        verts.push_back(normal);

        v3 = glm::vec3(0.5f*glm::cos(theta0), 1.0f, 0.5f*glm::sin(theta0));
        v2 = glm::vec3(0.5f*glm::cos(theta0), -1.0f, 0.5f*glm::sin(theta0));
        v1 = glm::vec3(0.5f*glm::cos(theta1), 1.0f, 0.5f*glm::sin(theta1));

        normal = glm::cross(v2-v1, v3-v1);
        normal = glm::normalize(normal);

        verts.push_back(v1);
        verts.push_back(normal);
        verts.push_back(v2);
        verts.push_back(normal);
        verts.push_back(v3);
        verts.push_back(normal);

        v3 = glm::vec3(0.5f*glm::cos(theta1), 1.0f, 0.5f*glm::sin(theta1));
        v2 = glm::vec3(0.5f*glm::cos(theta0), -1.0f, 0.5f*glm::sin(theta0));
        v1 = glm::vec3(0.5f*glm::cos(theta1), -1.0f, 0.5f*glm::sin(theta1));

        normal = glm::cross(v2-v1, v3-v1);
        normal = glm::normalize(normal);

        verts.push_back(v1);
        verts.push_back(normal);
        verts.push_back(v2);
        verts.push_back(normal);
        verts.push_back(v3);
        verts.push_back(normal);

        v3 = glm::vec3(0.5f*glm::cos(theta0), -1.0f, 0.5f*glm::sin(theta0));
        v2 = glm::vec3(0, -1.0f, 0);
        v1 = glm::vec3(0.5f*glm::cos(theta1), -1.0f, 0.5f*glm::sin(theta1));

        normal = glm::cross(v2-v1, v3-v1);
        normal = glm::normalize(normal);

        verts.push_back(v1);
        verts.push_back(normal);
        verts.push_back(v2);
        verts.push_back(normal);
        verts.push_back(v3);
        verts.push_back(normal);
    }
    cylinderVerts = verts;


    verts = std::vector<glm::vec3>();
    //The outer loop will determine the horizontal angle and point height
    for (int j = 0; j < divs; j++)
    {
        //The inner loop will determine the vertical angle and horizontal position
        for (int i = 0; i < 2 * divs; i++) {

            //The algoithm works by  calculating a theta and psi polar coordiante angles and doing square patches
            //along the face of the sphere
            float theta = (float)i * PI / (float)divs;
            float theta1 = (float)(i + 1) * PI / (float)divs;
            float psi = (float)j * PI / (double)divs + PI / 2.0;
            float psi1 = (float)(j + 1) * PI / (float)divs + PI / 2.0;

            glm::vec3 v3 = glm::vec3(0.5f * glm::cos(theta) * glm::cos(psi), 0.5f * glm::sin(psi), 0.5f * glm::sin(theta) * glm::cos(psi));
            glm::vec3 v2 = glm::vec3(0.5f * glm::cos(theta) * glm::cos(psi1), 0.5f * glm::sin(psi1), 0.5f * glm::sin(theta) * glm::cos(psi1));
            glm::vec3 v1 = glm::vec3(0.5f * glm::cos(theta1) * glm::cos(psi), 0.5f * glm::sin(psi), 0.5f * glm::sin(theta1) * glm::cos(psi));

            glm::vec3 normal = glm::cross(v2-v1, v3-v1);
            normal = glm::normalize(normal);

            verts.push_back(v1);
            verts.push_back(normal);
            verts.push_back(v2);
            verts.push_back(normal);
            verts.push_back(v3);
            verts.push_back(normal);

            v3 = glm::vec3(0.5f * glm::cos(theta) * glm::cos(psi1), 0.5f * glm::sin(psi1), 0.5f * glm::sin(theta) * glm::cos(psi1));
            v2 = glm::vec3(0.5f * glm::cos(theta1) * glm::cos(psi1), 0.5f * glm::sin(psi1), 0.5f * glm::sin(theta1) * glm::cos(psi1));
            v1 = glm::vec3(0.5f * glm::cos(theta1) * glm::cos(psi), 0.5f * glm::sin(psi), 0.5f * glm::sin(theta1) * glm::cos(psi));

            normal = glm::cross(v2-v1, v3-v1);
            normal = glm::normalize(normal);

            verts.push_back(v1);
            verts.push_back(normal);
            verts.push_back(v2);
            verts.push_back(normal);
            verts.push_back(v3);
            verts.push_back(normal);

        }
    }
    sphereVerts = verts;

    verts = std::vector<glm::vec3>();
    for(int i = 0; i<divs; i++)
    {
        float theta0 = (float)i*2.0f*PI/divs;
        float theta1 = (float)(i+1)*2.0f*PI/divs;

        verts.push_back(glm::vec3(0.505f*glm::cos(theta0), 1.0f, 0.505f*glm::sin(theta0)));
        verts.push_back(glm::vec3(0.505f*glm::cos(theta1), 1.0f, 0.505f*glm::sin(theta1)));

        verts.push_back(glm::vec3(0.505f*glm::cos(theta0), -1.0f, 0.505f*glm::sin(theta0)));
        verts.push_back(glm::vec3(0.505f*glm::cos(theta1), -1.0f, 0.505f*glm::sin(theta1)));
    }

    verts.push_back(glm::vec3(0.505f, 1.0f, 0));
    verts.push_back(glm::vec3(0.505f, -1.0f, 0));

    verts.push_back(glm::vec3(-0.505f, 1.0f, 0));
    verts.push_back(glm::vec3(-0.505f, -1.0f, 0));

    verts.push_back(glm::vec3(0, 1.0f, 0.505f));
    verts.push_back(glm::vec3(0, -1.0f, 0.505f));

    verts.push_back(glm::vec3(0, 1.0f, -0.505f));
    verts.push_back(glm::vec3(0, -1.0f, -0.505f));
    boundingCylinderVerts = verts;

    verts = std::vector<glm::vec3>();
    for(int i = 0; i<divs; i++)
    {
        float theta0 = (float)i*2.0f*PI/divs;
        float theta1 = (float)(i+1)*2.0f*PI/divs;

        verts.push_back(glm::vec3(0.505f*glm::cos(theta0), 0.0f, 0.505f*glm::sin(theta0)));
        verts.push_back(glm::vec3(0.505f*glm::cos(theta1), 0.0f, 0.505f*glm::sin(theta1)));

        verts.push_back(glm::vec3(0.0f, 0.505f*glm::cos(theta0), 0.505f*glm::sin(theta0)));
        verts.push_back(glm::vec3(0.0f, 0.505f*glm::cos(theta1), 0.505f*glm::sin(theta1)));

        verts.push_back(glm::vec3(0.505f*glm::cos(theta0), 0.505f*glm::sin(theta0), 0.0f));
        verts.push_back(glm::vec3(0.505f*glm::cos(theta1), 0.505f*glm::sin(theta1), 0.0f));
    }
    boundingSphereVerts = verts;

    verts = std::vector<glm::vec3>();
    for(int i = 0; i<divs; i++)
    {
        float theta0 = (float)i*2.0f*PI/divs;
        float theta1 = (float)(i+1)*2.0f*PI/divs;
        glm::vec3 v1 = glm::vec3(0.5f*glm::cos(theta0), -0.5f, 0.5f*glm::sin(theta0));
        glm::vec3 v2 = glm::vec3(0, 0.5f, 0);
        glm::vec3 v3 = glm::vec3(0.5f*glm::cos(theta1), -0.5f, 0.5f*glm::sin(theta1));

        glm::vec3 normal = glm::cross(v2-v1, v3-v1);
        normal = glm::normalize(normal);

        verts.push_back(v1);
        verts.push_back(normal);
        verts.push_back(v2);
        verts.push_back(normal);
        verts.push_back(v3);
        verts.push_back(normal);

        v3 = glm::vec3(0.5f*glm::cos(theta0), -0.5f, 0.5f*glm::sin(theta0));
        v2 = glm::vec3(0, -0.5f, 0);
        v1 = glm::vec3(0.5f*glm::cos(theta1), -0.5f, 0.5f*glm::sin(theta1));

        normal = glm::cross(v2-v1, v3-v1);
        normal = glm::normalize(normal);

        verts.push_back(v1);
        verts.push_back(normal);
        verts.push_back(v2);
        verts.push_back(normal);
        verts.push_back(v3);
        verts.push_back(normal);
    }
    coneVerts = verts;

    verts = std::vector<glm::vec3>();
    verts.push_back(glm::vec3(0, 0.5f, 0));
    verts.push_back(glm::vec3(0.5f, -0.5f, 0));
    verts.push_back(glm::vec3(0, 0.5f, 0));
    verts.push_back(glm::vec3(-0.5f, -0.5f, 0));
    verts.push_back(glm::vec3(0, 0.5f, 0));
    verts.push_back(glm::vec3(0, -0.5f, 0.5f));
    verts.push_back(glm::vec3(0, 0.5f, 0));
    verts.push_back(glm::vec3(0, -0.5f, -0.5f));
    verts.push_back(glm::vec3(0.5f, -0.5f, 0));
    verts.push_back(glm::vec3(-0.5f, -0.5f, 0));
    verts.push_back(glm::vec3(0, -0.5f, 0.5f));
    verts.push_back(glm::vec3(0, -0.5f, -0.5f));
    boundingConeVerts = verts;

    Mesh cube(cubeVerts, GL_TRIANGLES);
    cubePBD = Mesh::extractPrimitiveBufferData(cube);
    Mesh boundingCube(boundingBoxVerts, GL_LINES);
    cubeBoundsPBD = Mesh::extractPrimitiveBufferData(boundingCube);

    Mesh cyl(cylinderVerts, GL_TRIANGLES);
    cylinderPBD = Mesh::extractPrimitiveBufferData(cyl);
    Mesh boundingCyl(boundingCylinderVerts, GL_LINES);
    cylinderBoundsPBD = Mesh::extractPrimitiveBufferData(boundingCyl);

    Mesh sphere(sphereVerts, GL_TRIANGLES);
    spherePBD = Mesh::extractPrimitiveBufferData(sphere);
    Mesh boundingSphere(boundingSphereVerts, GL_LINES);
    sphereBoundsPBD = Mesh::extractPrimitiveBufferData(boundingSphere);

    Mesh plane(planeVerts, GL_TRIANGLES);
    planePBD = Mesh::extractPrimitiveBufferData(plane);
    Mesh planeBounds(boundingPlaneVerts, GL_LINES);
    planeBoundsPBD = Mesh::extractPrimitiveBufferData(planeBounds);

    Mesh cone(coneVerts, GL_TRIANGLES);
    conePBD = Mesh::extractPrimitiveBufferData(cone);
    Mesh coneBounds(boundingConeVerts, GL_LINES);
    coneBoundsPBD = Mesh::extractPrimitiveBufferData(coneBounds);
    }



void Mesh::draw()
{
    openglFunctions->glBindVertexArray(vao);
    openglFunctions->glDrawArrays(type, 0, numVerts);
}

void Mesh::draw(Shader& shader)
{
    shader.setVec3("color", color);
    draw();
}

void Mesh::setColor(glm::vec3 color)
{
    this->color = color;
}

const GLuint Mesh::getVao()
{
    return vao;
}

const unsigned int Mesh::getNumVerts()
{
    return numVerts;
}

const GLenum Mesh::getType()
{
    return type;
}
