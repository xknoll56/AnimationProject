#include <glm/glm.hpp>
#include <QOpenGLFunctions_4_5_Core>
#include <Shader.h>

#ifndef MESH_H
#define MESH_H

#define SEGMENTS 30;

struct PrimitiveBufferData;

//This structure will be use to store pointers to all the vertex data in the
//gpu so that vertex buffers will only be allocated once
struct PrimitiveBufferData
{
    GLuint vao, vbo;
    unsigned int numVerts;
    GLenum type;
};


class Mesh
{
private:
    GLuint vao, vbo;
    //some objects will use an element/normal buffers separate
    GLuint nbo, ebo;
    unsigned int numVerts;
    glm::vec3 color;
    GLenum type;



public:
    //this constructor will contain the vertices packed in with the normals
    Mesh(std::vector<glm::vec3> verts, GLenum type);
    Mesh(std::vector<float> verts, GLenum type);
    Mesh(const PrimitiveBufferData& pbd);
    Mesh(const Mesh& mesh);

    static Mesh createCube();
    static Mesh createPlane();
    static Mesh createBoundingBox();
    static Mesh createBoundingSphere();
    static Mesh createBoundingCylinder();
    static Mesh createBoundingPlane();
    static Mesh createGrid(int size);
    static Mesh createCylinder();
    static Mesh createSphere();
    static Mesh createCone();
    static Mesh createBoundingCone();
    static Mesh createCapsule();
    static Mesh createBoundingCapsule();
    static void initializeStaticArrays();
    void draw();
    void draw(Shader& shader);
    void setColor(glm::vec3 color);
    const GLuint getVao();
    const unsigned int getNumVerts();
    const GLenum getType();
    static PrimitiveBufferData extractPrimitiveBufferData(const Mesh& mesh);

};

static std::vector<glm::vec3> capsuleVerts;

static std::vector<glm::vec3> boundingCapsuleVerts;

static std::vector<glm::vec3> coneVerts;

static std::vector<glm::vec3> boundingConeVerts;

static std::vector<glm::vec3> cylinderVerts;

static std::vector<glm::vec3> sphereVerts;

static const std::vector<float> planeVerts =
{
    -0.5f,  0.0f, -0.5f,  0.0f, 1.0f, 0.0f,
    0.5f,  0.0f,  0.5f,  0.0f, 1.0f, 0.0f,
    0.5f,  0.0f, -0.5f,  0.0f, 1.0f, 0.0f,
    -0.5f,  0.0f, -0.5f,  0.0f, 1.0f, 0.0f,
    -0.5f,  0.0f,  0.5f,  0.0f, 1.0f, 0.0f,
    0.5f,  0.0f,  0.5f,  0.0f, 1.0f, 0.0f,
};

static const std::vector<glm::vec3> boundingPlaneVerts =
{
    glm::vec3(-0.5f, 0.05f, -0.5f),
    glm::vec3(-0.5f, 0.05f, 0.5f),

    glm::vec3(0.5f, 0.05f, 0.5f),
    glm::vec3(0.5f, 0.05f, -0.5f),

    glm::vec3(-0.5f, 0.05f, 0.5f),
    glm::vec3(0.5f, 0.05f, 0.5f),

    glm::vec3(0.5f, 0.05f, -0.5f),
    glm::vec3(-0.5f, 0.05f, -0.5f),

};

static std::vector<glm::vec3> boundingSphereVerts;

static std::vector<glm::vec3> boundingCylinderVerts;

static const std::vector<glm::vec3> boundingBoxVerts =
{
    glm::vec3(-0.505f, 0.505f, -0.505f), glm::vec3(-0.505f, 0.505f, 0.505f),
    glm::vec3(-0.505f, 0.505f, 0.505f), glm::vec3(0.505f, 0.505f, 0.505f),
    glm::vec3(0.505f, 0.505f, 0.505f), glm::vec3(0.505f, 0.505f, -0.505f),
    glm::vec3(0.505f, 0.505f, -0.505f), glm::vec3(-0.505f, 0.505f, -0.505f),

    glm::vec3(-0.505f, -0.505f, -0.505f), glm::vec3(-0.505f, -0.505f, 0.505f),
    glm::vec3(-0.505f, -0.505f, 0.505f), glm::vec3(0.505f, -0.505f, 0.505f),
    glm::vec3(0.505f, -0.505f, 0.505f), glm::vec3(0.505f, -0.505f, -0.505f),
    glm::vec3(0.505f, -0.505f, -0.505f), glm::vec3(-0.505f, -0.505f, -0.505f),

    glm::vec3(-0.505f, 0.505f, -0.505f), glm::vec3(-0.505f, -0.505f, -0.505f),
    glm::vec3(-0.505f, 0.505f, 0.505f), glm::vec3(-0.505f, -0.505f, 0.505f),
    glm::vec3(0.505f, 0.505f, 0.505f), glm::vec3(0.505f, -0.505f, 0.505f),
    glm::vec3(0.505f, 0.505f, -0.505f), glm::vec3(0.505f, -0.505f, -0.505f),
};

static const std::vector<float> cubeVerts=
{
    -0.5f, -0.5f, -0.5f,  0.0f, 0.0f, -1.0f,
    0.5f,  0.5f, -0.5f,  0.0f, 0.0f, -1.0f,
    0.5f, -0.5f, -0.5f,  0.0f, 0.0f, -1.0f,
    0.5f,  0.5f, -0.5f,  0.0f, 0.0f, -1.0f,
    -0.5f, -0.5f, -0.5f,  0.0f, 0.0f, -1.0f,
    -0.5f,  0.5f, -0.5f,  0.0f, 0.0f, -1.0f,

    -0.5f, -0.5f,  0.5f,  0.0f, 0.0f, 1.0f,
    0.5f, -0.5f,  0.5f,  0.0f, 0.0f, 1.0f,
    0.5f,  0.5f,  0.5f,  0.0f, 0.0f, 1.0f,
    0.5f,  0.5f,  0.5f,  0.0f, 0.0f, 1.0f,
    -0.5f,  0.5f,  0.5f,  0.0f, 0.0f, 1.0f,
    -0.5f, -0.5f,  0.5f,  0.0f, 0.0f, 1.0f,

    -0.5f,  0.5f,  0.5f,  -1.0f, 0.0f, 0.0f,
    -0.5f,  0.5f, -0.5f,  -1.0f, 0.0f, 0.0f,
    -0.5f, -0.5f, -0.5f,  -1.0f, 0.0f, 0.0f,
    -0.5f, -0.5f, -0.5f,  -1.0f, 0.0f, 0.0f,
    -0.5f, -0.5f,  0.5f,  -1.0f, 0.0f, 0.0f,
    -0.5f,  0.5f,  0.5f,  -1.0f, 0.0f, 0.0f,

    0.5f,  0.5f,  0.5f,  1.0f, 0.0f, 0.0f,
    0.5f, -0.5f, -0.5f,  1.0f, 0.0f, 0.0f,
    0.5f,  0.5f, -0.5f,  1.0f, 0.0f, 0.0f,
    0.5f, -0.5f, -0.5f,  1.0f, 0.0f, 0.0f,
    0.5f,  0.5f,  0.5f,  1.0f, 0.0f, 0.0f,
    0.5f, -0.5f,  0.5f,  1.0f, 0.0f, 0.0f,

    -0.5f, -0.5f, -0.5f,  0.0f, -1.0f, 0.0f,
    0.5f, -0.5f, -0.5f,  0.0f, -1.0f, 0.0f,
    0.5f, -0.5f,  0.5f,  0.0f, -1.0f, 0.0f,
    0.5f, -0.5f,  0.5f,  0.0f, -1.0f, 0.0f,
    -0.5f, -0.5f,  0.5f,  0.0f, -1.0f, 0.0f,
    -0.5f, -0.5f, -0.5f,  0.0f, -1.0f, 0.0f,

    -0.5f,  0.5f, -0.5f,  0.0f, 1.0f, 0.0f,
    0.5f,  0.5f,  0.5f,  0.0f, 1.0f, 0.0f,
    0.5f,  0.5f, -0.5f,  0.0f, 1.0f, 0.0f,
    -0.5f,  0.5f, -0.5f,  0.0f, 1.0f, 0.0f,
    -0.5f,  0.5f,  0.5f,  0.0f, 1.0f, 0.0f,
    0.5f,  0.5f,  0.5f,  0.0f, 1.0f, 0.0f,
};


#endif // MESH_H
