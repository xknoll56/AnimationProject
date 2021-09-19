#include <glm/glm.hpp>
#include <QOpenGLFunctions_3_3_Core>
#include <Shader.h>

#ifndef MESH_H
#define MESH_H

class Mesh
{
private:
    GLuint vao, vbo;
    //some objects will use an element/normal buffers separate
    GLuint nbo, ebo;
    std::vector<glm::vec3> verts;
    std::vector<glm::vec3> norms;
    unsigned int numVerts;
    glm::vec3 color;


public:
    //this constructor will contain the vertices packed in with the normals
    Mesh(std::vector<glm::vec3> verts);
    Mesh(std::vector<float> verts);
    static Mesh createCube();
    void draw();
    void draw(Shader& shader);
    void setColor(glm::vec3 color);
    const GLuint getVao();
    const unsigned int getNumVerts();

};

static const std::vector<float> cubeVerts({
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
});

#endif // MESH_H
