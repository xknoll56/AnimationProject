#include "Mesh.h"
extern QOpenGLFunctions_3_3_Core* openglFunctions;

//this constructor will contain the vertices packed in with the normals
Mesh::Mesh(std::vector<glm::vec3> verts)
{
    openglFunctions->glGenVertexArrays(1, &vao);
    openglFunctions->glGenBuffers(1, &vbo);
    // bind the Vertex Array Object first, then bind and set vertex buffer(s), and then configure vertex attributes(s).
    openglFunctions->glBindVertexArray(vao);

    openglFunctions->glBindBuffer(GL_ARRAY_BUFFER, vbo);
    openglFunctions->glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3)*verts.size(), &verts[0], GL_STATIC_DRAW);

    openglFunctions->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 2 * sizeof(glm::vec3), (void*)0);
    openglFunctions->glEnableVertexAttribArray(0);

    openglFunctions->glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 2 * sizeof(glm::vec3), (void*)(sizeof(glm::vec3)));
    openglFunctions->glEnableVertexAttribArray(1);
    numVerts = verts.size()/2;
    color = glm::vec3(1,1,1);
}

Mesh::Mesh(std::vector<float> verts)
{
    openglFunctions->glGenVertexArrays(1, &vao);
    openglFunctions->glGenBuffers(1, &vbo);
    // bind the Vertex Array Object first, then bind and set vertex buffer(s), and then configure vertex attributes(s).
    openglFunctions->glBindVertexArray(vao);

    openglFunctions->glBindBuffer(GL_ARRAY_BUFFER, vbo);
    openglFunctions->glBufferData(GL_ARRAY_BUFFER, sizeof(float)*verts.size(), &verts[0], GL_STATIC_DRAW);

    openglFunctions->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    openglFunctions->glEnableVertexAttribArray(0);

    openglFunctions->glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3*sizeof(float)));
    openglFunctions->glEnableVertexAttribArray(1);
    numVerts = verts.size()/6;
    color = glm::vec3(1,1,1);
}

Mesh Mesh::createCube()
{
    return Mesh(cubeVerts);
}

void Mesh::draw()
{
    openglFunctions->glBindVertexArray(vao);
    openglFunctions->glDrawArrays(GL_TRIANGLES, 0, numVerts);
}

void Mesh::draw(Shader& shader)
{
   shader.setVec3("color", color);
   openglFunctions->glBindVertexArray(vao);
   openglFunctions->glDrawArrays(GL_TRIANGLES, 0, numVerts);
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
