
#include "Mesh.h"
#include "glm/glm.hpp"
#include <glm/common.hpp>
#include <glm/gtc/matrix_transform.hpp>

extern Shader* gridShader;

void drawLine(Mesh& line, glm::vec3 from, glm::vec3 to)
{
    glm::vec3 dir = to-from;
    float dist = glm::length(dir);
    dir = glm::normalize(dir);
    glm::vec3 dirXZ(dir.x, 0, dir.z);
    dirXZ = glm::normalize(dirXZ);
    float theta = -glm::asin(dirXZ.z);
    theta = dirXZ.x<=0.0f?(3.14159f-theta):theta;
    float psi = glm::atan(dir.y/glm::sqrt(dir.x*dir.x+dir.z*dir.z));
    if(dir.x==0.0f && dir.z ==0.0f)
    {
        dir.y>0.0f?psi=3.14159f*0.5f:psi=-3.141595f*0.5f;
        theta = 0.0f;
    }

    glm::mat4 trans(1.0f);
    trans = glm::translate(trans, from);
    trans = glm::rotate(trans, theta, glm::vec3(0,1,0));
    trans = glm::rotate(trans, psi, glm::vec3(0,0,1));
    trans = glm::scale(trans, glm::vec3(dist, 1, 1));
    gridShader->setMat4("model", trans);
    line.draw(*gridShader);
}

void drawLine(Mesh& line, glm::vec3 from, glm::vec3 dir, float len)
{

    glm::mat4 trans(1.0f);
    trans = glm::lookAt(from, from+dir*len, glm::vec3(0, 1, 0));
    gridShader->setMat4("model", trans);
    line.draw(*gridShader);
}


