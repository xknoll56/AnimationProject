
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
    float theta = glm::acos(dir.x);
    theta = dir.z>0?-theta:theta;
    float psi = glm::asin(dir.y);
    glm::mat4 trans(1.0f);
    trans = glm::translate(trans, from);
    trans = glm::rotate(trans, theta, glm::vec3(0,1,0));
    trans = glm::rotate(trans, psi, glm::vec3(0,0,1));
    trans = glm::scale(trans, glm::vec3(dist, 1, 1));
    gridShader->setMat4("model", trans);
    line.draw(*gridShader);
}

