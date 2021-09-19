#ifndef SHADER_H
#define SHADER_H

#include <QOpenGLShaderProgram>
#include <QString>
#include <glm/glm.hpp>



class Shader
{
private:
    const QString getShaderSource(const char* path);
    GLuint handle;
    std::map<std::string, GLuint> uniforms;
    QOpenGLShaderProgram shader;

public:
    Shader(const char* vertexPath, const char* fragmentPath);
    void use();
    void insertUniform(std::string uniformName);
    void setVec3(std::string uniformName, const glm::vec3& value);
    void setMat4(std::string uniformName, const glm::mat4& value);
    const GLuint getUniformLocation(std::string uniformName);
    const GLuint getHandle();

};

#endif // SHADER_H
