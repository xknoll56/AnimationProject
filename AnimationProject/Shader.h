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
    std::map<const char*, GLuint> uniforms;
    QOpenGLShaderProgram shader;

public:
    Shader(const char* vertexPath, const char* fragmentPath);
    void use();
    void insertUniform(const char* name);
    void setVec3(const char* uniformName, const glm::vec3& value);
    void setMat4(const char* uniformName, const glm::mat4& value);
    const GLuint getUniformLocation(const char* uniformName);
    const GLuint getHandle();

};

#endif // SHADER_H
