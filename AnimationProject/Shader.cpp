#include "Shader.h"

#include <QFile>
#include <QOpenGLFunctions_4_4_Core>
#include <QString>
extern QOpenGLFunctions_4_4_Core* openglFunctions;

const QString Shader::getShaderSource(const char* path)
{
    QString code;
    QFile file(path);
    if(file.open(QIODevice::ReadOnly))
    {
        QTextStream stream(&file);
        code = stream.readAll();
    }
    return code;
}

Shader::Shader(const char* vertexPath, const char* fragmentPath)
{
    shader.addShaderFromSourceCode(QOpenGLShader::Vertex, getShaderSource("model.vert").toStdString().c_str());
    shader.addShaderFromSourceCode(QOpenGLShader::Fragment, getShaderSource("model.frag").toStdString().c_str());
    shader.link();
    handle = shader.programId();
    use();
}

void Shader::use()
{
    openglFunctions->glUseProgram(handle);
}

void Shader::insertUniform(const std::string& uniformName)
{
    use();
    GLuint id = openglFunctions->glGetUniformLocation(handle, uniformName.c_str());
    uniforms[uniformName] = id;
}

void Shader::setVec3(const std::string& uniformName, const glm::vec3& value)
{
    use();
    openglFunctions->glUniform3fv(uniforms[uniformName], 1, &value[0]);
}

void Shader::setMat4(const std::string& uniformName, const glm::mat4& value)
{
    use();
    openglFunctions->glUniformMatrix4fv(uniforms[uniformName], 1, false, &value[0][0]);
}

const GLuint Shader::getUniformLocation(const std::string& uniformName)
{
    return uniforms[uniformName];
}

const GLuint Shader::getHandle()
{
    return handle;
}
