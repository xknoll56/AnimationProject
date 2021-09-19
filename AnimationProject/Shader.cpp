#include "Shader.h"

#include <QFile>
#include <QOpenGLFunctions_3_3_Core>
#include <QString>
extern QOpenGLFunctions_3_3_Core* openglFunctions;

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

void Shader::insertUniform(const char* name)
{
    use();
    GLuint id = openglFunctions->glGetUniformLocation(handle, name);
    uniforms[name] = id;
}

void Shader::setVec3(const char* uniformName, const glm::vec3& value)
{
    use();
    openglFunctions->glUniform3f(uniforms[uniformName], value.x, value.y, value.z);
}

void Shader::setMat4(const char* uniformName, const glm::mat4& value)
{
    use();
    openglFunctions->glUniformMatrix4fv(uniforms[uniformName], 1, false, &value[0][0]);
}

const GLuint Shader::getUniformLocation(const char* uniformName)
{
    return uniforms[uniformName];
}

const GLuint Shader::getHandle()
{
    return handle;
}
