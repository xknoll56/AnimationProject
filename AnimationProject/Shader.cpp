#include "Shader.h"

#include <QFile>
#include <QOpenGLFunctions_4_5_Core>
#include <QString>
extern QOpenGLFunctions_4_5_Core* openglFunctions;

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
    std::string vShaderString = getShaderSource(vertexPath).toStdString();
    std::string fShaderString = getShaderSource(fragmentPath).toStdString();

    const char* vSource = vShaderString.c_str();
    const char* fSource = fShaderString.c_str();
    unsigned int vertex, fragment;
    int success;
    char infoLog[512];

    // vertex Shader
    vertex = openglFunctions->glCreateShader(GL_VERTEX_SHADER);
    openglFunctions->glShaderSource(vertex, 1, &vSource, NULL);
    openglFunctions->glCompileShader(vertex);
    // print compile errors if any
    openglFunctions->glGetShaderiv(vertex, GL_COMPILE_STATUS, &success);
    if(!success)
    {
        openglFunctions->glGetShaderInfoLog(vertex, 512, NULL, infoLog);
        qDebug() << "ERROR::SHADER::VERTEX::COMPILATION_FAILED\n" << infoLog;
    };

    // similiar for Fragment Shader
    fragment = openglFunctions->glCreateShader(GL_FRAGMENT_SHADER);
    openglFunctions->glShaderSource(fragment, 1, &fSource, NULL);
    openglFunctions->glCompileShader(fragment);
    // print compile errors if any
    openglFunctions->glGetShaderiv(fragment, GL_COMPILE_STATUS, &success);
    if(!success)
    {
        openglFunctions->glGetShaderInfoLog(fragment, 512, NULL, infoLog);
        qDebug() << "ERROR::SHADER::VERTEX::COMPILATION_FAILED\n" << infoLog;
    };

    // shader Program
    handle = openglFunctions->glCreateProgram();
    openglFunctions->glAttachShader(handle, vertex);
    openglFunctions->glAttachShader(handle, fragment);
    openglFunctions->glLinkProgram(handle);
    // print linking errors if any
    openglFunctions->glGetProgramiv(handle, GL_LINK_STATUS, &success);
    if(!success)
    {
        openglFunctions->glGetProgramInfoLog(handle, 512, NULL, infoLog);
        qDebug() << "ERROR::SHADER::PROGRAM::LINKING_FAILED\n" << infoLog;
    }

    // delete the shaders as they're linked into our program now and no longer necessary
    openglFunctions->glDeleteShader(vertex);
    openglFunctions->glDeleteShader(fragment);
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
