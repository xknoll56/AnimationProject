
#include <QApplication>
#include <QSurfaceFormat>
#include <QWindow>
#include <QtOpenGL>
#include <QOpenGLContext>
#include <QOpenGLPaintDevice>
#include <QOpenGLFunctions>
#include <QOpenGLFunctions_4_5_Core>
#include <QOpenGLFunctions_3_3_Core>
#include <QPainter>
#include <QMouseEvent>
#include <QDebug>
#include <QCloseEvent>
#include <QOpenGLShaderProgram>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <fstream>
#include <string>
#include <stdio.h>
#include <sstream>
#include <chrono>

#define PI 3.14159265359f


class MainWindow: public QWindow
{
public:
bool running;
MainWindow() : QWindow()
{
    setKeyboardGrabEnabled(true);
    running = true;
}

void closeEvent(QCloseEvent* event)
{
    event->accept();
    running = true;
}

void keyPressEvent(QKeyEvent* event) override
{
    switch(event->key())
    {
    case Qt::Key_Escape:
        running = false;
    }
}

bool event(QEvent* event) override
{
    switch(event->type())
    {
    case QEvent::Close:
        running = false;
    default:
        return QWindow::event(event);
    }
}
};

const char *vertexShaderSource = "#version 330 core\n"
    "layout (location = 0) in vec3 aPos;\n"
    "void main()\n"
    "{\n"
    "   gl_Position = vec4(aPos.x, aPos.y, aPos.z, 1.0);\n"
    "}\0";
const char *fragmentShaderSource = "#version 330 core\n"
    "out vec4 FragColor;\n"
    "void main()\n"
    "{\n"
    "   FragColor = vec4(1.0f, 0.5f, 0.2f, 1.0f);\n"
    "}\n\0";

const char* getShaderSource(const char* path)
{
    std::string code;
    std::ifstream shaderStream(path);
    if(shaderStream.is_open())
    {
        std::stringstream ss;
        ss << shaderStream.rdbuf();
        ss << '\0';
        code = ss.str();
        shaderStream.close();
    }
    qDebug(code.c_str());
    return code.c_str();
}

int main(int argc, char *argv[])
{
    qDebug("Starting");
    QGuiApplication app(argc, argv);

    QSurfaceFormat format;
    format.setSamples(16);
    format.setDepthBufferSize(24);
    format.setMajorVersion(4);
    format.setMinorVersion(3);
    format.setSwapInterval(0);
    format.setProfile(QSurfaceFormat::CoreProfile);

    MainWindow window;
    window.setTitle("Animation Project");
    window.setFormat(format);
    window.setSurfaceType(QWindow::OpenGLSurface);
    window.resize(1024, 768);
    window.show();

    QOpenGLContext* context = new QOpenGLContext(&window);
    context->setFormat(window.requestedFormat());
    context->create();
    context->makeCurrent(&window);

    QOpenGLFunctions_3_3_Core* openGLFunctions = context->versionFunctions<QOpenGLFunctions_3_3_Core>();
//    QOpenGLFunctions* openGLFunctions = new QOpenGLFunctions(context);
//    openGLFunctions->initializeOpenGLFunctions();
    if(!openGLFunctions)
    {
        qDebug("Could not obtain required version of opengl");
        app.exit();
    //openGLFunctions.initializeOpenGLFunctions();
    }
    QOpenGLPaintDevice* paintDevice = new QOpenGLPaintDevice;
    paintDevice->setSize(window.size() * window.devicePixelRatio());
    paintDevice->setDevicePixelRatio(window.devicePixelRatio());

    QOpenGLShaderProgram program;
    program.addShaderFromSourceCode(QOpenGLShader::Vertex, getShaderSource("model.vert"));
    program.addShaderFromSourceCode(QOpenGLShader::Fragment, getShaderSource("model.frag"));
    program.link();
    program.bind();

    openGLFunctions->glViewport(0, 0, window.width() * window.devicePixelRatio(), window.height() * window.devicePixelRatio());
    openGLFunctions->glEnable(GL_DEPTH_TEST);
    glm::vec3 euler(0,0,0);
    glm::mat4 trans(1.0f);
    GLuint modelLoc = program.uniformLocation("model");
    GLuint viewLoc = program.uniformLocation("view");
    GLuint projectionLoc = program.uniformLocation("projection");
    glm::mat4 projection = glm::perspective((float)PI*0.33f, (float)window.devicePixelRatio(), 0.1f, 100.0f);
    glm::mat4 view = glm::lookAt(glm::vec3(2,2, -5.0f), glm::vec3(0,0,0), glm::vec3(0,1,0));

    openGLFunctions->glUniformMatrix4fv(modelLoc, 1, false, &trans[0][0]);
    openGLFunctions->glUniformMatrix4fv(viewLoc, 1, false, &view[0][0]);
    openGLFunctions->glUniformMatrix4fv(projectionLoc, 1, false, &projection[0][0]);
    unsigned int shaderProgram = program.programId();

    // set up vertex data (and buffer(s)) and configure vertex attributes
    // ------------------------------------------------------------------
    float vertices[] = {
        -0.5f, -0.5f, -0.5f,  0.0f, 0.0f, -1.0f,
         0.5f, -0.5f, -0.5f,  0.0f, 0.0f, -1.0f,
         0.5f,  0.5f, -0.5f,  0.0f, 0.0f, -1.0f,
         0.5f,  0.5f, -0.5f,  0.0f, 0.0f, -1.0f,
        -0.5f,  0.5f, -0.5f,  0.0f, 0.0f, -1.0f,
        -0.5f, -0.5f, -0.5f,  0.0f, 0.0f, -1.0f,

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
         0.5f,  0.5f, -0.5f,  1.0f, 0.0f, 0.0f,
         0.5f, -0.5f, -0.5f,  1.0f, 0.0f, 0.0f,
         0.5f, -0.5f, -0.5f,  1.0f, 0.0f, 0.0f,
         0.5f, -0.5f,  0.5f,  1.0f, 0.0f, 0.0f,
         0.5f,  0.5f,  0.5f,  1.0f, 0.0f, 0.0f,

        -0.5f, -0.5f, -0.5f,  0.0f, -1.0f, 0.0f,
         0.5f, -0.5f, -0.5f,  0.0f, -1.0f, 0.0f,
         0.5f, -0.5f,  0.5f,  0.0f, -1.0f, 0.0f,
         0.5f, -0.5f,  0.5f,  0.0f, -1.0f, 0.0f,
        -0.5f, -0.5f,  0.5f,  0.0f, -1.0f, 0.0f,
        -0.5f, -0.5f, -0.5f,  0.0f, -1.0f, 0.0f,

        -0.5f,  0.5f, -0.5f,  0.0f, 1.0f, 0.0f,
         0.5f,  0.5f, -0.5f,  0.0f, 1.0f, 0.0f,
         0.5f,  0.5f,  0.5f,  0.0f, 1.0f, 0.0f,
         0.5f,  0.5f,  0.5f,  0.0f, 1.0f, 0.0f,
        -0.5f,  0.5f,  0.5f,  0.0f, 1.0f, 0.0f,
        -0.5f,  0.5f, -0.5f,  0.0f, 1.0f, 0.0f
    };
    unsigned int VBO, VAO;

    openGLFunctions->glGenVertexArrays(1, &VAO);
    openGLFunctions->glGenBuffers(1, &VBO);
    // bind the Vertex Array Object first, then bind and set vertex buffer(s), and then configure vertex attributes(s).
    openGLFunctions->glBindVertexArray(VAO);

    openGLFunctions->glBindBuffer(GL_ARRAY_BUFFER, VBO);
    openGLFunctions->glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    openGLFunctions->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    openGLFunctions->glEnableVertexAttribArray(0);

    openGLFunctions->glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));
    openGLFunctions->glEnableVertexAttribArray(1);

    // note that this is allowed, the call to glVertexAttribPointer registered VBO as the vertex attribute's bound vertex buffer object so afterwards we can safely unbind
    openGLFunctions->glBindBuffer(GL_ARRAY_BUFFER, 0);

    // You can unbind the VAO afterwards so other VAO calls won't accidentally modify this VAO, but this rarely happens. Modifying other
    // VAOs requires a call to glBindVertexArray anyways so we generally don't unbind VAOs (nor VBOs) when it's not directly necessary.
    openGLFunctions->glBindVertexArray(0);

    auto elapsedTime = std::chrono::high_resolution_clock::now();
    double dt;
    while(window.running)
    {
        auto newTime = std::chrono::high_resolution_clock::now();
        dt = std::chrono::duration<double, std::milli>(newTime-elapsedTime).count()/1000.0;
        elapsedTime = newTime;

        app.processEvents();

        openGLFunctions->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

        openGLFunctions->glUseProgram(shaderProgram);
        euler += glm::vec3(dt, 0.5*dt, 0.25*dt);
        glm::mat4 rot = glm::rotate(trans, glm::length(euler), euler);
        openGLFunctions->glUniformMatrix4fv(modelLoc, 1, false, &rot[0][0]);
        openGLFunctions->glBindVertexArray(VAO); // seeing as we only have a single VAO there's no need to bind it every time, but we'll do so to keep things a bit more organized
        openGLFunctions->glDrawArrays(GL_TRIANGLES, 0, 36);

        context->makeCurrent(&window);
        context->swapBuffers(&window);

    }

    app.quit();
}
