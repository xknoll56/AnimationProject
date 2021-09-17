
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
#include <QElapsedTimer>
#include <QOpenGLShaderProgram>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <fstream>
#include <string>
#include <stdio.h>
#include <sstream>
#include <chrono>
#include <map>

#define PI 3.14159265359f

double dt;
float pitch, yaw;

std::map<int, bool> keys;

class MainWindow: public QWindow
{
public:
    bool running;
    MainWindow() : QWindow()
    {
        setKeyboardGrabEnabled(true);
        running = true;
        keys.insert(std::pair<int, bool>(Qt::Key_Left, false));
        keys.insert(std::pair<int, bool>(Qt::Key_Right, false));
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
        keys[event->key()] = true;
    }

    void keyReleaseEvent(QKeyEvent* event) override
    {
        keys[event->key()] = false;
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
    return code.c_str();
}




glm::mat4 FPSViewRH( glm::vec3 eye, float pitch, float yaw )
{

    float cosPitch = glm::cos(pitch);
    float sinPitch = glm::sin(pitch);
    float cosYaw = glm::cos(yaw);
    float sinYaw = glm::sin(yaw);

    glm::vec3 xaxis = { cosYaw, 0, -sinYaw };
    glm::vec3 yaxis = { sinYaw * sinPitch, cosPitch, cosYaw * sinPitch };
    glm::vec3 zaxis = { sinYaw * cosPitch, -sinPitch, cosPitch * cosYaw };

    glm::mat4 viewMatrix =
    {

        glm::vec4(       xaxis.x,            yaxis.x,            zaxis.x,      0 ),
        glm::vec4(       xaxis.y,            yaxis.y,            zaxis.y,      0 ),
        glm::vec4(       xaxis.z,            yaxis.z,            zaxis.z,      0 ),
        glm::vec4( -glm::dot( xaxis, eye ), -glm::dot( yaxis, eye ), -glm::dot( zaxis, eye ), 1 )
    };
    return viewMatrix;
}





int main(int argc, char *argv[])
{
    qDebug("Starting");
    QGuiApplication app(argc, argv);

    QSurfaceFormat format;
    format.setSamples(4);
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

    QOpenGLPaintDevice* paintDevice = new QOpenGLPaintDevice;
    paintDevice->setSize(window.size() * window.devicePixelRatio());
    paintDevice->setDevicePixelRatio(window.devicePixelRatio());
    QPainter painter(paintDevice);
    painter.setPen(Qt::white);
    painter.setFont(QFont("Arial", 12));
    QRectF rect(0.0f,0.0f,paintDevice->size().width(), paintDevice->size().height());
    painter.setWorldMatrixEnabled(false);

    QOpenGLFunctions_3_3_Core* openGLFunctions = context->versionFunctions<QOpenGLFunctions_3_3_Core>();
    if(!openGLFunctions)
    {
        qDebug("Could not obtain required version of opengl");
        app.exit();
    }
    openGLFunctions->initializeOpenGLFunctions();

    openGLFunctions->glViewport(0, 0, window.width() * window.devicePixelRatio(), window.height() * window.devicePixelRatio());
    openGLFunctions->glEnable(GL_DEPTH_TEST);
    openGLFunctions->glEnable(GL_CULL_FACE);


    QOpenGLShaderProgram modelShader;
    modelShader.addShaderFromSourceCode(QOpenGLShader::Vertex, getShaderSource("model.vert"));
    modelShader.addShaderFromSourceCode(QOpenGLShader::Fragment, getShaderSource("model.frag"));
    modelShader.link();
    modelShader.bind();


    glm::vec3 euler(0,0,0);
    glm::mat4 trans(1.0f);
    GLuint modelLoc = modelShader.uniformLocation("model");
    GLuint viewLoc = modelShader.uniformLocation("view");
    GLuint projectionLoc = modelShader.uniformLocation("projection");
    glm::mat4 projection = glm::perspective((float)PI*0.33f, (float)window.devicePixelRatio(), 0.1f, 100.0f);
    //glm::mat4 view = glm::lookAt(glm::vec3(2,2, -5.0f), glm::vec3(0,0,0), glm::vec3(0,1,0));
    pitch = yaw = 0;
    glm::mat4 view = FPSViewRH(glm::vec3(0, 2, 5.0f), pitch, yaw);
    openGLFunctions->glUniformMatrix4fv(modelLoc, 1, false, &trans[0][0]);
    openGLFunctions->glUniformMatrix4fv(viewLoc, 1, false, &view[0][0]);
    openGLFunctions->glUniformMatrix4fv(projectionLoc, 1, false, &projection[0][0]);
    unsigned int shaderProgram = modelShader.programId();

    // set up vertex data (and buffer(s)) and configure vertex attributes
    // ------------------------------------------------------------------
    float vertices[] = {
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

    QElapsedTimer timer;
    timer.start();
    while(window.running)
    {
        dt = timer.nsecsElapsed()/1000000000.0;
        timer.restart();
        qDebug() << timer.nsecsElapsed() << "\n";
        openGLFunctions->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        openGLFunctions->glUseProgram(shaderProgram);
        if(keys[Qt::Key_Left])
             yaw+=dt;
        if(keys[Qt::Key_Right])
             yaw-=dt;
        view = FPSViewRH(glm::vec3(0, 2, 5.0f), pitch, yaw);
        euler += glm::vec3(dt, 0, 0);
        glm::mat4 rot = glm::rotate(trans, glm::length(euler), euler);
        openGLFunctions->glUniformMatrix4fv(modelLoc, 1, false, &rot[0][0]);
        openGLFunctions->glUniformMatrix4fv(viewLoc, 1, false, &view[0][0]);

        openGLFunctions->glBindVertexArray(VAO); // seeing as we only have a single VAO there's no need to bind it every time, but we'll do so to keep things a bit more organized
        openGLFunctions->glDrawArrays(GL_TRIANGLES, 0, 36);

        openGLFunctions->glUniformMatrix4fv(modelLoc, 1, false, &trans[0][0]);


        painter.beginNativePainting();
        painter.drawText(rect, "Hello, World!");
        painter.endNativePainting();

        app.processEvents();
        context->makeCurrent(&window);
        context->swapBuffers(&window);
    }

    app.quit();
}
