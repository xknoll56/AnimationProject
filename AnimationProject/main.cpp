#define QT_ONSCREEN_PAINT
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
//#include <glm/ext.hpp>
#include <glm/common.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
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
float camSpeed = 5.0f;
glm::vec3 camfwd;
glm::vec3 camup;
glm::vec3 camright;
glm::vec3 camPos;
QPointF mousePos;

std::map<int, bool> inputs;
std::map<int, bool> inputsDown;
std::map<int, bool> inputsDownReset;

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

    void mousePressEvent(QMouseEvent *event) override
    {
        inputs[event->button()] = true;
        if(!inputsDownReset[event->button()])
        {
            mousePos = QCursor::pos();
            inputsDown[event->button()] = true;
            inputsDownReset[event->button()] = true;
        }
    }

    void mouseReleaseEvent(QMouseEvent *event) override
    {

        inputs[event->button()] = false;
        inputsDownReset[event->button()] = false;

    }

    void keyPressEvent(QKeyEvent* event) override
    {
        switch(event->key())
        {
        case Qt::Key_Escape:
            running = false;
        }
        inputs[event->key()] = true;
        if(!inputsDownReset[event->key()])
        {
            mousePos = QCursor::pos();
            inputsDown[event->key()] = true;
            inputsDownReset[event->key()] = true;
        }
    }

    void keyReleaseEvent(QKeyEvent* event) override
    {
        if(!event->isAutoRepeat())
        {
            inputs[event->key()] = false;
            inputsDownReset[event->key()] = false;
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

const QString getShaderSource(const char* path)
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




glm::mat4 FPSViewRH( glm::vec3 eye, float pitch, float yaw )
{

    float cosPitch = glm::cos(pitch);
    float sinPitch = glm::sin(pitch);
    float cosYaw = glm::cos(yaw);
    float sinYaw = glm::sin(yaw);

    camright = { cosYaw, 0, -sinYaw };
    camup = { sinYaw * sinPitch, cosPitch, cosYaw * sinPitch };
    camfwd = { sinYaw * cosPitch, -sinPitch, cosPitch * cosYaw };

    glm::mat4 viewMatrix =
    {

        glm::vec4(       camright.x,            camup.x,            camfwd.x,      0 ),
        glm::vec4(       camright.y,            camup.y,            camfwd.y,      0 ),
        glm::vec4(       camright.z,            camup.z,            camfwd.z,      0 ),
        glm::vec4( -glm::dot( camright, eye ), -glm::dot( camup, eye ), -glm::dot( camfwd, eye ), 1 )
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
    window.setKeyboardGrabEnabled(true);
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

    QOpenGLFunctions_3_3_Core* openglFunctions = context->versionFunctions<QOpenGLFunctions_3_3_Core>();
    if(!openglFunctions)
    {
        qDebug("Could not obtain required version of opengl");
        app.exit();
    }
    openglFunctions->initializeOpenGLFunctions();

    openglFunctions->glViewport(0, 0, window.width() * window.devicePixelRatio(), window.height() * window.devicePixelRatio());
    openglFunctions->glEnable(GL_DEPTH_TEST);
    openglFunctions->glEnable(GL_CULL_FACE);


    QOpenGLShaderProgram modelShader;
    modelShader.addShaderFromSourceCode(QOpenGLShader::Vertex, getShaderSource("model.vert").toStdString().c_str());
    modelShader.addShaderFromSourceCode(QOpenGLShader::Fragment, getShaderSource("model.frag").toStdString().c_str());
    modelShader.link();
    modelShader.bind();

    QOpenGLShaderProgram gridShader;
    gridShader.addShaderFromSourceCode(QOpenGLShader::Vertex, getShaderSource("grid.vert").toStdString().c_str());
    gridShader.addShaderFromSourceCode(QOpenGLShader::Fragment, getShaderSource("grid.frag").toStdString().c_str());
    gridShader.link();


    glm::vec3 euler(0,0,0);
    glm::mat4 trans(1.0f);
    GLuint modelLoc = modelShader.uniformLocation("model");
    GLuint viewLoc = modelShader.uniformLocation("view");
    GLuint projectionLoc = modelShader.uniformLocation("projection");
    glm::mat4 projection = glm::perspective((float)PI*0.33f, (float)window.devicePixelRatio(), 0.1f, 100.0f);
    pitch = yaw = 0;
    camPos = glm::vec3(0, 2, 5.0f);
    glm::mat4 view = FPSViewRH(camPos, pitch, yaw);
    openglFunctions->glUniformMatrix4fv(modelLoc, 1, false, &trans[0][0]);
    openglFunctions->glUniformMatrix4fv(viewLoc, 1, false, &view[0][0]);
    openglFunctions->glUniformMatrix4fv(projectionLoc, 1, false, &projection[0][0]);
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

    openglFunctions->glGenVertexArrays(1, &VAO);
    openglFunctions->glGenBuffers(1, &VBO);
    // bind the Vertex Array Object first, then bind and set vertex buffer(s), and then configure vertex attributes(s).
    openglFunctions->glBindVertexArray(VAO);

    openglFunctions->glBindBuffer(GL_ARRAY_BUFFER, VBO);
    openglFunctions->glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    openglFunctions->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    openglFunctions->glEnableVertexAttribArray(0);

    openglFunctions->glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));
    openglFunctions->glEnableVertexAttribArray(1);


    QElapsedTimer timer;
    timer.start();

    glm::quat q(glm::vec3(0,0,0));

    while(window.running)
    {
        dt = timer.nsecsElapsed()/1000000000.0;
        timer.restart();
        openglFunctions->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        openglFunctions->glUseProgram(shaderProgram);
        if(inputs[Qt::Key_A])
        {
            camPos -= camSpeed*camright*(float)dt;
        }
        if(inputs[Qt::Key_D])
        {
            camPos += camSpeed*camright*(float)dt;
        }
        if(inputs[Qt::Key_W])
        {
            camPos -= camSpeed*camfwd*(float)dt;
        }
        if(inputs[Qt::Key_S])
        {
            camPos += camSpeed*camfwd*(float)dt;
        }
        if(inputs[Qt::MouseButton::LeftButton])
        {
            QPointF deltaPos = QCursor::pos()-mousePos;
            mousePos = QCursor::pos();
            yaw -= dt*deltaPos.x();
            pitch -= dt*deltaPos.y();
        }
        view = FPSViewRH(camPos, pitch, yaw);
        euler += glm::vec3(dt, 0, 0);
        q = glm::rotate(q, (float)dt, glm::vec3(1,1,0));
        //glm::mat4 rot = glm::rotate(trans, glm::length(euler), euler);
        glm::mat4 rot = glm::toMat4(q);
        openglFunctions->glUniformMatrix4fv(modelLoc, 1, false, &rot[0][0]);
        openglFunctions->glUniformMatrix4fv(viewLoc, 1, false, &view[0][0]);

        openglFunctions->glBindVertexArray(VAO); // seeing as we only have a single VAO there's no need to bind it every time, but we'll do so to keep things a bit more organized
        openglFunctions->glDrawArrays(GL_TRIANGLES, 0, 36);

        openglFunctions->glUniformMatrix4fv(modelLoc, 1, false, &trans[0][0]);


        painter.beginNativePainting();
        painter.drawText(rect, std::to_string(1.0/dt).c_str());
        painter.endNativePainting();

        for (auto& it: inputsDown)
            it.second = false;
        app.processEvents();
        context->makeCurrent(&window);
        openglFunctions->glFinish();
        context->swapBuffers(&window);

    }

    app.quit();
}
