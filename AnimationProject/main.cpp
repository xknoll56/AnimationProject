#define QT_ONSCREEN_PAINT
#define PI 3.14159265359f

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
#include <glm/common.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <MainWindow.h>
#include <Camera.h>
#include <Shader.h>


double dt;
QOpenGLFunctions_3_3_Core* openglFunctions;



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



int main(int argc, char *argv[])
{
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

    openglFunctions = context->versionFunctions<QOpenGLFunctions_3_3_Core>();
    if(!openglFunctions)
    {
        qDebug("Could not obtain required version of opengl");
        app.exit();
    }
    openglFunctions->initializeOpenGLFunctions();

    openglFunctions->glViewport(0, 0, window.width() * window.devicePixelRatio(), window.height() * window.devicePixelRatio());
    openglFunctions->glEnable(GL_DEPTH_TEST);
    openglFunctions->glEnable(GL_CULL_FACE);

    Shader modelShader("model.vert", "model.frag");


    glm::vec3 euler(0,0,0);
    glm::mat4 trans(1.0f);
    modelShader.insertUniform("model");
    modelShader.insertUniform("view");
    modelShader.insertUniform("projection");
    glm::mat4 projection = glm::perspective((float)PI*0.33f, (float)window.devicePixelRatio(), 0.1f, 100.0f);

    Camera cam(glm::vec3(0,2,5));
    cam.updateView();

    modelShader.setMat4("model", trans);
    modelShader.setMat4("view", cam.view);
    modelShader.setMat4("projection", projection);

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
        modelShader.use();
        if(window.inputs[Qt::Key_A])
        {
            cam.translateRight(-(float)dt);
        }
        if(window.inputs[Qt::Key_D])
        {
            cam.translateRight((float)dt);
        }
        if(window.inputs[Qt::Key_W])
        {
            cam.translateFwd(-(float)dt);
        }
        if(window.inputs[Qt::Key_S])
        {
            cam.translateFwd((float)dt);
        }
        if(window.inputs[Qt::MouseButton::LeftButton])
        {
            QPointF deltaPos = QCursor::pos()-window.mousePos;
            window.mousePos = QCursor::pos();
            cam.rotateYaw(-(float)dt*deltaPos.x());
            cam.rotatePitch(-(float)dt*deltaPos.y());
        }
        cam.updateView();
        euler += glm::vec3(dt, 0, 0);
        q = glm::rotate(q, (float)dt, glm::vec3(0,1,0));
        //glm::mat4 rot = glm::rotate(trans, glm::length(euler), euler);
        glm::mat4 rot = glm::toMat4(q);
        modelShader.setMat4("model", rot);
        modelShader.setMat4("view", cam.view);

        openglFunctions->glBindVertexArray(VAO); // seeing as we only have a single VAO there's no need to bind it every time, but we'll do so to keep things a bit more organized
        openglFunctions->glDrawArrays(GL_TRIANGLES, 0, 36);

        painter.beginNativePainting();
        painter.drawText(rect, std::to_string(1.0/dt).c_str());
        painter.endNativePainting();


        window.resetInputs();
        app.processEvents();
        context->makeCurrent(&window);
        openglFunctions->glFinish();
        context->swapBuffers(&window);

    }

    app.quit();
    return 0;
}
