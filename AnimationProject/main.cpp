#define QT_ONSCREEN_PAINT
#define PI 3.14159265359f

#include <QApplication>
#include <QSurfaceFormat>
#include <QWindow>
#include <QObject>
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
#include <Mesh.h>


double dt;
QOpenGLFunctions_4_5_Core* openglFunctions;

class CloseEventFilter : public QObject
{
public:
     CloseEventFilter(QObject *parent) : QObject(parent) {}
     MainWindow* window;

protected:
     bool eventFilter(QObject *obj, QEvent *event)
     {
          if (event->type() == QEvent::Close)
          {
               exit(EXIT_SUCCESS);
          }

          return QObject::eventFilter(obj, event);
     }

};

int main(int argc, char *argv[])
{
    QGuiApplication app(argc, argv);

    QSurfaceFormat format;
    format.setSamples(4);
    format.setDepthBufferSize(24);
    format.setMajorVersion(4);
    format.setMinorVersion(5);
    format.setSwapInterval(0);
    format.setProfile(QSurfaceFormat::CoreProfile);

    MainWindow window;
    window.setTitle("Animation Project");
    window.setFormat(format);
    window.setSurfaceType(QWindow::OpenGLSurface);
    window.resize(1280, 720);
    window.setKeyboardGrabEnabled(true);
    window.show();
    CloseEventFilter closeFilter(&window);
    window.installEventFilter(&closeFilter);


    QOpenGLContext* context = new QOpenGLContext(&window);
    context->setFormat(window.requestedFormat());
    context->create();
    context->makeCurrent(&window);

    //app.processEvents();
    QOpenGLPaintDevice* paintDevice = new QOpenGLPaintDevice;
    paintDevice->setSize(window.size() * window.devicePixelRatio());
    paintDevice->setDevicePixelRatio(window.devicePixelRatio());

    //painter->setWorldMatrixEnabled(false);

    openglFunctions = context->versionFunctions<QOpenGLFunctions_4_5_Core>();
    if(!openglFunctions)
    {
        qDebug("Could not obtain required version of opengl");
        app.exit();
    }
    openglFunctions->initializeOpenGLFunctions();



    window.openglInitialized = true;
    openglFunctions->glViewport(0, 0, window.width() * window.devicePixelRatio(), window.height() * window.devicePixelRatio());
    openglFunctions->glEnable(GL_DEPTH_TEST);
    openglFunctions->glEnable(GL_CULL_FACE);
    openglFunctions->glDisable(GL_LIGHTING);

    Shader modelShader("model.vert", "model.frag");
    Shader gridShader("grid.vert", "grid.frag");


    glm::vec3 euler(0,0,0);
    glm::mat4 trans(1.0f);
    modelShader.insertUniform("model");
    modelShader.insertUniform("view");
    modelShader.insertUniform("projection");
    modelShader.insertUniform("color");
    modelShader.insertUniform("lightDir");
    gridShader.insertUniform("model");
    gridShader.insertUniform("view");
    gridShader.insertUniform("projection");
    gridShader.insertUniform("color");
    //modelShader.setVec3("color", glm::vec3(1,1,1));
    glm::mat4 projection = glm::perspective((float)PI*0.33f, (float)window.width()/window.height(), 0.1f, 100.0f);

    Camera cam(glm::vec3(0,2,5));
    cam.updateView();

    modelShader.setVec3("lightDir", glm::vec3(-0.5f, -1.0f, -0.75));
    modelShader.setMat4("model", trans);
    modelShader.setMat4("view", cam.view);
    modelShader.setMat4("projection", projection);
    gridShader.setMat4("model", trans);
    gridShader.setMat4("view", cam.view);
    gridShader.setMat4("projection", projection);

    Mesh::initializeStaticArrays();
    //Mesh mesh = Mesh::createCube();
    Mesh plane = Mesh::createPlane();
    Mesh cubeGrid = Mesh::createBoundingBox();
    Mesh gridMesh = Mesh::createGrid(10);


    Mesh sphereMesh = Mesh::createSphere();
    Mesh boundingSphere = Mesh::createBoundingSphere();
    Mesh cylinderMesh = Mesh::createCylinder();
    Mesh boundingCylinder = Mesh::createBoundingCylinder();
    Mesh cubeMesh = Mesh::createCube();
    Mesh boundingCube = Mesh::createBoundingBox();

    QElapsedTimer timer;
    timer.start();
    glm::quat q(glm::vec3(0,0,0));

    while(window.shouldRun())
    {

        dt = timer.nsecsElapsed()/1000000000.0;
        timer.restart();

        openglFunctions->glEnable(GL_DEPTH_TEST);
        openglFunctions->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

//        if(window.windowResized())
//        {
//            paintDevice->setSize(window.size() * window.devicePixelRatio());
//            paintDevice->setDevicePixelRatio(window.devicePixelRatio());
//            projection = glm::perspective((float)PI*0.33f, (float)window.width()/window.height(), 0.1f, 100.0f);
//            modelShader.setMat4("projection", projection);
//        }

        if(window.getKey(Qt::Key_A))
        {
            cam.translateRight(-(float)dt);
        }
        if(window.getKey(Qt::Key_D))
        {
            cam.translateRight((float)dt);
        }
        if(window.getKey(Qt::Key_W))
        {
            cam.translateFwd(-(float)dt);
        }
        if(window.getKey(Qt::Key_S))
        {
            cam.translateFwd((float)dt);
        }
        if(window.getMouse(Qt::MouseButton::LeftButton))
        {
            QPointF deltaPos = QCursor::pos()-window.mousePos;
            window.mousePos = QCursor::pos();
            cam.rotateYaw(-(float)dt*deltaPos.x());
            cam.rotatePitch(-(float)dt*deltaPos.y());
        }
        cam.updateView();
        euler += glm::vec3(dt, 0, 0);
        //q = glm::rotate(q, (float)dt, glm::vec3(0,1,0));
        glm::mat4 s = glm::scale(trans, glm::vec3(4,0,4));
        glm::mat4 t = glm::translate(trans, glm::vec3(2.0f, 1.0f, 0));
        gridShader.setVec3("color", glm::vec3(0,1,0));
        modelShader.setMat4("view", cam.view);
        gridShader.setMat4("view", cam.view);

        modelShader.setMat4("model", t);
        cylinderMesh.draw(modelShader);
        gridShader.setMat4("model", t);
        boundingCylinder.draw();

        t = glm::translate(trans, glm::vec3(0, 0.5f, 0));
        modelShader.setMat4("model", t);
        cubeMesh.draw(modelShader);
        gridShader.setMat4("model", t);
        boundingCube.draw();

        t = glm::translate(trans, glm::vec3(-2.0f, 0.5f, 0));
        modelShader.setMat4("model", t);
        sphereMesh.draw(modelShader);
        gridShader.setMat4("model", t);
        boundingSphere.draw();


        s = glm::scale(trans, glm::vec3(20, 1, 20));
        modelShader.setMat4("model", s);
        plane.draw(modelShader);

        gridShader.setMat4("model", trans);
        gridShader.setVec3("color", glm::vec3(1.0f, 0, 0));
        gridMesh.draw();



        openglFunctions->glDisable(GL_DEPTH_TEST);
        QPainter painter(paintDevice);
        painter.setWorldMatrixEnabled(false);
        painter.setPen(Qt::white);
        painter.setFont(QFont("Arial", 12));
        QRectF rect(0.0f,0.0f,paintDevice->size().width(), paintDevice->size().height());
        painter.beginNativePainting();
        painter.drawText(rect, std::to_string(1.0/dt).c_str());
        painter.endNativePainting();

        window.resetInputs();
        app.processEvents();
        context->makeCurrent(&window);
        context->swapBuffers(&window);
        openglFunctions->glFinish();
    }

    app.quit();
    return 0;
}
