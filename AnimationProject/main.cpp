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
#include <QOpenGLFunctions_4_4_Core>
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
extern QOpenGLFunctions_4_4_Core* openglFunctions;

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
    format.setMinorVersion(4);
    format.setSwapInterval(0);
    format.setProfile(QSurfaceFormat::CoreProfile);

    MainWindow window;
    window.setTitle("Animation Project");
    window.setFormat(format);
    window.setSurfaceType(QWindow::OpenGLSurface);
    window.resize(1024, 768);
    window.setKeyboardGrabEnabled(true);
    window.show();
    CloseEventFilter closeFilter(&window);
    window.installEventFilter(&closeFilter);

    if(!window.InitializeContext())
        return -1;
    if(!window.InitializeOpenGLFunctions())
        return -1;

    QRectF rect(0.0f,0.0f,window.width(), window.height());
    openglFunctions->glViewport(0, 0, window.width() * window.devicePixelRatio(), window.height() * window.devicePixelRatio());
    openglFunctions->glEnable(GL_DEPTH_TEST);
    openglFunctions->glEnable(GL_CULL_FACE);

    Shader modelShader("model.vert", "model.frag");


    float aspect = (float)window.width()/window.height();
    qDebug() << "Aspect: " << aspect;
    glm::vec3 euler(0,0,0);
    glm::mat4 trans(1.0f);
    modelShader.insertUniform("model");
    modelShader.insertUniform("view");
    modelShader.insertUniform("projection");
    modelShader.insertUniform("color");
    glm::mat4 projection = glm::perspective((float)PI*0.33f, (float)window.width()/window.height(), 0.1f, 100.0f);

    Camera cam(glm::vec3(0,2,5));
    cam.updateView();

    modelShader.setMat4("model", trans);
    modelShader.setMat4("view", cam.view);
    modelShader.setMat4("projection", projection);


    Mesh mesh = Mesh::createCube();
    mesh.setColor(glm::vec3(1, 1, 0));

    QElapsedTimer timer;
    timer.start();

    glm::quat q(glm::vec3(0,0,0));
    while(window.shouldRun())
    {
        window.context->makeCurrent(&window);
        dt = timer.nsecsElapsed()/1000000000.0;
        timer.restart();

       if(window.shouldResize())
       {
           openglFunctions->glViewport(0, 0, window.width()* window.devicePixelRatio(), window.height()* window.devicePixelRatio());
           rect = QRectF(0.0f,0.0f,window.width(), window.height());
           window.paintDevice->setSize(window.size() * window.devicePixelRatio());
           window.paintDevice->setDevicePixelRatio(window.devicePixelRatio());
           delete window.painter;
           window.painter = new QPainter(window.paintDevice);
           projection = glm::perspective((float)PI*0.33f, (float)window.width()/window.height(), 0.1f, 100.0f);
           modelShader.setMat4("projection", projection);
       }
        openglFunctions->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

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
        q = glm::rotate(q, (float)dt, glm::vec3(0,1,0));
        //glm::mat4 rot = glm::rotate(trans, glm::length(euler), euler);
        glm::mat4 rot = glm::toMat4(q);
        modelShader.setMat4("model", rot);
        modelShader.setMat4("view", cam.view);
        mesh.draw(modelShader);

        window.painter->beginNativePainting();
        window.painter->drawText(rect, std::to_string(1.0/dt).c_str());
        window.painter->endNativePainting();


        window.resetInputs();


        window.context->swapBuffers(&window);
        //openglFunctions->glFinish();
        app.processEvents();
    }

    app.quit();
    return 0;
}
