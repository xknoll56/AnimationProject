#include <Common.h>

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
#include <Entity.h>


float dt;
QOpenGLFunctions_4_5_Core* openglFunctions;
Shader* modelShader;
Shader* gridShader;

struct UniformRigidBody
{

    //constants
    const float mass;
    const float inertia;
    float massInv;
    float inertiaInv;

    //state variables
    glm::vec3 position;
    glm::quat rotation;
    glm::vec3 linearMomentum;
    glm::vec3 angularMomentum;

    //derived quantities
    glm::vec3 velocity;
    glm::vec3 angularVelocity;

    //known quantities
    glm::vec3 force;
    glm::vec3 torque;

    //applied force/torque will be applie for a single step
    glm::vec3 appliedForce;
    glm::vec3 appliedTorque;
    bool applyForce = false;
    bool applyTorque = false;

    UniformRigidBody(float _mass, float _inertia): mass(_mass), inertia(_inertia)
    {
        massInv = 1.0f/mass;
        inertiaInv = 1.0f/inertia;
        position = glm::vec3();
        rotation = glm::quat(glm::vec3(0,0,0));
        linearMomentum = glm::vec3();
        angularMomentum = glm::vec3();
        velocity = glm::vec3();
        angularVelocity = glm::vec3();
        force = glm::vec3();
        torque = glm::vec3();
        appliedForce = glm::vec3();
        appliedTorque = glm::vec3();
    }

    void addForce(const glm::vec3& force)
    {
        appliedForce = force;
        applyForce = true;
    }

    void addTorque(const glm::vec3& torque)
    {
        appliedTorque = torque;
        applyTorque = true;
    }

    void stepQuantities(float dt)
    {

        if(applyForce)
        {
            linearMomentum+=dt*appliedForce;
            applyForce = false;
        }
        if(applyTorque)
        {
            angularMomentum+=dt*appliedTorque;
            applyTorque = false;
        }
        angularMomentum += torque*dt;
        linearMomentum += force*dt;
        angularVelocity = inertiaInv*angularMomentum;
        velocity = massInv*linearMomentum;
        rotation+= dt*0.5f*glm::quat(0, angularVelocity.x, angularVelocity.y, angularVelocity.z)*rotation;
        rotation = glm::normalize(rotation);
        position+=dt*velocity;
    }
};

struct PhysicsWorld
{
    std::vector<UniformRigidBody*> bodies;

    void stepWorld(float dt)
    {
        for(const auto& body: bodies)
        {
            body->stepQuantities(dt);
        }
    }
};

//class RigidBody
//{
//private:
//    //constants
//    const float mass;
//    const glm::mat3 inertialTensor;
//    const glm::mat3 inertialTensorInverse;

//    //state variables
//    glm::vec3 position;
//    glm::quat rotation;
//    glm::vec3 linearMomentum;
//    glm::vec3 angularMomentum;

//    //derived quantities
//    glm::mat3 inertiaInverse;
//    glm::mat3 rotationMatrix;
//    glm::vec3 velocity;
//    glm::vec3 angularVelocity;

//    //known quantities
//    glm::vec3 force;
//    glm::vec3 torque;
//public:
//    void computeQuantities()
//    {
//        velocity = linearMomentum/mass;
//        inertiaInverse = rotationMatrix*inertialTensorInverse*glm::transpose(rotationMatrix);
//        angularVelocity = inertiaInverse*angularMomentum;
//        rotationMatrix = glm::toMat3(glm::normalize(rotation));
//    }


//};


int main(int argc, char *argv[])
{
    QGuiApplication app(argc, argv);

    QSurfaceFormat format;
    format.setSamples(4);
    format.setDepthBufferSize(24);
    format.setMajorVersion(4);
    format.setMinorVersion(5);
    format.setSwapInterval(0);
    format.setSwapBehavior(QSurfaceFormat::SwapBehavior::DefaultSwapBehavior);
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
    openglFunctions->glEnable(GL_LINE_SMOOTH);
    openglFunctions->glEnable(GL_LINE_WIDTH);
    openglFunctions->glLineWidth(2.5f);
    openglFunctions->glDisable(GL_LIGHTING);


    Shader modelShaderObj("model.vert", "model.frag");
    Shader gridShaderObj("grid.vert", "grid.frag");
    modelShader = &modelShaderObj;
    gridShader = &gridShaderObj;

    glm::vec3 euler(0,0,0);
    glm::mat4 trans(1.0f);
    modelShader->insertUniform("model");
    modelShader->insertUniform("view");
    modelShader->insertUniform("projection");
    modelShader->insertUniform("color");
    modelShader->insertUniform("lightDir");
    gridShader->insertUniform("model");
    gridShader->insertUniform("view");
    gridShader->insertUniform("projection");
    gridShader->insertUniform("color");
    //modelShader.setVec3("color", glm::vec3(1,1,1));
    glm::mat4 projection = glm::perspective((float)PI*0.33f, (float)window.width()/window.height(), 0.1f, 100.0f);

    Camera cam(glm::vec3(0,2,5));
    cam.updateView();

    modelShader->setVec3("lightDir", glm::vec3(-0.5f, -1.0f, -0.75));
    modelShader->setMat4("model", trans);
    modelShader->setMat4("view", cam.view);
    modelShader->setMat4("projection", projection);
    gridShader->setMat4("model", trans);
    gridShader->setMat4("view", cam.view);
    gridShader->setMat4("projection", projection);

    Mesh::initializeStaticArrays();

    Entity plane = createGridedPlaneEntity(10);

//    Entity unitDirs = createUnitDirs();
//    Entity cube = createBoundedCubeEntity();
//    Entity cone = createBoundedConeEntity();
//    Entity sphere = createBoundedSphereEntity();
//    Entity cylinder = createBoundedCylinderEntity();
//    Entity capsule = createBoundedCapsuleEntity();
//    cube.setPosition(glm::vec3(3,0,0));
//    capsule.setPosition(glm::vec3(-3, 0, 0));
//    sphere.setPosition(glm::vec3(0,0,3));
//    cylinder.setPosition(glm::vec3(0,0,-3));
//    unitDirs.addChild(cube);
//    unitDirs.addChild(capsule);
//    unitDirs.addChild(sphere);
//    unitDirs.addChild(cylinder);

//    unitDirs.setPosition(glm::vec3(0,3, 0));

    Entity sphere = createBoundedSphereEntity();
    float mass = 1.0f;
    float radius = 1.0f;
    float inertia = (2.0f/5.0f)*mass*radius*radius;
    UniformRigidBody rb(mass, inertia);
    PhysicsWorld world;
    world.bodies.push_back(&rb);


    rb.position = glm::vec3(0, 15, 0);

    QElapsedTimer elapsedTimer;
    elapsedTimer.start();
    long time = elapsedTimer.nsecsElapsed();
    //app.
    while(window.shouldRun())
    {

        long timeNow = elapsedTimer.nsecsElapsed();
        dt = (timeNow-time)/1000000000.0f;
        time = timeNow;

        openglFunctions->glEnable(GL_DEPTH_TEST);
        openglFunctions->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        if(window.windowResized())
        {
            paintDevice->setSize(window.size() * window.devicePixelRatio());
            paintDevice->setDevicePixelRatio(window.devicePixelRatio());
            projection = glm::perspective((float)PI*0.33f, (float)window.width()/window.height(), 0.1f, 100.0f);
            modelShader->setMat4("projection", projection);
            gridShader->setMat4("projection", projection);
        }

        if(window.getKey(Qt::Key_A))
        {
            cam.translateRight(-dt);
        }
        if(window.getKey(Qt::Key_D))
        {
            cam.translateRight(dt);
        }
        if(window.getKey(Qt::Key_W))
        {
            cam.translateFwd(-dt);
        }
        if(window.getKey(Qt::Key_S))
        {
            cam.translateFwd(dt);
        }
        if(window.getMouse(Qt::MouseButton::LeftButton))
        {
            QPointF deltaPos = QCursor::pos()-window.mousePos;
            window.mousePos = QCursor::pos();
            cam.smoothRotateYaw(-dt*deltaPos.x());
            cam.smoothRotatePitch(-dt*deltaPos.y());
//            cam.rotateYaw(-(float)dt*deltaPos.x());
//            cam.rotatePitch(-(float)dt*deltaPos.y());
        }
        if(window.getGetDown(Qt::Key_Space))
        {
            rb.addForce(glm::vec3(20, 0, 0));
        }
        cam.smoothUpdateView();
        //cam.updateView();
        modelShader->setMat4("view", cam.view);
        gridShader->setMat4("view", cam.view);

        world.stepWorld(dt);
        sphere.setPosition(rb.position);
        sphere.setRotation(rb.rotation);
        sphere.draw();


        plane.draw();


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
        openglFunctions->glFinish();
        context->makeCurrent(&window);
        context->swapBuffers(&window);
    }

    app.quit();
    return 0;
}
