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
#include <UniformRigidBody.h>
#include <Collider.h>
#include <PhysicsWorld.h>


float dt;
QOpenGLFunctions_4_5_Core* openglFunctions;
Shader* modelShader;
Shader* gridShader;
Mesh lineMesh;

static void drawLine(glm::vec3 from, glm::vec3 to)
{
    glm::vec3 dir = to-from;
    float dist = glm::length(dir);
    dir = glm::normalize(dir);
    float theta = glm::acos(dir.x);
    theta = dir.z>0?-theta:theta;
    float psi = glm::asin(dir.y);
    glm::mat4 trans(1.0f);
    trans = glm::translate(trans, from);
    trans = glm::rotate(trans, theta, glm::vec3(0,1,0));
    trans = glm::rotate(trans, psi, glm::vec3(0,0,1));
    trans = glm::scale(trans, glm::vec3(dist, 1, 1));
    gridShader->setMat4("model", trans);
    lineMesh.draw(*gridShader);
}



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
    lineMesh = Mesh::createLine();
    lineMesh.setColor(glm::vec3(0,0,1));
    Mesh pointMesh = Mesh::createSphere();
    pointMesh.setColor(glm::vec3(0.3,0.45,0.3));
    Entity point(pointMesh);
    point.setScale(glm::vec3(0.2,0.2,0.2));

    Entity plane = createGridedPlaneEntity(10);

    Entity unitDirs = createUnitDirs();
    Entity cube = createBoundedCubeEntity();
    Entity cone = createBoundedConeEntity();
    Entity sphere = createBoundedSphereEntity();
    Entity cylinder = createBoundedCylinderEntity();
    Entity capsule = createBoundedCapsuleEntity();
    Entity arrow = createArrow();
   // cube.addChild(unitDirs);

    //Entity sphere = createBoundedSphereEntity();
    float mass = 1.0f;
    float radius = 1.0f;
    float inertia = (2.0f/5.0f)*mass*radius*radius;
    UniformRigidBody rb(mass, inertia);
    UniformRigidBody otherRb(mass, inertia);
    // SphereBody otherRb(mass, 0.5f);
    CubeCollider collider(glm::vec3(0.5f,0.5f,0.5f));
    CubeCollider otherCollider(glm::vec3(10.0f,0.1f,10.0f));
    //CubeCollider otherCollider(glm::vec3(0.5f,0.5f,0.5f));

    collider.rb = &rb;
    otherCollider.rb = &otherRb;
    std::vector<Collider*> colliders = {&collider, &otherCollider};
    PhysicsWorld world(&colliders, glm::vec3(0, -10.0f, 0));

    PlaneCollider p1(glm::vec3(-10, 0, -10), glm::vec3(-10, 0, 10), glm::vec3(10, 0, 10));
    PlaneCollider p2(glm::vec3(-10, 0, -10), glm::vec3(10, 0, 10), glm::vec3(10, 0, -10));
    world.colliders.push_back(&p1);
    world.colliders.push_back(&p2);


    rb.position = glm::vec3(0, 5, 0);
    rb.dynamic = true;
    otherRb.position = glm::vec3(0,-0.1f,0);
    otherRb.dynamic = false;
    rb.rotation = glm::quat(glm::vec3(PI/3.0f,0.0f, PI/3.0f));
    //rb.rotation = glm::quat(glm::vec3(PI/3.0f,0.0f, PI/3.0f));

    //otherRb.rotation = glm::quat(glm::vec3(0.0f, PI/4.0f, 0));

    QElapsedTimer elapsedTimer;
    elapsedTimer.start();
    long time = elapsedTimer.nsecsElapsed();
    while(window.shouldRun())
    {

        long timeNow = elapsedTimer.nsecsElapsed();
        dt = elapsedTimer.nsecsElapsed()/1000000000.0f;
        elapsedTimer.restart();

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
        //rb.setVelocity(glm::vec3(0,0,0));
        if(window.getKey(Qt::Key_Right))
        {
            //rb.setVelocity(1.0f*cam.getRight());
            rb.addForce(2.0f*cam.getRight());
        }
        if(window.getKey(Qt::Key_Left))
        {
            //rb.setVelocity(-1.0f*cam.getRight());
            rb.addForce(-2.0f*cam.getRight());
        }
        if(window.getKey(Qt::Key_E))
        {
            rb.setVelocity(1.0f*glm::vec3(0,1,0));
        }
        if(window.getKey(Qt::Key_Q))
        {
            rb.setVelocity(-1.0f*glm::vec3(0,1,0));
        }
        if(window.getKey(Qt::Key_Up))
        {
            //rb.setVelocity(glm::cross(glm::vec3(0,1,0), cam.getRight()));
            rb.addForce(glm::cross(glm::vec3(0,2,0), cam.getRight()));

        }
        if(window.getKey(Qt::Key_Down))
        {
            //rb.setVelocity(glm::cross(glm::vec3(0,-1,0), cam.getRight()));
            rb.addForce(glm::cross(glm::vec3(0,-2,0), cam.getRight()));

        }
        if(window.getGetDown(Qt::Key_Space))
        {
            rb.addForce(glm::vec3(0,800,0));
        }
        if(window.getGetDown(Qt::Key_R))
        {
            rb.setVelocity(glm::vec3(0,0,0));
            rb.position = glm::vec3(0,2,0);
            rb.setAngularVelocity(glm::vec3(0,0,0));

            otherRb.setVelocity(glm::vec3(0,0,0));
            otherRb.position = glm::vec3(0,2,2);
            otherRb.setAngularVelocity(glm::vec3(0,0,0));
        }
        if(window.getGetDown(Qt::Key_1))
        {
            rb.setVelocity(glm::vec3(0,0,0));
        }
        cam.smoothUpdateView();
        //cam.updateView();
        modelShader->setMat4("view", cam.view);
        gridShader->setMat4("view", cam.view);

        //unitDirs.rotate(glm::quat(glm::vec3(0,dt,0)));
        //unitDirs.draw();

        //world.stepWorld(0.0009f);
        world.stepWorld(dt);

        // cube.meshes[1].setColor(glm::vec3(1,0,0));
        //rb.setAngularVelocity(glm::vec3(0.5,0.25,0.75));
        //otherRb.setAngularVelocity(glm::vec3(0,1,0));

        if(collider.collisionDetected)
        {
            cube.meshes[1].setColor(glm::vec3(1,0,0));

            for(int i =0;i<world.contactInfo.points.size();i++)
            {
                point.setPosition(world.contactInfo.points[i]);
                point.draw();

            }
             drawLine(rb.position, rb.position+2.0f*world.contactInfo.normal);

        }
        else
        {
            cube.meshes[1].setColor(glm::vec3(0,1,0));
        }
        cube.setPosition(rb.position);
        cube.setRotation(rb.rotation);
        cube.setScale(collider.scale);
        cube.draw();

        //arrow.draw();
        //        collider.updateContactVerts(CubeCollider::ContactDir::BACK);
        //        for(int i =0; i<4; i++)
        //        {

        //            point.setPosition(collider.contactVertBuffer[i]);
        //            point.draw();
        //        }
        //        collider.updateContactEdges(CubeCollider::ContactDir::BACK);
        //        for(int i =0;i<4;i++)
        //        {
        //            point.setPosition(collider.contactEdgeBuffer[i]);
        //            point.draw();
        //        }
        cube.setPosition(otherRb.position);
        cube.setRotation(otherRb.rotation);
        cube.setScale(otherCollider.scale);
        cube.draw();


        plane.draw();
        // RayCastData rcd;
        ///if(world.Raycast(rb.position, 5.0f*world.info.normal, rcd))

        //drawLine(glm::vec3(0, 5.0, 0), glm::vec3(5, -5.0, -5));
        //glm::mat4 ok = glm::translate(trans, glm::vec3(0, 2, 0));



        std::string vector;
        //vector = std::to_string(rb.getLocalYAxis().x) + ", " + std::to_string(rb.getLocalYAxis().y) + ", "+ std::to_string(rb.getLocalYAxis().z);
        openglFunctions->glDisable(GL_DEPTH_TEST);
        QPainter painter(paintDevice);
        painter.setWorldMatrixEnabled(false);
        painter.setPen(Qt::white);
        painter.setFont(QFont("Arial", 12));
        QRectF rect(0.0f,0.0f,paintDevice->size().width(), paintDevice->size().height());
        painter.beginNativePainting();
        //painter.drawText(rect, std::to_string(1.0/dt).c_str());
        //painter.drawText(rect, vector.c_str());
        painter.endNativePainting();

        window.resetInputs();
        app.processEvents();
       // openglFunctions->glFinish();
        context->makeCurrent(&window);
        context->swapBuffers(&window);
    }

    app.quit();
    return 0;
}
