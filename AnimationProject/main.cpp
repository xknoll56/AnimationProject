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
Mesh lineMesh;

enum ColliderType
{
    SPHERE = 0,
    CUBE = 1,
    PLANE = 2,
    NONE = 3
};

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



struct RayCastData
{
    glm::vec3 point;
    glm::vec3 normal;
    float length;
};


struct UniformRigidBody
{

    //constants
    const float mass;
    const float inertia;
    float massInv;
    float inertiaInv;
    float elasticity = 0.25f;

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
    std::vector<glm::vec3> appliedForces;
    std::vector<glm::vec3> appliedTorques;
    bool applyForce = false;
    bool applyTorque = false;

    ColliderType type;

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
        type = ColliderType::NONE;
    }

    virtual ~UniformRigidBody()
    {

    }

    void addForce(const glm::vec3& force)
    {
        appliedForces.push_back(force);
        applyForce = true;
    }

    void addTorque(const glm::vec3& torque)
    {
        appliedTorques.push_back(torque);
        applyTorque = true;
    }

    void setVelocity(const glm::vec3& velocity)
    {
        linearMomentum = velocity*mass;
    }

    void setAngularVelocity(const glm::vec3& angularVelocity)
    {
        angularMomentum = inertia*angularVelocity;
    }

    glm::vec3 peekNextPosition(float dt)
    {
        glm::vec3 tempMomentum = linearMomentum+force*dt;
        return position + massInv*tempMomentum*dt;
    }
    void stepQuantities(float dt)
    {

        if(applyForce)
        {
            for(glm::vec3 force: appliedForces)
                linearMomentum+=dt*force;
            applyForce = false;
            appliedForces.clear();
        }
        if(applyTorque)
        {
            for(glm::vec3 torque: appliedTorques)
                angularMomentum+=dt*torque;
            applyTorque = false;
            appliedTorques.clear();
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

struct Collider
{
    ColliderType type;
    UniformRigidBody* rb = nullptr;

    virtual ~Collider()
    {

    }
};

struct PlaneCollider: public Collider
{
    glm::vec3 normal;
    glm::vec3 point1, point2, point3;

    PlaneCollider(const glm::vec3& point1, const glm::vec3& point2, const glm::vec3& point3)
    {

        this->point1 = point1;
        this->point2 = point2;
        this->point3 = point3;
        this->normal = glm::normalize(glm::cross(point2-point1, point3-point1));
        type = ColliderType::PLANE;
    }
};


struct SphereCollider: public Collider
{
    float radius;

    SphereCollider(const float radius)
    {
        this->radius = radius;
        type = ColliderType::SPHERE;
    }

    SphereCollider(const float radius, UniformRigidBody* const rb)
    {
        this->radius = radius;
        type = ColliderType::SPHERE;
        this->rb = rb;
    }
};


struct PhysicsWorld
{
private:
    //PlaneCollider standardPlane;
public:
    //std::vector<UniformRigidBody*> bodies;
    glm::vec3 gravity;
    std::vector<Collider*> colliders;
    RayCastData rcd;
    float friction = 25.0f;
    float restitutionSlope = 0.085f;
    float restitutionIntersect = 0.4f;

    bool Raycast(const glm::vec3& start, const glm::vec3& dir, RayCastData& data, Collider* collider)
    {
        switch(collider->type)
        {
        case ColliderType::PLANE:
            PlaneCollider* pc = dynamic_cast<PlaneCollider*>(collider);
            float d = glm::dot((pc->point1-start), pc->normal)/glm::dot(dir, pc->normal);
            glm::vec3 pos = start+dir*d;
            data.length = d;
            data.point = pos;
            data.normal = pc->normal;
            glm::vec3 dir1 = glm::normalize(glm::cross(pos-pc->point1, pc->point2-pc->point1));
            glm::vec3 dir2 = glm::normalize(glm::cross(pos-pc->point2, pc->point3-pc->point2));
            glm::vec3 dir3 = glm::normalize(glm::cross(pos-pc->point3, pc->point1-pc->point3));
            if(glm::all(glm::isnan(dir1))||glm::all(glm::isnan(dir2))||glm::all(glm::isnan(dir3)))
                return true;
            float mag1 = glm::dot(dir1, dir2);
            float mag2 = glm::dot(dir1, dir3);
            float mag3 = glm::dot(dir2, dir3);
            if(glm::epsilonEqual(mag1, mag2, 0.1f) && glm::epsilonEqual(mag2, mag3, 0.1f))
                return true;
            break;
        }
        return false;
    }

    bool Raycast(const glm::vec3& start, const glm::vec3& dir, RayCastData& data)
    {
        for(Collider* collider: colliders)
        {
            switch(collider->type)
            {
            case ColliderType::PLANE:
                PlaneCollider* pc = dynamic_cast<PlaneCollider*>(collider);
                float d = glm::dot((pc->point1-start), pc->normal)/glm::dot(dir, pc->normal);
                glm::vec3 pos = start+dir*d;
                data.length = d;
                data.point = pos;
                data.normal = pc->normal;
                glm::vec3 dir1 = glm::normalize(glm::cross(pos-pc->point1, pc->point2-pc->point1));
                glm::vec3 dir2 = glm::normalize(glm::cross(pos-pc->point2, pc->point3-pc->point2));
                glm::vec3 dir3 = glm::normalize(glm::cross(pos-pc->point3, pc->point1-pc->point3));
                if(glm::all(glm::isnan(dir1))||glm::all(glm::isnan(dir2))||glm::all(glm::isnan(dir3)))
                    return true;
                float mag1 = glm::dot(dir1, dir2);
                float mag2 = glm::dot(dir1, dir3);
                float mag3 = glm::dot(dir2, dir3);
                if(glm::epsilonEqual(mag1, mag2, 0.1f) && glm::epsilonEqual(mag2, mag3, 0.1f))
                    return true;
                break;
            }
        }
        return false;
    }

    PhysicsWorld(std::vector<Collider*>* colliders, glm::vec3 gravity)
    {
        this->gravity = gravity;
        this->colliders.reserve(colliders->size());
        for(auto& collider: *colliders)
        {
            this->colliders.push_back(collider);
        }
        for(auto& collider: *colliders)
        {
            if(collider->rb!=nullptr)
                collider->rb->force += collider->rb->mass*gravity;
        }

    }

    PhysicsWorld(std::vector<Collider*>* colliders)
    {
        gravity = glm::vec3(0, -9.81f, 0);
        this->colliders.reserve(colliders->size());
        for(auto& collider: *colliders)
        {
            this->colliders.push_back(collider);
        }
        for(auto& collider: *colliders)
        {
            if(collider->rb!=nullptr)
                collider->rb->force += collider->rb->mass*gravity;
        }

    }
    void checkForCollisions(float dt)
    {
        for(auto& collider: colliders)
        {
            switch(collider->type)
            {
            case ColliderType::SPHERE:
                SphereCollider* sphere = dynamic_cast<SphereCollider*>(collider);
                spherePlaneCollision(dt, sphere);
                for(auto& other: colliders)
                {
                    if(other!=collider)
                    {
                        switch(other->type)
                        {
                        case ColliderType::SPHERE:
                            SphereCollider* otherSphere = dynamic_cast<SphereCollider*>(other);
                            sphereSphereCollision(dt, sphere, otherSphere);
                            break;
                        }
                    }
                }
                break;
            }
        }
    }

    void sphereSphereCollision(float dt, SphereCollider* sphere, SphereCollider* other)
    {
        glm::vec3 dp = other->rb->position-sphere->rb->position;
        float lSquared = glm::length2(dp);
        float minDist = other->radius+sphere->radius;
        if(lSquared<=minDist*minDist)
        {
            glm::vec3 relativeMomentum = sphere->rb->linearMomentum-other->rb->linearMomentum;
            dp = glm::normalize(dp);
            float mag = glm::dot(dp, relativeMomentum);
            if(mag>0)
            {
                glm::vec3 relativeMomentumNorm = mag*dp;
                other->rb->addForce((1.0f/dt)*relativeMomentumNorm);
            }
        }
    }
    void spherePlaneCollision(float dt, SphereCollider* sphere)
    {

        if(Raycast(sphere->rb->position, glm::vec3(0,-1,0), rcd))
        {
            if(rcd.length<=sphere->radius)
            {
                float velNorm = glm::dot(glm::vec3(0,-1,0), sphere->rb->velocity);
                float sMax = restitutionSlope*-gravity.y+restitutionIntersect;
                if(velNorm<sMax)
                {
                    sphere->rb->position = glm::vec3(0,1,0)*sphere->radius+rcd.point;
                    sphere->rb->linearMomentum = glm::cross(glm::cross(glm::vec3(0,1,0), sphere->rb->linearMomentum), glm::vec3(0,1,0));
                    sphere->rb->setAngularVelocity(glm::cross(glm::vec3(0,1,0),sphere->rb->velocity/sphere->radius));
                    sphere->rb->addForce(-sphere->rb->velocity);
                }
                else
                {
                    sphere->rb->position = glm::vec3(sphere->rb->position.x, 0.05f+sphere->radius, sphere->rb->position.z);
                    //sphere->addForce(glm::vec3(0, -2.0f*(sphere->velocity.y)/dt, 0));
                    glm::vec3 pNorm = glm::dot(glm::vec3(0,1,0), sphere->rb->linearMomentum)*sphere->rb->elasticity*glm::vec3(0,1,0);
                    glm::vec3 pPerp = glm::cross(glm::cross(glm::vec3(0,1,0), sphere->rb->linearMomentum), glm::vec3(0,1,0));
                    sphere->rb->linearMomentum = pPerp-pNorm;
                    sphere->rb->addTorque(glm::cross(glm::vec3(0,1,0),friction*pPerp));

                }
            }
        }
    }
    void updateQuantities(float dt)
    {
        for(const auto& collider: colliders)
        {
            if(collider->rb!=nullptr)
                collider->rb->stepQuantities(dt);
        }
    }
    void stepWorld(float dt)
    {
        updateQuantities(dt);
        checkForCollisions(dt);
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

    Entity plane = createGridedPlaneEntity(10);

    Entity unitDirs = createUnitDirs();
    Entity cube = createBoundedCubeEntity();
    Entity cone = createBoundedConeEntity();
    Entity sphere = createBoundedSphereEntity();
    Entity cylinder = createBoundedCylinderEntity();
    Entity capsule = createBoundedCapsuleEntity();
    cube.setPosition(glm::vec3(3,0,0));
    capsule.setPosition(glm::vec3(-3, 0, 0));
    sphere.setPosition(glm::vec3(0,0,3));
    cylinder.setPosition(glm::vec3(0,0,-3));
    unitDirs.addChild(cube);
    unitDirs.addChild(capsule);
    unitDirs.addChild(sphere);
    unitDirs.addChild(cylinder);

    unitDirs.setPosition(glm::vec3(0,3, 0));

    //Entity sphere = createBoundedSphereEntity();
    float mass = 1.0f;
    float radius = 1.0f;
    float inertia = (2.0f/5.0f)*mass*radius*radius;
    UniformRigidBody rb(mass, inertia);
    // SphereBody otherRb(mass, 0.5f);
    SphereCollider collider(0.5f);
    collider.rb = &rb;
    std::vector<SphereCollider> scs;
    std::vector<UniformRigidBody> rbs;
    std::vector<Collider*> colliders;
    scs.reserve(20);
    rbs.reserve(20);
    colliders.reserve(20);
    colliders.push_back(&collider);
    for(int i=0; i<15; i++)
    {
        rbs.push_back(UniformRigidBody(mass, inertia));
        rbs[rbs.size()-1].position = glm::vec3(7-i, 5, -3);
        rbs[rbs.size()-1].linearMomentum  = glm::vec3(0, 0.0f, 0);

        scs.push_back(SphereCollider(0.5f));
        colliders.push_back(&scs[scs.size()-1]);
        colliders[colliders.size()-1]->rb = &rbs[rbs.size()-1];

    }
//    for(auto it = rbs.begin();it!=rbs.end();it++)
//    {
//        SphereCollider sc(0.5f);
//        sc.rb = &(*it);
//        scs.push_back(sc);
//    }
//    for(auto it = scs.begin();it!=scs.end();it++)
//    {
//        colliders.push_back(&(*it));
//    }
    PhysicsWorld world(&colliders, glm::vec3(0, -1.5f, 0));

    PlaneCollider p1(glm::vec3(-10, 0, -10), glm::vec3(-10, 0, 10), glm::vec3(10, 0, 10));
    PlaneCollider p2(glm::vec3(-10, 0, -10), glm::vec3(10, 0, 10), glm::vec3(10, 0, -10));
    world.colliders.push_back(&p1);
    world.colliders.push_back(&p2);


    rb.position = glm::vec3(0, 5, 0);

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
        if(window.getKey(Qt::Key_Right))
        {
            rb.addForce(5.0f*cam.getRight());
        }
        if(window.getKey(Qt::Key_Left))
        {
            rb.addForce(-5.0f*cam.getRight());
        }
        if(window.getKey(Qt::Key_Up))
        {
            rb.addForce(glm::cross(glm::vec3(0,5,0), cam.getRight()));
        }
        if(window.getKey(Qt::Key_Down))
        {
            rb.addForce(glm::cross(glm::vec3(0,-5,0), cam.getRight()));
        }
        if(window.getGetDown(Qt::Key_Space))
        {
            rb.addForce(glm::vec3(0,800,0));
        }
        if(window.getGetDown(Qt::Key_R))
        {
            rb.setVelocity(glm::vec3(0,0,0));
            rb.position = glm::vec3(0,5,0);
        }
        cam.smoothUpdateView();
        //cam.updateView();
        modelShader->setMat4("view", cam.view);
        gridShader->setMat4("view", cam.view);

        //unitDirs.rotate(glm::quat(glm::vec3(0,dt,0)));
        //unitDirs.draw();

        world.stepWorld(dt);

        sphere.meshes[1].setColor(glm::vec3(1,0,0));
        sphere.setPosition(rb.position);
        sphere.setRotation(rb.rotation);
        sphere.draw();


        sphere.meshes[1].setColor(glm::vec3(0,1,0));
        for(auto& rb : rbs)
        {
            sphere.setPosition(rb.position);
            sphere.setRotation(rb.rotation);
            sphere.draw();
        }


        plane.draw();
        RayCastData rcd;
        if(world.Raycast(rb.position, glm::vec3(0,-1,0), rcd))
            drawLine(rb.position, rcd.point);
        //drawLine(glm::vec3(0, 5.0, 0), glm::vec3(5, -5.0, -5));
        //glm::mat4 ok = glm::translate(trans, glm::vec3(0, 2, 0));




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
