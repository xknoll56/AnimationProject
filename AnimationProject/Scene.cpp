#include "Scene.h"
#include "Debug.h"
#include <QDebug>


extern Shader* modelShader;
extern Shader* gridShader;
extern MainWindow* gMainWindow;
extern QPaintDevice* gPaintDevice;
extern QOpenGLFunctions_4_5_Core* openglFunctions;



Scene::Scene()
{
    painter.setWorldMatrixEnabled(false);
    doUpdateConsole = true;
    consoleToggle = false;
    gMainWindow->toggleWriteEnable();
    //replys.push_back("");
    commands.push_back("");
    replys.push_back("Press Escape to start, or enter a command.\n");
    nearPlane = 0.1f;
    farPlane = 100.0f;
    aspectRatio = (float)gMainWindow->width()/gMainWindow->height();
    fov = (float)PI*0.33f;
    projection = glm::perspective(fov, aspectRatio, nearPlane, farPlane);
    glm::mat4 trans(1.0f);
    cam = Camera(glm::vec3(0,2,5));
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
    point = Entity(pointMesh);
    point.setScale(glm::vec3(0.2,0.2,0.2));

    plane = createGridedPlaneEntity(10);
    unitDirs = createUnitDirs();
    cube = createBoundedCubeEntity();
    cone = createBoundedConeEntity();
    sphere = createBoundedSphereEntity();
    cylinder = createBoundedCylinderEntity();
    capsule = createBoundedCapsuleEntity();
    arrow = createArrow();
}

void Scene::start()
{

}


void Scene::update(float dt)
{
    if(gMainWindow->getKeyDown(Qt::Key_Escape))
    {
        doUpdateConsole = true;
        consoleToggle = false;
        gMainWindow->toggleWriteEnable();
    }
    if(gMainWindow->getKey(Qt::Key_A))
    {
        cam.translateRight(-dt);
    }
    if(gMainWindow->getKey(Qt::Key_D))
    {
        cam.translateRight(dt);
    }
    if(gMainWindow->getKey(Qt::Key_W))
    {
        cam.translateFwd(-dt);
    }
    if(gMainWindow->getKey(Qt::Key_S))
    {
        cam.translateFwd(dt);
    }
    if(gMainWindow->getMouse(Qt::MouseButton::LeftButton))
    {
        gMainWindow->lockCursorState();
        //QPointF deltaPos = QCursor::pos()-gMainWindow->mousePos;
        QPointF deltaPos((float)QCursor::pos().x()-gMainWindow->position().x()-gMainWindow->width()/2.0f,
                         (float)QCursor::pos().y()-gMainWindow->position().y()-gMainWindow->height()/2.0f);
        //gMainWindow->mousePos = QCursor::pos();
        if(glm::abs(deltaPos.x())<50 && glm::abs(deltaPos.y())<50)
        {
        cam.rotateYaw(-dt*deltaPos.x());
        cam.rotatePitch(-dt*deltaPos.y());
        }
        QCursor::setPos(gMainWindow->position().x()+gMainWindow->width()/2,
                        gMainWindow->position().y()+gMainWindow->height()/2);
    }
    else
        gMainWindow->unlockCursorState();

    cam.updateView();
    modelShader->setMat4("view", cam.view);
    gridShader->setMat4("view", cam.view);

}

void Scene::drawCrosshair()
{
    openglFunctions->glDisable(GL_DEPTH_TEST);
    openglFunctions->glDisable(GL_CULL_FACE);

    painter.begin(gPaintDevice);
    painter.setBrush(Qt::red);
    painter.setPen(Qt::transparent);

    QRectF rect(gMainWindow->size().width()*(0.5f-0.005f),gMainWindow->size().height()*(0.5f-0.0025f),
                gMainWindow->size().width()*(0.01f), gMainWindow->size().height()*(0.005f));

    float adjustedLength = (float)gMainWindow->height()*0.005f/gMainWindow->width();
    float adjustedWidth = (float)gMainWindow->width()*0.01f/gMainWindow->height();

    QRectF rect2(gMainWindow->size().width()*(0.5f-adjustedLength/2),gMainWindow->size().height()*(0.5f-adjustedWidth/2),
                 gMainWindow->size().width()*(adjustedLength), gMainWindow->size().height()*(adjustedWidth));


    painter.drawRect(rect);
    painter.drawRect(rect2);
    painter.end();
}

void Scene::updateConsole(float dt)
{
    openglFunctions->glDisable(GL_DEPTH_TEST);
    openglFunctions->glDisable(GL_CULL_FACE);

    if(gMainWindow->getKeyDown(Qt::Key_Return))
    {
        QString text = gMainWindow->writtenText;
        if(text.compare("exit")==0)
            gMainWindow->quit();
        commands.push_front(text);
        for(QString& reply: replys)
            reply.append("\n\n");
        replys.push_front(console.ParseCommand(text)+"\n");
        for(QString& command: commands)
            command.append("\n\n");
        gMainWindow->clearText();
    }

    painter.begin(gPaintDevice);
    painter.setPen(Qt::green);
    painter.setBrush(Qt::black);
    painter.setOpacity(0.8f);
    painter.setFont(QFont("Courier", 12));

    QRectF rect(0.0f,gMainWindow->size().height()*3/4,gMainWindow->size().width(), gMainWindow->size().height()/4);


    painter.drawRect(rect);
    for(QString& command: commands)
        painter.drawText(rect, Qt::AlignBottom, command);
    painter.drawText(rect, Qt::AlignBottom, ">"+gMainWindow->writtenText);

    painter.setPen(Qt::red);
    for(QString& reply: replys)
        painter.drawText(rect, Qt::AlignBottom, reply);
    painter.end();


    if(gMainWindow->getKeyDown(Qt::Key_Escape) && consoleToggle)
    {
        doUpdateConsole = false;
        gMainWindow->clearText();
        gMainWindow->toggleWriteEnable();
    }
    else
        consoleToggle = true;
}

void Scene::selectRigidBody(PhysicsWorld& world)
{
    if(gMainWindow->getMouseDown(Qt::MouseButton::RightButton))
    {
        QPoint mouse = QCursor::pos(gMainWindow->screen())-gMainWindow->position();
        glm::vec3 rayNds((mouse.x()-(float)gMainWindow->width()/2)/(gMainWindow->width()/2),
                                   (-mouse.y()+(float)gMainWindow->height()/2)/(gMainWindow->height()/2), 2.0f);
        glm::vec4 rayClip(rayNds.x, rayNds.y, -1.0f, 1.0f);
        glm::vec4 rayEye = glm::inverse(projection)*rayClip;
        rayEye.z = -1.0f;
        rayEye.w = 0.0f;
        glm::vec3 rayWor = (glm::inverse(cam.getView())*rayEye);
        rayWor = glm::normalize(rayWor);

        RayCastData rcd;
        if(world.raycastAll(cam.getPosition(), rayWor, rcd))
        {
            selectedRb = rcd.collider->rb;
            console.rb = selectedRb;
        }
    }
}

void Scene::updateDraw(float dt)
{

}

void Scene::drawBoundedCollider(Collider& collider, const glm::vec3& baseColor, const glm::vec3& boundsColor)
{
    switch(collider.type)
    {
    case ColliderType::CUBE:
    {
        BoxCollider* cc = dynamic_cast<BoxCollider*>(&collider);
        if(cc)
        {
            cube.setScale(cc->scale);
            cube.setRotation(cc->rb->rotation);
            cube.setPosition(cc->rb->position);
            cube.meshes[0].setColor(baseColor);
            cube.meshes[1].setColor(boundsColor);
            cube.draw();
        }
    }
        break;

    case ColliderType::SPHERE:
    {
        SphereCollider* sc = dynamic_cast<SphereCollider*>(&collider);
        if(sc)
        {
            sphere.setScale(sc->scale);
            sphere.setRotation(sc->rb->rotation);
            sphere.setPosition(sc->rb->position);
            sphere.meshes[0].setColor(baseColor);
            sphere.meshes[1].setColor(boundsColor);
            sphere.draw();
        }
    }
        break;
    }
}

void Scene::drawBoundedCollider(Collider& collider)
{
    switch(collider.type)
    {
    case ColliderType::CUBE:
    {
        BoxCollider* cc = dynamic_cast<BoxCollider*>(&collider);
        if(cc)
        {
            cube.setScale(cc->scale);
            cube.setRotation(cc->rb->rotation);
            cube.setPosition(cc->rb->position);
            cube.draw();
        }
    }
        break;

    case ColliderType::SPHERE:
    {
        SphereCollider* sc = dynamic_cast<SphereCollider*>(&collider);
        if(sc)
        {
            sphere.setScale(sc->scale);
            sphere.setRotation(sc->rb->rotation);
            sphere.setPosition(sc->rb->position);
            sphere.draw();
        }
    }
        break;
    }
}



