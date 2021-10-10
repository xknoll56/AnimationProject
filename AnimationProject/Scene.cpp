#include "Scene.h"
#include "Debug.h"

extern Shader* modelShader;
extern Shader* gridShader;
extern MainWindow* gMainWindow;
extern QPaintDevice* gPaintDevice;
extern QOpenGLFunctions_4_5_Core* openglFunctions;

Scene::Scene()
{

    painter.setWorldMatrixEnabled(false);
    doUpdateConsole = false;
    consoleToggle = true;
    glm::mat4 projection = glm::perspective((float)PI*0.33f, (float)gMainWindow->width()/gMainWindow->height(), 0.1f, 100.0f);
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
    if(gMainWindow->getGetDown(Qt::Key_0))
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
        QPointF deltaPos = QCursor::pos()-gMainWindow->mousePos;
        gMainWindow->mousePos = QCursor::pos();
        cam.rotateYaw(-dt*deltaPos.x());
        cam.rotatePitch(-dt*deltaPos.y());
    }
    cam.updateView();
    modelShader->setMat4("view", cam.view);
    gridShader->setMat4("view", cam.view);

}

void Scene::updateConsole(float dt)
{
    openglFunctions->glDisable(GL_DEPTH_TEST);
    openglFunctions->glDisable(GL_CULL_FACE);

    painter.begin(gPaintDevice);
    painter.setPen(Qt::green);
    painter.setBrush(Qt::black);
    painter.setOpacity(0.8f);
    painter.setFont(QFont("Monospace", 12));

    QRectF rect(0.0f,gMainWindow->size().height()*3/4,gMainWindow->size().width(), gMainWindow->size().height()/4);


    painter.drawRect(rect);
    painter.drawText(rect, Qt::AlignBottom, gMainWindow->writtenText);

    painter.end();


    if(gMainWindow->getGetDown(Qt::Key_0) && consoleToggle)
    {
        doUpdateConsole = false;
        gMainWindow->clearText();
        gMainWindow->toggleWriteEnable();
    }
    else
        consoleToggle = true;
}

void Scene::updateDraw(float dt)
{

}
