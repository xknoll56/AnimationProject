#include "Scene.h"
#include "Debug.h"

extern Shader* modelShader;
extern Shader* gridShader;


Scene::Scene(MainWindow& window) : window(window)
{
    glm::mat4 projection = glm::perspective((float)PI*0.33f, (float)window.width()/window.height(), 0.1f, 100.0f);
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
    }
    cam.smoothUpdateView();
    modelShader->setMat4("view", cam.view);
    gridShader->setMat4("view", cam.view);
}

