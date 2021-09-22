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

struct Transform
{
    glm::vec3 position;
    glm::vec3 scale;
    glm::quat rotation;
    glm::vec3 euler;

    Transform()
    {
        position = glm::vec3();
        scale = glm::vec3(1,1,1);
        euler = glm::vec3();
        rotation = glm::quat(euler);
    }

    Transform(glm::vec3 position, glm::vec3 scale, glm::vec3 euler)
    {
        this->position = position;
        this->scale = scale;
        this->euler = euler;
        this->rotation = glm::quat(euler);
    }

    Transform(glm::vec3 position, glm::vec3 scale, glm::quat rotation)
    {
        this->position = position;
        this->scale = scale;
        this->rotation = rotation;
        this->euler = glm::eulerAngles(rotation);
    }
};

glm::mat4 IDENTITY(1.0f);
Shader* modelShader;
Shader* gridShader;

class Entity
{
private:
    Transform transform;
    glm::mat4 positionMat;
    glm::mat4 scaleMat;
    glm::mat4 rotationMat;
    glm::mat4 model;
    std::vector<Entity> children;


    void setModel()
    {
        model = positionMat*rotationMat*scaleMat;
    }

    void drawEntitiesRecursively(const glm::mat4& parentModel)
    {
        glm::mat4 mat =  parentModel*model;
        for(auto& mesh: meshes)
        {
            switch(mesh.getType())
            {
            case GL_TRIANGLES:
                modelShader->setMat4("model", mat);
                mesh.draw(*modelShader);
                break;
            case GL_LINES:
                gridShader->setMat4("model", mat);
                mesh.draw(*gridShader);
                break;
            }
        }

        for(auto& child: children)
        {
            child.drawEntitiesRecursively(mat);
        }
    }

public:

    std::vector<Mesh> meshes;

    Entity(Mesh& mesh)
    {
        positionMat = glm::translate(IDENTITY, transform.position);
        scaleMat = glm::scale(IDENTITY, transform.scale);
        rotationMat = glm::toMat4(transform.rotation);
        setModel();
        meshes.push_back(mesh);
    }

    Entity(Mesh& mesh, const Transform& transform)
    {
        setTransform(transform);
        meshes.push_back(mesh);
    }

    Entity(const Transform& transform)
    {
        setTransform(transform);
    }

    Entity()
    {
        positionMat = glm::translate(IDENTITY, transform.position);
        scaleMat = glm::scale(IDENTITY, transform.scale);
        rotationMat = glm::toMat4(transform.rotation);
        setModel();
    }

    Entity(const std::vector<Mesh>& meshes)
    {
        positionMat = glm::translate(IDENTITY, transform.position);
        scaleMat = glm::scale(IDENTITY, transform.scale);
        rotationMat = glm::toMat4(transform.rotation);
        setModel();
        this->meshes = meshes;
    }

    const Transform& getTransform()
    {
        return transform;
    }

    void setTransform(const Transform& transform)
    {
        this->transform = transform;
        positionMat = glm::translate(IDENTITY, transform.position);
        scaleMat = glm::scale(IDENTITY, transform.scale);
        rotationMat = glm::toMat4(transform.rotation);
        setModel();
    }

    void translate(const glm::vec3& trans)
    {
        transform.position += trans;
        positionMat = glm::translate(positionMat, trans);
        setModel();
    }

    void rotate(const glm::quat& rot)
    {
        transform.rotation *= rot;
        rotationMat = glm::toMat4(transform.rotation);
        setModel();
    }

    void scale(const glm::vec3 scale)
    {
        transform.scale += scale;
        scaleMat = glm::scale(IDENTITY, transform.scale);
        setModel();
    }

    void setPosition(const glm::vec3& position)
    {
        transform.position = position;
        positionMat = glm::translate(IDENTITY, transform.position);
        setModel();
    }

    void setScale(const glm::vec3& scale)
    {
        transform.scale = scale;
        scaleMat = glm::scale(IDENTITY, transform.scale);
        setModel();
    }

    void setRotation(const glm::quat& rotation)
    {
        transform.rotation = rotation;
        transform.euler = glm::eulerAngles(rotation);
        rotationMat = glm::toMat4(rotation);
        setModel();
    }

    void addMeshWithColor(const Mesh& mesh, glm::vec3 color)
    {
        Mesh toAdd = Mesh(mesh);
        toAdd.setColor(color);
        meshes.push_back(toAdd);
    }
    void addChild(Entity& entity)
    {
        children.push_back(entity);
    }

    void draw()
    {
        drawEntitiesRecursively(IDENTITY);
    }
};

class Cube: public Entity
{
public:

};

Entity createBoundedCubeEntity()
{
    std::vector<Mesh> meshes = {Mesh::createCube(), Mesh::createBoundingBox()};
    return Entity(meshes);
}

Entity createBoundedCylinderEntity()
{
    std::vector<Mesh> meshes = {Mesh::createCylinder(), Mesh::createBoundingCylinder()};
    return Entity(meshes);
}

Entity createBoundedSphereEntity()
{
    std::vector<Mesh> meshes = {Mesh::createSphere(), Mesh::createBoundingSphere()};
    return Entity(meshes);
}

Entity createBoundedConeEntity()
{
    std::vector<Mesh> meshes = {Mesh::createCone(), Mesh::createBoundingCone()};
    return Entity(meshes);
}

Entity createBoundedPlaneEntity()
{
    std::vector<Mesh> meshes = {Mesh::createPlane(), Mesh::createBoundingPlane()};
    return Entity(meshes);
}

Entity createUnitDirs()
{
    Entity unitDirs;
    Mesh cone = Mesh::createCone();
    Mesh cyl = Mesh::createCylinder();

    Transform tCylX(glm::vec3(0.5f, 0, 0), glm::vec3(0.1f, 0.5f, 0.1f), glm::vec3(0, 0, -(float)PI*0.5f));
    Transform tConeX(glm::vec3(1.0f, 0, 0), glm::vec3(0.25f, 0.25f, 0.25f), glm::vec3(0, 0, -(float)PI*0.5f));
    Transform tCylY(glm::vec3(0, 0.5f, 0), glm::vec3(0.1f, 0.5f, 0.1f), glm::vec3(0, 0, 0));
    Transform tConeY(glm::vec3(0, 1.0f, 0), glm::vec3(0.25f, 0.25f, 0.25f), glm::vec3(0, 0, 0));
    Transform tCylZ(glm::vec3(0, 0, 0.5f), glm::vec3(0.1f, 0.5f, 0.1f), glm::vec3((float)PI*0.5f, 0, 0));
    Transform tConeZ(glm::vec3(0, 0, 1.0f), glm::vec3(0.25f, 0.25f, 0.25f), glm::vec3((float)PI*0.5f, 0, 0));

    Entity cylX(tCylX);
    Entity coneX(tConeX);
    Entity cylY(tCylY);
    Entity coneY(tConeY);
    Entity cylZ(tCylZ);
    Entity coneZ(tConeZ);

    cylX.addMeshWithColor(cyl, glm::vec3(1,0,0));
    coneX.addMeshWithColor(cone, glm::vec3(1,0,0));
    cylY.addMeshWithColor(cyl, glm::vec3(0,1,0));
    coneY.addMeshWithColor(cone, glm::vec3(0,1,0));
    cylZ.addMeshWithColor(cyl, glm::vec3(0,0,1));
    coneZ.addMeshWithColor(cone, glm::vec3(0,0,1));

    unitDirs.addChild(cylX);
    unitDirs.addChild(coneX);
    unitDirs.addChild(cylY);
    unitDirs.addChild(coneY);
    unitDirs.addChild(cylZ);
    unitDirs.addChild(coneZ);

    return unitDirs;
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
    openglFunctions->glLineWidth(2.0f);
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

    Mesh gridMesh = Mesh::createGrid(10);




    Entity plane = createBoundedPlaneEntity();
    plane.setScale(glm::vec3(20,1,20));

    Entity unitDirs = createUnitDirs();
    Entity cube = createBoundedCubeEntity();
    Entity cone = createBoundedConeEntity();
    Entity sphere = createBoundedSphereEntity();
    Entity cylinder = createBoundedCylinderEntity();
    cube.setPosition(glm::vec3(3,0,0));
    cone.setPosition(glm::vec3(-3, 0, 0));
    sphere.setPosition(glm::vec3(0,0,3));
    cylinder.setPosition(glm::vec3(0,0,-3));
    unitDirs.addChild(cube);
    unitDirs.addChild(cone);
    unitDirs.addChild(sphere);
    unitDirs.addChild(cylinder);

    unitDirs.setPosition(glm::vec3(0,3, 0));


    QElapsedTimer timer;
    timer.start();
    glm::quat q(glm::vec3(0,0,0));

    while(window.shouldRun())
    {

        dt = timer.nsecsElapsed()/1000000000.0;
        timer.restart();

        openglFunctions->glEnable(GL_DEPTH_TEST);
        openglFunctions->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

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
        modelShader->setMat4("view", cam.view);
        gridShader->setMat4("view", cam.view);

        unitDirs.rotate(glm::quat(glm::vec3(0, dt,0)));
        unitDirs.draw();

        plane.draw();


        gridShader->setMat4("model", trans);
        gridShader->setVec3("color", glm::vec3(1, 0, 0));
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
