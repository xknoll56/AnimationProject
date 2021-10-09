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
#include <Scene.h>
#include <MainApplication.h>
#include <Scenes/StackScene.h>
#include <Scenes/CollisionTestScene.h>
#include <Scenes/CubeDropScene.h>



float dt;
QOpenGLFunctions_4_5_Core* openglFunctions;
Shader* modelShader;
Shader* gridShader;
//Mesh* lineMesh;


int main(int argc, char *argv[])
{
    MainApplication app(argc, argv);
    app.setup(1280, 720);
    //CollisionTestScene scene;
    CubeDropScene scene;
   // StackScene scene;
    app.scene = &scene;
    app.execute();
}
