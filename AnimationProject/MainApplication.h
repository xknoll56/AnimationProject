#ifndef MAINAPPLICATION_H
#define MAINAPPLICATION_H

#include <QElapsedTimer>
#include <QGuiApplication>
#include <QOpenGLContext>
#include <QOpenGLPaintDevice>
#include <QSurfaceFormat>
#include "MainWindow.h"
#include "Scene.h"
#include <Scenes/StackScene.h>
#include <Scenes/CollisionTestScene.h>
#include <Scenes/CubeDropScene.h>
#include <Scenes/DemoScene.h>
#include <Scenes/VaccumeScene.h>



class MainApplication: QGuiApplication
{
public:
    MainWindow window;
    MainApplication(int argc, char *argv[]);
    bool setup(int windowWidth, int windowHeight);
    int execute();
private:
    Scene* currentScene;
    StackScene* stackScene;
    CollisionTestScene* collisionTestScene;
    CubeDropScene* cubeDropScene;
    DemoScene* demoScene;
    VaccumeScene* vaccumeScene;
    QOpenGLContext* context;
    void swapScenes(QElapsedTimer* sceneTimer);
    QElapsedTimer elapsedTimer;
    QOpenGLPaintDevice* paintDevice;
    CloseEventFilter *closeFilter;
    int currentSceneIndex;


};

#endif // MAINAPPLICATION_H
