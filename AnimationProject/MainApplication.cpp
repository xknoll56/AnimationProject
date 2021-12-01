#include "MainApplication.h"


QOpenGLFunctions_4_5_Core* openglFunctions;
Shader* modelShader;
Shader* gridShader;
MainWindow* gMainWindow;
QPaintDevice* gPaintDevice;


MainApplication::MainApplication(int argc, char *argv[]): QGuiApplication(argc, argv)
{

}

bool MainApplication::setup(int windowWidth, int windowHeight)
{

    QSurfaceFormat format;
    format.setSamples(4);
    format.setDepthBufferSize(24);
    format.setMajorVersion(4);
    format.setMinorVersion(5);
    format.setSwapInterval(0);
    format.setSwapBehavior(QSurfaceFormat::SwapBehavior::DefaultSwapBehavior);
    format.setProfile(QSurfaceFormat::CoreProfile);

    gMainWindow = &window;
    window.setTitle("Animation Project");
    window.setFormat(format);
    window.setSurfaceType(QWindow::OpenGLSurface);
    window.resize(1280, 720);
    window.setKeyboardGrabEnabled(true);
    window.show();
    closeFilter = new CloseEventFilter(&window);
    window.installEventFilter(closeFilter);


    context = new QOpenGLContext(&window);
    context->setFormat(window.requestedFormat());
    context->create();
    context->makeCurrent(&window);

    //app.processEvents();
    paintDevice = new QOpenGLPaintDevice;
    gPaintDevice = paintDevice;
    paintDevice->setSize(window.size() * window.devicePixelRatio());
    paintDevice->setDevicePixelRatio(window.devicePixelRatio());

    //painter->setWorldMatrixEnabled(false);

    openglFunctions = context->versionFunctions<QOpenGLFunctions_4_5_Core>();
    if(!openglFunctions)
    {
        qDebug("Could not obtain required version of opengl");
        exit();
    }
    openglFunctions->initializeOpenGLFunctions();



    window.openglInitialized = true;
    openglFunctions->glViewport(0, 0, window.width() * window.devicePixelRatio(), window.height() * window.devicePixelRatio());
//    openglFunctions->glEnable(GL_LINE_SMOOTH);
//    openglFunctions->glEnable(GL_LINE_WIDTH);
//    openglFunctions->glLineWidth(2.5f);
    openglFunctions->glEnable(GL_DEPTH_TEST);
    openglFunctions->glEnable(GL_CULL_FACE);
    openglFunctions->glDisable(GL_LIGHTING);

    modelShader = new Shader("model.vert", "model.frag");
    gridShader = new Shader("grid.vert", "grid.frag");
    modelShader->insertUniform("model");
    modelShader->insertUniform("view");
    modelShader->insertUniform("projection");
    modelShader->insertUniform("color");
    modelShader->insertUniform("lightDir");
    gridShader->insertUniform("model");
    gridShader->insertUniform("view");
    gridShader->insertUniform("projection");
    gridShader->insertUniform("color");



    currentScene = new DemoScene();
    currentSceneIndex = 0;

    return true;
}

int MainApplication::execute()
{
    currentScene->start();
    QElapsedTimer elapsedTimer;
    elapsedTimer.start();
    QElapsedTimer sceneTimer;
    sceneTimer.start();
    long time = elapsedTimer.nsecsElapsed();
    float dt;
    currentScene->elapsedTime = 0.0f;

    while(window.shouldRun())
    {
        swapScenes(&sceneTimer);
        long timeNow = elapsedTimer.nsecsElapsed();
        dt = elapsedTimer.nsecsElapsed()/1000000000.0f;
        elapsedTimer.restart();
        currentScene->elapsedTime = sceneTimer.elapsed()/1000.0f;

        openglFunctions->glEnable(GL_DEPTH_TEST);
        openglFunctions->glEnable(GL_CULL_FACE);
        openglFunctions->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        if(window.windowResized())
        {
            paintDevice->setSize(window.size() * window.devicePixelRatio());
            paintDevice->setDevicePixelRatio(window.devicePixelRatio());
            glm::mat4 projection = glm::perspective((float)PI*0.33f, (float)window.width()/window.height(), 0.1f, 100.0f);
            modelShader->setMat4("projection", projection);
            gridShader->setMat4("projection", projection);
        }


        if(!currentScene->doUpdateConsole)
            currentScene->update(dt);

        currentScene->updateDraw(dt);

        if(currentScene->doUpdateConsole)
            currentScene->updateConsole(dt);



        window.resetInputs();
        processEvents();
        context->makeCurrent(&window);
        context->swapBuffers(&window);
    }

    quit();
    return 0;
}

void MainApplication::swapScenes(QElapsedTimer* sceneTimer)
{
    if(currentSceneIndex!=currentScene->console.sceneIndex)
    {
        int newIndex = currentScene->console.sceneIndex;
        //In case the index has be changes from leaving the scene
        currentSceneIndex = newIndex;
        delete currentScene;
        switch(currentSceneIndex)
        {
        case 0:
            demoScene = new DemoScene();
            currentScene = demoScene;
            break;
        case 1:
            cubeDropScene = new CubeDropScene();
            currentScene = cubeDropScene;
            break;
        case 2:
            collisionTestScene = new CollisionTestScene();
            currentScene = collisionTestScene;
            break;
        case 3:
            vaccumeScene = new VaccumeScene();
            currentScene = vaccumeScene;
            break;
        case 4:
            stackScene = new StackScene();
            currentScene = stackScene;
            break;
        }
        currentScene->console.sceneIndex = newIndex;
        currentScene->start();
        gMainWindow->toggleWriteEnable();
        currentScene->elapsedTime = 0.0f;
        sceneTimer->restart();
    }
}
