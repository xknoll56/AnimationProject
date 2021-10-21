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
    format.setSwapInterval(1);
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
    return true;
}

int MainApplication::execute()
{
    scene->start();
    QElapsedTimer elapsedTimer;
    elapsedTimer.start();
    long time = elapsedTimer.nsecsElapsed();
    float dt;
    while(window.shouldRun())
    {

        long timeNow = elapsedTimer.nsecsElapsed();
        dt = elapsedTimer.nsecsElapsed()/1000000000.0f;
        elapsedTimer.restart();

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


        if(!scene->doUpdateConsole)
            scene->update(dt);

        scene->updateDraw(dt);

        if(scene->doUpdateConsole)
            scene->updateConsole(dt);



        window.resetInputs();
        processEvents();
        context->makeCurrent(&window);
        context->swapBuffers(&window);
    }

    quit();
    return 0;
}
