#include <MainWindow.h>
#include <QDebug>
#include <QOpenGLFunctions_4_4_Core>

QOpenGLFunctions_4_4_Core* openglFunctions;

MainWindow::MainWindow() : QWindow()
{
    setKeyboardGrabEnabled(true);
    running = true;
}


void MainWindow::mousePressEvent(QMouseEvent *event)
{
    inputs[event->button()] = true;
    if(!inputsDownReset[event->button()])
    {
        mousePos = QCursor::pos();
        inputsDown[event->button()] = true;
        inputsDownReset[event->button()] = true;
    }
}

void MainWindow::mouseReleaseEvent(QMouseEvent *event)
{

    inputs[event->button()] = false;
    inputsDownReset[event->button()] = false;

}

void MainWindow::keyPressEvent(QKeyEvent* event)
{
    switch(event->key())
    {
    case Qt::Key_Escape:
        running = false;
    }
    inputs[event->key()] = true;
    if(!inputsDownReset[event->key()])
    {
        mousePos = QCursor::pos();
        inputsDown[event->key()] = true;
        inputsDownReset[event->key()] = true;
    }
}

void MainWindow::keyReleaseEvent(QKeyEvent* event)
{
    if(!event->isAutoRepeat())
    {
        inputs[event->key()] = false;
        inputsDownReset[event->key()] = false;
    }
}

void MainWindow::resizeEvent(QResizeEvent *ev)
{
    if(openglInitialized)
    {
        resized = true;
    }
}


void MainWindow::resetInputs()
{
    for (auto& it: inputsDown)
        it.second = false;
}

bool MainWindow::shouldRun()
{
    return running;
}
bool MainWindow::getKey(Qt::Key key)
{
    return inputs[key];
}
bool MainWindow::getGetDown(Qt::Key key)
{
    return inputsDown[key];
}

bool MainWindow::getMouse(Qt::MouseButton button)
{
    return inputs[button];
}
bool MainWindow::getMouseDown(Qt::MouseButton button)
{
    return inputsDown[button];
}

bool MainWindow::InitializeContext()
{
    if(!context)
    {
        context = new QOpenGLContext(this);
        context->setFormat(requestedFormat());
        context->create();
        context->makeCurrent(this);

        paintDevice = new QOpenGLPaintDevice;
        paintDevice->setSize(size() * devicePixelRatio());
        paintDevice->setDevicePixelRatio(devicePixelRatio());
        painter = new QPainter(paintDevice);
        painter->setPen(Qt::white);
        painter->setFont(QFont("Arial", 12));

        painter->setWorldMatrixEnabled(false);
        return true;
    }
    else
        return false;
}

bool MainWindow::InitializeOpenGLFunctions()
{
    openglFunctions = context->versionFunctions<QOpenGLFunctions_4_4_Core>();
    if(!openglFunctions)
    {
        qDebug("Could not obtain required version of opengl");
        return false;
    }
    openglFunctions->initializeOpenGLFunctions();
    openglInitialized = true;
    return true;
}

bool MainWindow::shouldResize()
{
    bool didResize = resized;
    resized = false;
    return didResize;
}
