#include <MainWindow.h>
#include <QDebug>
#include <QOpenGLFunctions_4_5_Core>

extern QOpenGLFunctions_4_5_Core* openglFunctions;

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
        openglFunctions->glViewport(0, 0, width() * devicePixelRatio(), height() * devicePixelRatio());
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
