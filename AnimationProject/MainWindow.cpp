#include <MainWindow.h>

MainWindow::MainWindow() : QWindow()
{
    setKeyboardGrabEnabled(true);
    running = true;
}

void MainWindow::closeEvent(QCloseEvent* event)
{
    event->accept();
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

bool MainWindow::event(QEvent* event)
{
    switch(event->type())
    {
    case QEvent::Close:
        running = false;
    default:
        return QWindow::event(event);
    }
}

void MainWindow::resetInputs()
{
    for (auto& it: inputsDown)
        it.second = false;
}
