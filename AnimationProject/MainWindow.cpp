#include <MainWindow.h>
#include <QDebug>
#include <QOpenGLFunctions_4_5_Core>
#include <QApplication>

extern QOpenGLFunctions_4_5_Core* openglFunctions;

MainWindow::MainWindow() : QWindow()
{
    setKeyboardGrabEnabled(true);
    running = true;
    writtenText = "";
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
    if(enableWrite)
    {
        switch(event->key())
        {
        case Qt::Key_Space:
            writtenText.append(" ");
            break;
        case Qt::Key_Backspace:
            writtenText.chop(1);
            break;
        case Qt::Key_Period:
            writtenText.append(".");
            break;
        case Qt::Key_Comma:
            writtenText.append(",");
            break;
        case Qt::Key_0:
            writtenText.append("0");
            break;
        case Qt::Key_1:
            writtenText.append("1");
            break;
        case Qt::Key_2:
            writtenText.append("2");
            break;
        case Qt::Key_3:
            writtenText.append("3");
            break;
        case Qt::Key_4:
            writtenText.append("4");
            break;
        case Qt::Key_5:
            writtenText.append("5");
            break;
        case Qt::Key_6:
            writtenText.append("6");
            break;
        case Qt::Key_7:
            writtenText.append("7");
            break;
        case Qt::Key_8:
            writtenText.append("8");
            break;
        case Qt::Key_9:
            writtenText.append("9");
            break;
        case Qt::Key_Minus:
            writtenText.append("-");
            break;
        case Qt::Key_Equal:
            writtenText.append("=");
            break;
        case Qt::Key_Exclam:
            writtenText.append("!");
            break;
        case Qt::Key_Plus:
            writtenText.append("+");
            break;
        case Qt::Key_Underscore:
            writtenText+="_";
            break;
        case Qt::Key_Shift:
            caps = true;
            break;

        }
        QChar test = QChar(event->key());
        if(!caps)
            test = test.toLower();
        if(test.isLetter())
            writtenText.append(test);
    }
    inputs[event->key()] = true;
    if(!inputsDownReset[event->key()])
    {
        mousePos = QCursor::pos();
        inputsDown[event->key()] = true;
        inputsDownReset[event->key()] = true;
    }
}

void MainWindow::quit()
{
    running = false;
}

void MainWindow::keyReleaseEvent(QKeyEvent* event)
{
    if(enableWrite)
    {
        switch(event->key())
        {
        case Qt::Key_Shift:
            caps = false;
            break;
        }
    }
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

bool CloseEventFilter::eventFilter(QObject *obj, QEvent *event)
{
    if (event->type() == QEvent::Close)
    {
        exit(EXIT_SUCCESS);
    }
    return QObject::eventFilter(obj, event);
}

void MainWindow::toggleWriteEnable()
{
    enableWrite = !enableWrite;
}
void MainWindow::clearText()
{
    writtenText = "";
}
