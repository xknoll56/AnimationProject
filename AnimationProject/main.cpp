
#include <QApplication>
#include <QSurfaceFormat>
#include <QWindow>
#include <QOpenGLContext>
#include <QOpenGLPaintDevice>
#include <QOpenGLFunctions>
#include <QPainter>
#include <QMouseEvent>
#include <QDebug>

class MainWindow: public QWindow
{
public:
bool running;
MainWindow() : QWindow()
{
    setKeyboardGrabEnabled(true);
    running = true;
}

void keyPressEvent(QKeyEvent* event)
{
    switch(event->key())
    {
    case Qt::Key_Escape:
        running = false;
    }
}

bool event(QEvent* event) override
{
    switch(event->type())
    {
    case QKeyEvent::KeyPress:
        //keyPressEvent((QKeyEvent*)event);
    default:
        return QWindow::event(event);
    }
}
};

int main(int argc, char *argv[])
{
    qDebug("Starting");
    QGuiApplication app(argc, argv);

    QSurfaceFormat format;
    format.setSamples(16);

    MainWindow window;
    window.setFormat(format);
    window.setSurfaceType(QWindow::OpenGLSurface);
    window.resize(1024, 768);
    window.show();

    QOpenGLContext* context = new QOpenGLContext(&window);
    context->setFormat(window.requestedFormat());
    context->create();
    context->makeCurrent(&window);

    QOpenGLFunctions openGLFunctions(context);
    openGLFunctions.initializeOpenGLFunctions();

    QOpenGLPaintDevice* paintDevice = new QOpenGLPaintDevice;
    paintDevice->setSize(window.size() * window.devicePixelRatio());
    paintDevice->setDevicePixelRatio(window.devicePixelRatio());

    while(window.running)
    {
        openGLFunctions.glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
        context->makeCurrent(&window);
        context->swapBuffers(&window);
        app.processEvents();
    }

    app.quit();
}
