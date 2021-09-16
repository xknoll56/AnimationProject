
#include <QApplication>
#include <QSurfaceFormat>
#include <QWindow>
#include <QOpenGLContext>
#include <QOpenGLPaintDevice>
#include <QOpenGLFunctions>
#include <QOpenGLFunctions_4_3_Core>
#include <QPainter>
#include <QMouseEvent>
#include <QDebug>
#include <QCloseEvent>
#include <QOpenGLShaderProgram>


class MainWindow: public QWindow
{
public:
bool running;
MainWindow() : QWindow()
{
    setKeyboardGrabEnabled(true);
    running = true;
}

void closeEvent(QCloseEvent* event)
{
    event->accept();
    running = true;
}

void keyPressEvent(QKeyEvent* event) override
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
    case QEvent::Close:
        running = false;
    default:
        return QWindow::event(event);
    }
}
};

const char *vertexShaderSource = "#version 330 core\n"
    "layout (location = 0) in vec3 aPos;\n"
    "void main()\n"
    "{\n"
    "   gl_Position = vec4(aPos.x, aPos.y, aPos.z, 1.0);\n"
    "}\0";
const char *fragmentShaderSource = "#version 330 core\n"
    "out vec4 FragColor;\n"
    "void main()\n"
    "{\n"
    "   FragColor = vec4(1.0f, 0.5f, 0.2f, 1.0f);\n"
    "}\n\0";

int main(int argc, char *argv[])
{
    qDebug("Starting");
    QGuiApplication app(argc, argv);

    QSurfaceFormat format;
    format.setSamples(16);
    format.setDepthBufferSize(24);
    format.setMajorVersion(4);
    format.setMinorVersion(3);
    format.setProfile(QSurfaceFormat::CoreProfile);

    MainWindow window;
    window.setFormat(format);
    window.setSurfaceType(QWindow::OpenGLSurface);
    window.resize(1024, 768);
    window.show();

    QOpenGLContext* context = new QOpenGLContext(&window);
    context->setFormat(window.requestedFormat());
    context->create();
    context->makeCurrent(&window);

    QOpenGLFunctions_4_3_Core* openGLFunctions = context->versionFunctions<QOpenGLFunctions_4_3_Core>();
    if(!openGLFunctions)
    {
        qDebug("Could not obtain required version of opengl");
        app.exit();
    //openGLFunctions.initializeOpenGLFunctions();
    }
    QOpenGLPaintDevice* paintDevice = new QOpenGLPaintDevice;
    paintDevice->setSize(window.size() * window.devicePixelRatio());
    paintDevice->setDevicePixelRatio(window.devicePixelRatio());

    QOpenGLShaderProgram program;
    program.addShaderFromSourceCode(QOpenGLShader::Vertex, vertexShaderSource);
    program.addShaderFromSourceCode(QOpenGLShader::Fragment, fragmentShaderSource);
    program.link();

    openGLFunctions->glViewport(0, 0, window.width() * window.devicePixelRatio(), window.height() * window.devicePixelRatio());

    unsigned int shaderProgram = program.programId();

    // set up vertex data (and buffer(s)) and configure vertex attributes
    // ------------------------------------------------------------------
    float vertices[] = {
        -0.5f, -0.5f, 0.0f, // left
         0.5f, -0.5f, 0.0f, // right
         0.0f,  0.5f, 0.0f  // top
    };

    unsigned int VBO, VAO;

    openGLFunctions->glGenVertexArrays(1, &VAO);
    openGLFunctions->glGenBuffers(1, &VBO);
    // bind the Vertex Array Object first, then bind and set vertex buffer(s), and then configure vertex attributes(s).
    openGLFunctions->glBindVertexArray(VAO);

    openGLFunctions->glBindBuffer(GL_ARRAY_BUFFER, VBO);
    openGLFunctions->glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    openGLFunctions->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    openGLFunctions->glEnableVertexAttribArray(0);

    // note that this is allowed, the call to glVertexAttribPointer registered VBO as the vertex attribute's bound vertex buffer object so afterwards we can safely unbind
    openGLFunctions->glBindBuffer(GL_ARRAY_BUFFER, 0);

    // You can unbind the VAO afterwards so other VAO calls won't accidentally modify this VAO, but this rarely happens. Modifying other
    // VAOs requires a call to glBindVertexArray anyways so we generally don't unbind VAOs (nor VBOs) when it's not directly necessary.
    openGLFunctions->glBindVertexArray(0);

    while(window.running)
    {
        app.processEvents();

        openGLFunctions->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

        openGLFunctions->glUseProgram(shaderProgram);
        openGLFunctions->glBindVertexArray(VAO); // seeing as we only have a single VAO there's no need to bind it every time, but we'll do so to keep things a bit more organized
        openGLFunctions->glDrawArrays(GL_TRIANGLES, 0, 3);

        context->makeCurrent(&window);
        context->swapBuffers(&window);

    }

    app.quit();
}
