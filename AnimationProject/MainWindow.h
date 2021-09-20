#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QCloseEvent>
#include <QMainWindow>
#include <QOpenGLPaintDevice>
#include <QWindow>
#include <QPainter>



class MainWindow: public QWindow
{

public:
    QPointF mousePos;
    MainWindow();
    void mousePressEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void keyPressEvent(QKeyEvent* event) override;
    void keyReleaseEvent(QKeyEvent* event) override;
    void resizeEvent(QResizeEvent *ev) override;
    void resetInputs();
    bool shouldRun();
    bool getKey(Qt::Key);
    bool getGetDown(Qt::Key);
    bool getMouse(Qt::MouseButton);
    bool getMouseDown(Qt::MouseButton);
    bool InitializeContext();
    bool InitializeOpenGLFunctions();
    QOpenGLContext* context;
    QOpenGLPaintDevice* paintDevice;
    QPainter* painter;
    bool shouldResize();

private:
    std::map<int, bool> inputs;
    std::map<int, bool> inputsDown;
    std::map<int, bool> inputsDownReset;
    bool running;
    bool openglInitialized = false;
    bool resized = false;
};

#endif // MAINWINDOW_H
