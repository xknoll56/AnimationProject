#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QCloseEvent>
#include <QMainWindow>
#include <QWindow>




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
    void mouseMoveEvent(QMouseEvent* event) override;
    void resetInputs();
    bool shouldRun();
    void quit();
    void toggleWriteEnable();
    void clearText();
    bool getKey(Qt::Key);
    bool getKeyDown(Qt::Key);
    bool getMouse(Qt::MouseButton);
    bool getMouseDown(Qt::MouseButton);
    void lockCursorState();
    void unlockCursorState();
    bool openglInitialized = false;
    float mouseDx;
    float mouseDy;
    bool windowResized()
    {
        bool temp = resized;
        resized = false;
        return temp;
    }

    QString writtenText;
private:
    std::map<int, bool> inputs;
    std::map<int, bool> inputsDown;
    std::map<int, bool> inputsDownReset;
    bool running;
    bool resized = false;
    bool enableWrite = false;
    bool caps = false;
    bool lockCursor = false;
    bool adjustMouse;

};

class CloseEventFilter : public QObject
{
public:
     CloseEventFilter(QObject *parent) : QObject(parent) {}

protected:
     bool eventFilter(QObject *obj, QEvent *event);
};


#endif // MAINWINDOW_H
