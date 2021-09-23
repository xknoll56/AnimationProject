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
    void resetInputs();
    bool shouldRun();
    bool getKey(Qt::Key);
    bool getGetDown(Qt::Key);
    bool getMouse(Qt::MouseButton);
    bool getMouseDown(Qt::MouseButton);
    bool openglInitialized = false;
    bool windowResized()
    {
        bool temp = resized;
        resized = false;
        return temp;
    }

private:
    std::map<int, bool> inputs;
    std::map<int, bool> inputsDown;
    std::map<int, bool> inputsDownReset;
    bool running;
    bool resized = false;
};

class CloseEventFilter : public QObject
{
public:
     CloseEventFilter(QObject *parent) : QObject(parent) {}

protected:
     bool eventFilter(QObject *obj, QEvent *event);
};


#endif // MAINWINDOW_H
