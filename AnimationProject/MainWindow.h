#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QCloseEvent>
#include <QWindow>




class MainWindow: public QWindow
{
public:
    std::map<int, bool> inputs;
    std::map<int, bool> inputsDown;
    std::map<int, bool> inputsDownReset;
    QPointF mousePos;
    bool running;

    MainWindow();
    void closeEvent(QCloseEvent* event);
    void mousePressEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void keyPressEvent(QKeyEvent* event) override;
    void keyReleaseEvent(QKeyEvent* event) override;
    bool event(QEvent* event) override;
    void resetInputs();
};

#endif // MAINWINDOW_H
