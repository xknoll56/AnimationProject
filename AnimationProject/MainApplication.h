#ifndef MAINAPPLICATION_H
#define MAINAPPLICATION_H

#include <QElapsedTimer>
#include <QGuiApplication>
#include <QOpenGLContext>
#include <QOpenGLPaintDevice>
#include <QSurfaceFormat>
#include "MainWindow.h"
#include "Scene.h"



class MainApplication: QGuiApplication
{
public:
    Scene* scene;
    MainWindow window;
    MainApplication(int argc, char *argv[]);
    bool setup(int windowWidth, int windowHeight);
    int execute();
private:
    QOpenGLContext* context;
    QElapsedTimer elapsedTimer;
    QOpenGLPaintDevice* paintDevice;
    CloseEventFilter *closeFilter;
};

#endif // MAINAPPLICATION_H
