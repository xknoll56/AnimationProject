#ifndef SCENE_H
#define SCENE_H

#include <QPainter>
#include "Entity.h"
#include "PhysicsWorld.h"
#include "Camera.h"
#include "MainWindow.h"
#include <list>
#include <QDebug>

extern void drawLine(Mesh& line, glm::vec3 from, glm::vec3 to);

class ConsoleInterface
{
public:

    UniformRigidBody* rb;

    QString ParseCommand(QString& command)
    {
        QString response;
        QStringList tokens = command.split(' ');
        if(tokens[0].compare("get")==0)
        {
            if(tokens[1].compare("position")==0)
            {
                response.append("x: "+QString::number(rb->position.x)+" y: "+QString::number(rb->position.y)+" z: "+QString::number(rb->position.z));
            }
        }

        return response;
    }
};

class Scene
{
public:

    Scene();
    virtual void start();
    virtual void update(float dt);
    virtual void updateConsole(float dt);
    virtual void updateDraw(float dt);
    virtual void drawCrosshair();
    virtual void selectRigidBody(PhysicsWorld& world);

    float elapsedTime;
    bool doUpdateConsole;
    bool consoleToggle;
protected:
    Entity plane;
    Mesh lineMesh;
    Entity unitDirs;
    Entity cube;
    Entity cone;
    Entity sphere;
    Entity cylinder;
    Entity capsule;
    Entity arrow;
    Camera cam;
    Entity point;
    QPainter painter;
    ConsoleInterface console;
    std::list<QString> commands;
    std::list<QString> replys;
    UniformRigidBody* selectedRb;



};



#endif // SCENE_H
