#ifndef SCENE_H
#define SCENE_H

#include <QPainter>
#include "Entity.h"
#include "PhysicsWorld.h"
#include "Camera.h"
#include "MainWindow.h"

extern void drawLine(Mesh& line, glm::vec3 from, glm::vec3 to);


class Scene
{
public:

    Scene();
    virtual void start();
    virtual void update(float dt);
    virtual void updateConsole(float dt);
    virtual void updateDraw(float dt);

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


};

#endif // SCENE_H
