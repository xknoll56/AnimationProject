#ifndef SCENE_H
#define SCENE_H

#include "Entity.h"
#include "PhysicsWorld.h"
#include "Camera.h"
#include "MainWindow.h"

extern void drawLine(Mesh& line, glm::vec3 from, glm::vec3 to);


class Scene
{
public:

    Scene(MainWindow& window);
    virtual void start();
    virtual void update(float dt);

protected:
    MainWindow& window;
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

};

#endif // SCENE_H
