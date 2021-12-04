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

    UniformRigidBody* rb = nullptr;
    int sceneIndex = 0;

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
            else if(tokens[1].compare("rotation")==0)
            {
                response.append("x: "+QString::number(rb->rotation.x)+" y: "+QString::number(rb->rotation.y)+
                                " z: "+QString::number(rb->rotation.z)+" w: "+QString::number(rb->rotation.w));
            }
            else if(tokens[1].compare("velocity")==0)
            {
                response.append("x: "+QString::number(rb->velocity.x)+" y: "+QString::number(rb->velocity.y)+" z: "+QString::number(rb->velocity.z));
            }
            else if(tokens[1].compare("angular_velocity")==0)
            {
                response.append("x: "+QString::number(rb->angularVelocity.x)+" y: "+QString::number(rb->angularVelocity.y)+" z: "+QString::number(rb->angularVelocity.z));
            }
            else if(tokens[1].compare("mass")==0)
            {
                response.append("mass: "+QString::number(rb->mass));
            }
            else if(tokens[1].compare("inertia")==0)
            {
                response.append("inertia: "+QString::number(rb->inertia));
            }
        }
        if(tokens[0].compare("set")==0)
        {
            if(rb!=nullptr && rb->dynamic)
            {
                if(tokens[1].compare("position")==0)
                {
                    glm::vec3 vec;
                    if(!getVec3Args(tokens, vec))
                    {
                        response.append("failed to convert to vector");
                    }
                    else
                    {
                        rb->position = vec;
                        response.append("position set");
                    }
                }
                else if(tokens[1].compare("euler")==0)
                {
                    glm::vec3 vec;
                    if(!getVec3Args(tokens, vec))
                    {
                        response.append("failed to convert to vector");
                    }
                    else
                    {
                        rb->rotation = glm::quat(vec);
                        response.append("rotation set");
                    }
                }
                else if(tokens[1].compare("velocity")==0)
                {
                    glm::vec3 vec;
                    if(!getVec3Args(tokens, vec))
                    {
                        response.append("failed to convert to vector");
                    }
                    else
                    {
                        rb->setVelocity(vec);
                        response.append("velocity set");
                    }
                }
                else if(tokens[1].compare("angular_velocity")==0)
                {
                    glm::vec3 vec;
                    if(!getVec3Args(tokens, vec))
                    {
                        response.append("failed to convert to vector");
                    }
                    else
                    {
                        rb->setAngularVelocity(vec);
                        response.append("angular velocity set");
                    }
                }

            }
            if(tokens[1].compare("scene")==0)
            {
                if(tokens[2].compare("collision")==0)
                {
                    sceneIndex = 2;
                }
                else if(tokens[2].compare("demo")==0)
                {
                    sceneIndex = 0;
                }
                else if(tokens[2].compare("drop")==0)
                {
                    sceneIndex = 1;
                }
                else if(tokens[2].compare("vaccume")==0)
                {
                    sceneIndex = 3;
                }
                else  if(tokens[2].compare("stack")==0)
                {
                    sceneIndex = 4;
                }

            }
        }

        return response;
    }

    bool getVec3Args(QStringList& tokens, glm::vec3& returnVec)
    {
        for(int i = 2; i<5; i++)
        {
            for(QChar& c: tokens[i])
            {
                if(!c.isDigit() && c!='-')
                {
                    return false;
                }
            }
            returnVec[i-2] = tokens[i].toFloat();
        }
        return true;
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
    virtual void drawBoundedCollider(Collider& collider, const glm::vec3& baseColor, const glm::vec3& boundsColor);
    virtual void drawBoundedCollider(Collider& collider);

    float elapsedTime;
    bool doUpdateConsole;
    bool consoleToggle;
    ConsoleInterface console;

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

    std::list<QString> commands;
    std::list<QString> replys;
    UniformRigidBody* selectedRb;

    float nearPlane;
    float farPlane;
    float aspectRatio;
    float fov;
    glm::mat4 projection;




};



#endif // SCENE_H
