#pragma once
#include "Scene.h"
extern void drawLine(Mesh& line, glm::vec3 from, glm::vec3 to);
extern MainWindow* gMainWindow;

class StackScene: public Scene
{
private:
    PhysicsWorld world;
    CubeCollider collider;
    CubeCollider stackedCollider;
    UniformRigidBody stackedRb;
    CubeCollider otherCollider;
    UniformRigidBody rb;
    UniformRigidBody otherRb;


public:
    StackScene()
    {

    }
    void start()
    {
        float mass = 1.0f;
        float radius = 1.0f;
        float inertia = (2.0f/5.0f)*mass*radius*radius;
        rb = UniformRigidBody(mass, inertia);
        otherRb = UniformRigidBody(mass, inertia);
        stackedRb = UniformRigidBody(mass, inertia);
        // SphereBody otherRb(mass, 0.5f);
        collider = CubeCollider(glm::vec3(0.5f,0.5f,0.5f));
        otherCollider = CubeCollider(glm::vec3(10.0f,0.5f,10.0f));
        stackedCollider = CubeCollider(glm::vec3(0.5f,0.5f,0.5f));
        stackedCollider.rb = &stackedRb;
        collider.rb = &rb;
        otherCollider.rb = &otherRb;
        rb.position = glm::vec3(0, 1.5, 0);
        stackedRb.position = glm::vec3(0, 3, 0);
        rb.dynamic = true;
        otherRb.position = glm::vec3(0,-0.5f, 0);
        otherRb.dynamic = false;
        //rb.rotation = glm::quat(glm::vec3(PI/3.0f,0.0f, PI/3.0f));
        rb.rotation = glm::quat(glm::vec3(0.0f,0.0f, 0.0f));


        std::vector<Collider*> colliders = {&otherCollider, &collider, &stackedCollider};
        world.gravity = glm::vec3(0,-10.0f,0);
        world.enableResponse = true;
        world.setColliders(&colliders);
    }
    void update(float dt)
    {
        Scene::update(dt);
        //rb.setVelocity(glm::vec3());
        if(gMainWindow->getKey(Qt::Key_Right))
        {
            rb.setVelocity(1.0f*cam.getRight());
            //rb.addForce(2.0f*cam.getRight());
        }
        if(gMainWindow->getKey(Qt::Key_Left))
        {
            rb.setVelocity(-1.0f*cam.getRight());
            //rb.addForce(-2.0f*cam.getRight());
        }
        if(gMainWindow->getKey(Qt::Key_E))
        {
            rb.setVelocity(1.0f*glm::vec3(0,1,0));
        }
        if(gMainWindow->getKey(Qt::Key_Q))
        {
            rb.setVelocity(-1.0f*glm::vec3(0,1,0));
        }
        if(gMainWindow->getKey(Qt::Key_Up))
        {
            rb.setVelocity(glm::cross(glm::vec3(0,1,0), cam.getRight()));
            //rb.addForce(glm::cross(glm::vec3(0,2,0), cam.getRight()));

        }
        if(gMainWindow->getKey(Qt::Key_Down))
        {
            rb.setVelocity(glm::cross(glm::vec3(0,-1,0), cam.getRight()));
            //rb.addForce(glm::cross(glm::vec3(0,-2,0), cam.getRight()));

        }
        if(gMainWindow->getGetDown(Qt::Key_Space))
        {
            rb.addForce(glm::vec3(0,800,0));
        }
        if(gMainWindow->getGetDown(Qt::Key_R))
        {
            rb.setVelocity(glm::vec3(0,0,0));
            rb.position = glm::vec3(0,10,0);
            rb.setAngularVelocity(glm::vec3(0,0,0));
            rb.rotation = glm::quat(glm::vec3(2*PI/(rand()%8+1), 2*PI/(rand()%8+1), 2*PI/(rand()%8+1)));

        }
        if(gMainWindow->getGetDown(Qt::Key_1))
        {
            rb.setVelocity(glm::vec3(0,0,0));
        }

        //rb.setAngularVelocity(glm::vec3(0.2, 0.3, 0.4));
        //world.stepWorld(0.0009f);
        world.stepWorld(dt);

        if(world.contacts.size()>0)
        {
            cube.meshes[1].setColor(glm::vec3(1,0,0));

            for(int i =0;i<world.contacts.size();i++)
            {
                for(int j =0;j<world.contacts[i].points.size();j++)
                {
                    point.setPosition(world.contacts[i].points[j]);
                    point.draw();
                    if(j == 0)
                        drawLine(lineMesh, world.contacts[i].a->rb->position, world.contacts[i].a->rb->position+2.0f*world.contacts[i].normal);
                }
            }
        }
        else
        {
            cube.meshes[1].setColor(glm::vec3(0,1,0));
        }
        cube.setPosition(rb.position);
        cube.setRotation(rb.rotation);
        cube.setScale(collider.scale);
        cube.draw();

        cube.setPosition(stackedRb.position);
        cube.setRotation(stackedRb.rotation);
        cube.setScale(stackedCollider.scale);
        cube.draw();

        cube.setPosition(otherRb.position);
        cube.setRotation(otherRb.rotation);
        cube.setScale(otherCollider.scale);
        cube.draw();
        plane.draw();
    }
};
