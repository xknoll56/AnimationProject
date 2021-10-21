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
    UniformRigidBody thrownRb;
    CubeCollider thrownCube;


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
        // SphereBody otherRb(mass, 0.5f);
        collider = CubeCollider(glm::vec3(0.5f,0.5f,0.5f));
        otherCollider = CubeCollider(glm::vec3(10,5.0f,10));
        otherCollider = CubeCollider(glm::vec3(10.0f,0.1f,10.0f));
        collider.rb = &rb;
        otherCollider.rb = &otherRb;
        rb.position = glm::vec3(0, 1, 0);
        rb.setVelocity(glm::vec3(0,0,0));
        rb.dynamic = true;
        otherRb.position = glm::vec3(0,-0.1f, 0);
        otherRb.dynamic = false;
        rb.rotation = glm::quat(glm::vec3(0.0f,0.0f, 0.0f));
       // rb.rotation = glm::quat(glm::vec3(0.0f,0.0f, 0.0f));

        stackedRb = UniformRigidBody(mass, inertia);
        stackedCollider = CubeCollider(glm::vec3(0.5f,0.5f,0.5f));
        stackedCollider.rb = &stackedRb;
        stackedRb.position = glm::vec3(0.0f, 6.0f, 0.0f);
        stackedRb.rotation = glm::quat(glm::vec3(0.0f, 0.0f, 0.0f));
        console.rb = &stackedRb;

        thrownCube = CubeCollider(glm::vec3(0.5f,0.5f,0.5f));
        thrownRb = UniformRigidBody(mass, inertia);
        thrownCube.rb = &thrownRb;
        thrownRb.position = glm::vec3(5, 3, 5);


        std::vector<Collider*> colliders = {  &collider,  &otherCollider, &stackedCollider, &thrownCube};
        ///std::vector<Collider*> colliders = {  &otherCollider, &thrownCube};
        world.gravity = glm::vec3(0,-10.0f,0);
        world.enableResponse = true;
        world.setColliders(&colliders);
    }
    void update(float dt)
    {
        Scene::update(dt);
       // rb.setVelocity(glm::vec3());
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
            thrownRb.linearMomentum = glm::vec3(0,0,0);
            thrownRb.angularMomentum = glm::vec3(0,0,0);
            thrownRb.position = cam.getPosition()+cam.getFwd();
            thrownRb.addForce(-cam.getFwd()*10000.0f);
            thrownRb.atRest = false;
            thrownRb.restingContact = false;
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
                    {
                        drawLine(lineMesh, world.contacts[i].b->rb->position, world.contacts[i].b->rb->position+2.0f*world.contacts[i].normal);
                        drawLine(lineMesh, world.contacts[i].a->rb->position, world.contacts[i].a->rb->position-2.0f*world.contacts[i].normal);
                    }
                }
            }
        }
        else
        {
            cube.meshes[1].setColor(glm::vec3(0,1,0));
        }

    }

    void updateDraw(float dt)
    {
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

        cube.setPosition(thrownRb.position);
        cube.setRotation(thrownRb.rotation);
        cube.setScale(thrownCube.scale);
        cube.draw();


        plane.draw();
    }
};
