#pragma once
#include "Scene.h"
extern void drawLine(Mesh& line, glm::vec3 from, glm::vec3 to);
extern MainWindow* gMainWindow;

class DemoScene: public Scene
{
private:
    PhysicsWorld world;
    CubeCollider slope1Collider;
    CubeCollider slope2Collider;
    CubeCollider floorCollider;
    CubeCollider blockCollider;

    SphereCollider sphereCollider;
    UniformRigidBody blockRb;
    UniformRigidBody rb;
    UniformRigidBody otherRb;
    UniformRigidBody sphereRb;
    UniformRigidBody floorRb;


public:
    DemoScene()
    {

    }

    void start()
    {
        float mass = 1.0f;
        float radius = 1.0f;
        float inertia = (2.0f/5.0f)*mass*radius*radius;
        rb = UniformRigidBody(mass, inertia);
        otherRb = UniformRigidBody(mass, inertia);
        sphereRb = UniformRigidBody(mass, inertia);
        floorRb = UniformRigidBody(mass, inertia);
        blockRb = UniformRigidBody(4.0f, inertia);
        // SphereBody otherRb(mass, 0.5f);
        blockCollider = CubeCollider(glm::vec3(1.0f, 1.0f, 3.0f));
        slope1Collider = CubeCollider(glm::vec3(10.0f,0.1f,5.0f));
        slope2Collider = CubeCollider(glm::vec3(10.0f,0.1f,5.0f));
        sphereCollider = SphereCollider(0.5f);
        floorCollider = CubeCollider(glm::vec3(10.0f, 0.1f,10.0f));
        blockCollider.rb = &blockRb;
        floorCollider.rb = &floorRb;
        slope1Collider.rb = &rb;
        slope2Collider.rb = &otherRb;
        sphereCollider.rb = &sphereRb;
        blockRb.position = glm::vec3(-5.0f, 10.0f, 0.0f);
        floorRb.position = glm::vec3(0,-0.05,0);
        sphereRb.position = glm::vec3(-2.5,15,0);
        rb.position = glm::vec3(-2.5f, 12.5, 0);
        rb.dynamic = false;
        otherRb.position = glm::vec3(12.5f,5.0, 0);
        otherRb.dynamic = false;
        floorRb.dynamic = false;
        rb.rotation = glm::quat(glm::vec3(0, 0, -0.3f));
        otherRb.rotation = glm::quat(glm::vec3(0, 0,0.3f));
        console.rb = &rb;


        std::vector<Collider*> colliders = {  &sphereCollider, &slope1Collider, &slope2Collider, &floorCollider, &blockCollider};
        world.gravity = glm::vec3(0,-10.0f,0);
        world.enableResponse = true;
        world.setColliders(&colliders);
        cam.setPosition(glm::vec3(0,10,6));
    }

    void update(float dt)
    {
        Scene::update(dt);
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

        cube.setPosition(blockCollider.rb->position);
        cube.setRotation(blockCollider.rb->rotation);
        cube.setScale(blockCollider.scale);
        cube.draw();


        cube.setPosition(slope1Collider.rb->position);
        cube.setRotation(slope1Collider.rb->rotation);
        cube.setScale(slope1Collider.scale);
        cube.draw();

        cube.setPosition(otherRb.position);
        cube.setRotation(otherRb.rotation);
        cube.setScale(slope2Collider.scale);
        cube.draw();

        sphere.setPosition(sphereRb.position);
        sphere.setRotation(sphereRb.rotation);
        sphere.setScale(sphereCollider.scale);
        sphere.draw();



        plane.draw();
    }

};
