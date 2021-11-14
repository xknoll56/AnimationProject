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
    CubeCollider leftSideCollider;
    CubeCollider rightSideCollider;
    CubeCollider leftSideCollider1;
    CubeCollider rightSideCollider1;
    CubeCollider backCollider;
    CubeCollider backCollider1;

    std::vector<SphereCollider> sphereColliders;
    UniformRigidBody blockRb;
    UniformRigidBody rb;
    UniformRigidBody otherRb;
    UniformRigidBody leftSideRb;
    UniformRigidBody rightSideRb;
    UniformRigidBody leftSideRb1;
    UniformRigidBody rightSideRb1;
    UniformRigidBody backRb;
    UniformRigidBody backRb1;
    std::vector<UniformRigidBody> sphereRbs;
    UniformRigidBody floorRb;
    int numBodies = 15;


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
        sphereRbs.reserve(numBodies);
        sphereColliders.reserve(numBodies);
        for(int i  =0; i<numBodies; i++)
        {
            UniformRigidBody sphereRb(mass, inertia);
            sphereRbs.push_back(sphereRb);
            sphereColliders.push_back(SphereCollider(0.5f));
            sphereColliders[sphereColliders.size()-1].rb = &sphereRbs[sphereRbs.size()-1];
            sphereRbs[sphereRbs.size()-1].position =  glm::vec3(-2.5, 15+i, 0);
            sphereRbs[sphereRbs.size()-1].setVelocity(glm::vec3(0, 0, (7-i)*0.35f));

        }
        floorRb = UniformRigidBody(mass, inertia);
        blockRb = UniformRigidBody(4.0f, inertia);
        leftSideRb = UniformRigidBody(mass, inertia);
        rightSideRb = UniformRigidBody(mass, inertia);
        leftSideRb1 = UniformRigidBody(mass, inertia);
        rightSideRb1 = UniformRigidBody(mass, inertia);
        backRb = UniformRigidBody(mass, inertia);
        backRb1 = UniformRigidBody(mass, inertia);
        // SphereBody otherRb(mass, 0.5f);
        blockCollider = CubeCollider(glm::vec3(1.0f, 1.0f, 3.0f));
        slope1Collider = CubeCollider(glm::vec3(10.0f,0.1f,5.0f));
        slope2Collider = CubeCollider(glm::vec3(10.0f,0.1f,5.0f));
        floorCollider = CubeCollider(glm::vec3(10.0f, 0.1f,10.0f));
        leftSideCollider = CubeCollider(glm::vec3(10.0f, 1.0f, 0.1f));
        rightSideCollider = CubeCollider(glm::vec3(10.0f, 1.0f, 0.1f));
        leftSideCollider1 = CubeCollider(glm::vec3(10.0f, 1.0f, 0.1f));
        rightSideCollider1 = CubeCollider(glm::vec3(10.0f, 1.0f, 0.1f));
        backCollider = CubeCollider(glm::vec3(0.1f, 1.0f, 5.0f));
         backCollider1 = CubeCollider(glm::vec3(0.1f, 1.0f, 5.0f));
        leftSideCollider1.rb = &leftSideRb1;
        rightSideCollider1.rb = &rightSideRb1;
        leftSideCollider.rb = &leftSideRb;
        rightSideCollider.rb = &rightSideRb;
        backCollider.rb = &backRb;
        backCollider1.rb = &backRb1;

        blockCollider.rb = &blockRb;
        floorCollider.rb = &floorRb;
        slope1Collider.rb = &rb;
        slope2Collider.rb = &otherRb;
        leftSideRb.position = glm::vec3(-2.5, 13.5, -5.1f);
        rightSideRb.position = glm::vec3(-2.5, 13.5, 5.1f);
        leftSideRb1.position = glm::vec3(12.5f,6.0, -5.1f);
        rightSideRb1.position = glm::vec3(12.5f,6.0, 5.1f);
        backRb.position = glm::vec3(21.9f, 9.12, 0.0f);
        backRb1.position = glm::vec3(-12.0, 16.5f, 0.0f);
        blockRb.position = glm::vec3(-5.0f, 10.0f, 0.0f);
        floorRb.position = glm::vec3(0,-0.05,0);
        rb.position = glm::vec3(-2.5f, 12.5, 0);
        rb.dynamic = false;
        otherRb.position = glm::vec3(12.5f,5.0, 0);
        otherRb.dynamic = false;
        floorRb.dynamic = false;
        leftSideRb.dynamic = false;
        rightSideRb.dynamic = false;
        leftSideRb1.dynamic = false;
        rightSideRb1.dynamic = false;
        backRb.dynamic = false;
        backRb1.dynamic = false;
        rb.rotation = glm::quat(glm::vec3(0, 0, -0.3f));
        otherRb.rotation = glm::quat(glm::vec3(0, 0,0.3f));
        leftSideRb.rotation =glm::quat(glm::vec3(0,0,-0.3f));
        rightSideRb.rotation =glm::quat(glm::vec3(0,0,-0.3f));
        leftSideRb1.rotation =glm::quat(glm::vec3(0,0,0.3f));
        rightSideRb1.rotation =glm::quat(glm::vec3(0,0,0.3f));
        backRb.rotation =glm::quat(glm::vec3(0,0,0.3f));
        backRb1.rotation =glm::quat(glm::vec3(0,0,-0.3f));
        console.rb = &rb;


        std::vector<Collider*> colliders = { &slope1Collider, &slope2Collider, &floorCollider,
                                             &blockCollider, &leftSideCollider, &rightSideCollider, &leftSideCollider1, &rightSideCollider1, &backCollider, &backCollider1};
        for(auto& col: sphereColliders)
            colliders.push_back(&col);
        world.gravity = glm::vec3(0,-10.0f,0);
        world.enableResponse = true;
        world.setColliders(&colliders);
        cam.setPosition(glm::vec3(12,18,20));
        cam.setPitchAndYaw(-0.3f, 0.5f);
    }

    void update(float dt)
    {
        Scene::update(dt);
        world.stepWorld(dt, 5);

        for(auto& col: sphereColliders)
        {
            if(col.rb->position.y<-10.0f)
                col.rb->position =  glm::vec3(-2.5, 15, 0);
        }

        if(blockRb.position.y<-10.0f)
        {
            blockRb.position = glm::vec3(-5.0f, 10.0f, 0.0f);
            blockRb.setVelocity(glm::vec3(0,0,0));
            blockRb.setAngularVelocity(glm::vec3(0,0,0));
            blockRb.rotation = glm::quat(glm::vec3(0,0,0));
        }

        if(gMainWindow->getKeyDown(Qt::Key_Space))
        {
            blockRb.position = glm::vec3(-5.0f, 10.0f, 0.0f);
            blockRb.setVelocity(glm::vec3(0,0,0));
            blockRb.setAngularVelocity(glm::vec3(0,0,0));
            blockRb.rotation = glm::quat(glm::vec3(0,0,0));
        }

//        if(world.contacts.size()>0)
//        {
//            cube.meshes[1].setColor(glm::vec3(1,0,0));

//            for(int i =0;i<world.contacts.size();i++)
//            {
//                for(int j =0;j<world.contacts[i].points.size();j++)
//                {
//                    point.setPosition(world.contacts[i].points[j]);
//                    point.draw();
//                    if(j == 0)
//                    {
//                        drawLine(lineMesh, world.contacts[i].b->rb->position, world.contacts[i].b->rb->position+2.0f*world.contacts[i].normal);
//                        drawLine(lineMesh, world.contacts[i].a->rb->position, world.contacts[i].a->rb->position-2.0f*world.contacts[i].normal);
//                    }
//                }
//            }
//        }
//        else
//        {
//            cube.meshes[1].setColor(glm::vec3(0,1,0));
//        }

    }

    void updateDraw(float dt)
    {

        cube.meshes[1].setColor(glm::vec3(1, 0, 0));
        cube.meshes[0].setColor(glm::vec3(0.2, 0.2, 0.87f));
        cube.setPosition(backCollider.rb->position);
        cube.setRotation(backCollider.rb->rotation);
        cube.setScale(backCollider.scale);
        cube.draw();

        cube.setPosition(backCollider1.rb->position);
        cube.setRotation(backCollider1.rb->rotation);
        cube.setScale(backCollider1.scale);
        cube.draw();


        cube.setPosition(rightSideCollider1.rb->position);
        cube.setRotation(rightSideCollider1.rb->rotation);
        cube.setScale(rightSideCollider1.scale);
        cube.draw();

        cube.setPosition(leftSideCollider1.rb->position);
        cube.setRotation(leftSideCollider1.rb->rotation);
        cube.setScale(leftSideCollider1.scale);
        cube.draw();


        cube.setPosition(rightSideCollider.rb->position);
        cube.setRotation(rightSideCollider.rb->rotation);
        cube.setScale(rightSideCollider.scale);
        cube.draw();

        cube.setPosition(leftSideCollider.rb->position);
        cube.setRotation(leftSideCollider.rb->rotation);
        cube.setScale(leftSideCollider.scale);
        cube.draw();

        cube.meshes[1].setColor(glm::vec3(0, 1,0));
        cube.meshes[0].setColor(glm::vec3(1, 1, 1));

        cube.setPosition(blockCollider.rb->position);
        cube.setRotation(blockCollider.rb->rotation);
        cube.setScale(blockCollider.scale);
        cube.draw();

        cube.meshes[1].setColor(glm::vec3(1, 0, 0));
        cube.meshes[0].setColor(glm::vec3(0, 1, 0));

        cube.setPosition(slope1Collider.rb->position);
        cube.setRotation(slope1Collider.rb->rotation);
        cube.setScale(slope1Collider.scale);
        cube.draw();

        cube.setPosition(otherRb.position);
        cube.setRotation(otherRb.rotation);
        cube.setScale(slope2Collider.scale);
        cube.draw();





        sphere.meshes[0].setColor(glm::vec3(1, 1, 1));
        sphere.meshes[1].setColor(glm::vec3(0,0,0));
        for(auto& col: sphereColliders)
        {
        sphere.setPosition(col.rb->position);
        sphere.setRotation(col.rb->rotation);
        sphere.setScale(col.scale);
        sphere.draw();
        }

        plane.draw();
    }

};
