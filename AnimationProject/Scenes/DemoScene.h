#pragma once
#include "Scene.h"
extern void drawLine(Mesh& line, glm::vec3 from, glm::vec3 to);
extern MainWindow* gMainWindow;

class DemoScene: public Scene
{


private:


    struct AnimationTimer
    {
        float timer = 0.0f;
        float timeMax = 3.0f;
        glm::vec3 spawnPosition;
        bool shouldUpdate = false;
        bool respawned = false;
    };

    struct SpawnZone
    {
        float xMin, xSize, zMin, zSize, height, ySize;
    };



    PhysicsWorld world;
    BoxCollider slope1Collider;
    BoxCollider slope2Collider;
    BoxCollider floorCollider;
    BoxCollider leftSideCollider;
    BoxCollider rightSideCollider;
    BoxCollider leftSideCollider1;
    BoxCollider rightSideCollider1;
    BoxCollider backCollider;
    BoxCollider backCollider1;

    std::vector<SphereCollider> sphereColliders;
    std::vector<BoxCollider> boxColliders;
    std::vector<AnimationTimer> animationTimers;
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
    std::vector<UniformRigidBody> boxRbs;
    std::vector<glm::vec3> boxSpawns;
    UniformRigidBody floorRb;
    SpawnZone spawnZone;
    int numBodies = 15;


public:
    DemoScene()
    {

    }

    glm::vec3 getRandomPointInSpawnZone()
    {
        float xOffset = (rand()%1000/1000.0f)*spawnZone.xSize;
        float zOffset = (rand()%1000/1000.0f)*spawnZone.zSize;
        float yOffset = (rand()%1000/1000.0f)*spawnZone.ySize;
        return glm::vec3(spawnZone.xMin+xOffset, spawnZone.height+yOffset, spawnZone.zMin+zOffset);
    }

    void respawnAnimation(float dt, int sphereIndex)
    {
        AnimationTimer *animTimer = &animationTimers[sphereIndex];
        animTimer->timer += dt;
        cone.setRotation(glm::quat(glm::vec3(0, 5.0f*animTimer->timer, 0)));
        cone.setPosition(animTimer->spawnPosition);


        float s = animTimer->timer;
        if(animTimer->timer >= animTimer->timeMax*0.5f && !animTimer->respawned)
        {
            sphereColliders[sphereIndex].rb->position = animTimer->spawnPosition;
            animTimer->respawned = true;
             glm::vec3 velocity((rand()%100-50.0f)/20, -2.0f, (rand()%100-50.0f)/20);
             sphereColliders[sphereIndex].rb->setVelocity(velocity);
             sphereColliders[sphereIndex].scale = glm::vec3(0,0,0);
        }
        else if(animTimer->timer >= animTimer->timeMax*0.5f && animTimer->timer<animTimer->timeMax)
        {
            float sphereScale = (animTimer->timer - animTimer->timeMax*0.5f)*sphereColliders[sphereIndex].radius/(1.5f*0.5f);
            sphereColliders[sphereIndex].scale = glm::vec3(sphereScale, sphereScale, sphereScale);
            s = animTimer->timeMax- animTimer->timer;
        }
        else if(animTimer->timer>=animTimer->timeMax)
        {
            sphereColliders[sphereIndex].scale = glm::vec3(sphereColliders[sphereIndex].radius*2,
                                                           sphereColliders[sphereIndex].radius*2,
                                                           sphereColliders[sphereIndex].radius*2);
            s = animTimer->timeMax- animTimer->timer;
            animTimer->timer = 0.0f;
            animTimer->shouldUpdate = false;
            animTimer->respawned = false;
        }
        cone.setScale(glm::vec3(s,s,s));
        cone.draw();
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
        boxRbs.reserve(4);
        boxColliders.reserve(4);
        spawnZone.height = 18;
        spawnZone.xMin = -8;
        spawnZone.zMin = -2.5f;
        spawnZone.xSize = 5.0f;
        spawnZone.zSize = 5.0f;
        spawnZone.ySize = 5.0f;
        for(int i  =0; i<numBodies; i++)
        {
            UniformRigidBody sphereRb(mass, inertia);
            sphereRbs.push_back(sphereRb);
            float radius = 0.5f+(rand()%100)/200.0f;
            SphereCollider sc(radius);
            sphereColliders.push_back(sc);
            sphereColliders[sphereColliders.size()-1].rb = &sphereRbs[sphereRbs.size()-1];
            animationTimers.push_back(AnimationTimer());
            animationTimers[animationTimers.size()-1].spawnPosition = sphereRbs[sphereRbs.size()-1].position;
            animationTimers[animationTimers.size()-1].shouldUpdate = true;
            animationTimers[animationTimers.size()-1].spawnPosition = getRandomPointInSpawnZone();
        }
        for(int i  =0; i<1; i++)
        {
            UniformRigidBody boxRb(mass, inertia);
            boxRbs.push_back(boxRb);
            BoxCollider bc(glm::vec3(0.5f, 0.5f,1.5f));
            boxColliders.push_back(bc);
            boxColliders[boxColliders.size()-1].rb = &boxRbs[boxRbs.size()-1];
            boxSpawns.push_back(glm::vec3(-8, 8, 0));
            boxColliders[boxColliders.size()-1].rb->position = boxSpawns[boxSpawns.size()-1];
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
        slope1Collider = BoxCollider(glm::vec3(10.0f,0.1f,5.0f));
        slope2Collider = BoxCollider(glm::vec3(10.0f,0.1f,5.0f));
        floorCollider = BoxCollider(glm::vec3(10.0f, 0.1f,10.0f));
        leftSideCollider = BoxCollider(glm::vec3(10.0f, 1.0f, 0.1f));
        rightSideCollider = BoxCollider(glm::vec3(10.0f, 1.0f, 0.1f));
        leftSideCollider1 = BoxCollider(glm::vec3(10.0f, 1.0f, 0.1f));
        rightSideCollider1 = BoxCollider(glm::vec3(10.0f, 1.0f, 0.1f));
        backCollider = BoxCollider(glm::vec3(0.1f, 1.0f, 5.0f));
         backCollider1 = BoxCollider(glm::vec3(0.1f, 1.0f, 5.0f));
        leftSideCollider1.rb = &leftSideRb1;
        rightSideCollider1.rb = &rightSideRb1;
        leftSideCollider.rb = &leftSideRb;
        rightSideCollider.rb = &rightSideRb;
        backCollider.rb = &backRb;
        backCollider1.rb = &backRb1;

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
                                              &leftSideCollider, &rightSideCollider, &leftSideCollider1, &rightSideCollider1, &backCollider, &backCollider1};
        for(auto& col: sphereColliders)
            colliders.push_back(&col);
//        for(auto& col: boxColliders)
//            colliders.push_back(&col);
        world.gravity = glm::vec3(0,-10.0f,0);
        world.enableResponse = true;
        world.setColliders(&colliders);
        cam.setPosition(glm::vec3(12,18,20));
        cam.setPitchAndYaw(-0.3f, 0.5f);

        cone.meshes[0].setColor(glm::vec3(0,1,1));
        cone.meshes[1].setColor(glm::vec3(0.75, 0, 0));
    }

    void update(float dt)
    {
        Scene::update(dt);
        world.stepWorld(dt, 5);

        selectRigidBody(world);

        for(int i = 0; i<sphereColliders.size(); i++)
        {
            SphereCollider col = sphereColliders[i];
            if(col.rb->position.y<-10.0f)
            {
                animationTimers[i].shouldUpdate = true;
            }
        }

//        for(int i = 0; i<boxColliders.size(); i++)
//        {
//            BoxCollider col = boxColliders[i];
//            if(col.rb->position.y<-10.0f)
//            {
//                boxColliders[i].rb->position = boxSpawns[i];
//                boxColliders[i].rb->setVelocity(glm::vec3(0,0,0));
//                boxColliders[i].rb->atRest = false;
//                boxColliders[i].rb->restingContact = false;
//                boxColliders[i].rb->setAngularVelocity(glm::vec3(0,0,0));
//            }
//        }



        for(int i = 0; i<animationTimers.size(); i++)
        {
            if(animationTimers[i].shouldUpdate)
                respawnAnimation(dt, i);
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
        drawBoundedCollider(backCollider, glm::vec3(0.2, 0.2, 0.87f), glm::vec3(1, 0, 0));
        drawBoundedCollider(backCollider1);
        drawBoundedCollider(rightSideCollider);
        drawBoundedCollider(rightSideCollider1);
        drawBoundedCollider(leftSideCollider);
        drawBoundedCollider(leftSideCollider1);
        drawBoundedCollider(slope1Collider, glm::vec3(0, 1, 0), glm::vec3(1, 0, 0));
        drawBoundedCollider(slope2Collider);
        sphere.meshes[0].setColor(glm::vec3(1, 1, 1));
        sphere.meshes[1].setColor(glm::vec3(0,0,0));
        for(auto& col: sphereColliders)
        {
            if(col.rb == selectedRb)
                sphere.meshes[1].setColor(glm::vec3(1,0,0));
            else
                sphere.meshes[1].setColor(glm::vec3(0,0,0));
            drawBoundedCollider(col);
        }

//        for(auto& col: boxColliders)
//        {
//            if(col.rb == selectedRb)
//                cube.meshes[1].setColor(glm::vec3(1,0,0));
//            else
//                cube.meshes[1].setColor(glm::vec3(0,0,0));
//            cube.meshes[0].setColor(glm::vec3(1,1,1));
//            drawBoundedCollider(col );
//        }

        plane.draw();
    }

};
