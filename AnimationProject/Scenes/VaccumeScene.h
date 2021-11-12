#pragma once
#include "Scene.h"
extern void drawLine(Mesh& line, glm::vec3 from, glm::vec3 to);
extern MainWindow* gMainWindow;

class VaccumeScene: public Scene
{
private:
    PhysicsWorld world;
    std::vector<CubeCollider> cubeColliders;
    std::vector<UniformRigidBody> rbs;
    SphereCollider sphereCollider;
    UniformRigidBody sphereRb;



public:
    VaccumeScene()
    {

    }

    void start()
    {
        float mass = 1.0f;
        float radius = 1.0f;
        float inertia = (2.0f/5.0f)*mass*radius*radius;
        cubeColliders.reserve(25);
        rbs.reserve(25);
        for(int i = 0;i<5;i++)
        {
            for(int j = 0; j<5;j++)
            {
            UniformRigidBody rb(mass, inertia);
            rb.position = glm::vec3(i%5, j%5, 0);
            rb.dynamic = true;
            rbs.push_back(rb);
            cubeColliders.push_back(CubeCollider(glm::vec3(0.5f,0.5f,0.5f)));
            cubeColliders[cubeColliders.size()-1].rb = &rbs[rbs.size()-1];
            }
        }
        sphereRb = UniformRigidBody(mass, inertia);
        // SphereBody otherRb(mass, 0.5f);

        sphereCollider = SphereCollider(0.5f);
        sphereCollider.rb = &sphereRb;
        sphereRb.position = glm::vec3(2,8,0);

       // rb.rotation = glm::quat(glm::vec3(0.0f,0.0f, 0.0f));

        std::vector<Collider*> colliders = {  &sphereCollider};
        for(auto& col: cubeColliders)
            colliders.push_back(&col);
        world.gravity = glm::vec3(0,0.0f,0);
        world.enableResponse = true;
        world.setColliders(&colliders);
    }

    void update(float dt)
    {
        Scene::update(dt);
        for(auto& col: cubeColliders)
            col.rb->atRest = false;
        world.stepWorld(dt, 3);

        if(gMainWindow->getKeyDown(Qt::Key_Space))
        {
            sphereRb.position = cam.getPosition();
            sphereRb.setVelocity(-cam.getFwd()*30.0f);
        }



    }

    void updateDraw(float dt)
    {
        for(auto& col: cubeColliders)
        {
            cube.setPosition(col.rb->position);
            cube.setRotation(col.rb->rotation);
            cube.setScale(col.scale);
            cube.draw();
        }

        sphere.setPosition(sphereRb.position);
        sphere.setRotation(sphereRb.rotation);
        sphere.setScale(sphereCollider.scale);
        sphere.draw();

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

};
