#pragma once
#include "Scene.h"
extern void drawLine(Mesh& line, glm::vec3 from, glm::vec3 to);
extern MainWindow* gMainWindow;

class VaccumeScene: public Scene
{
private:
    PhysicsWorld world;
    std::vector<BoxCollider> BoxColliders;
    std::vector<UniformRigidBody> rbs;
    SphereCollider sphereCollider;
    UniformRigidBody sphereRb;
    std::vector<bool> inContact;


public:
    VaccumeScene()
    {

    }

    void start()
    {
        float mass = 1.0f;
        float radius = 1.0f;
        float inertia = 1.0f;
        BoxColliders.reserve(100);
        rbs.reserve(100);
        inContact.reserve(100);
        for(int i = 0;i<6;i++)
        {
            for(int j = 0; j<6;j++)
            {
            UniformRigidBody rb(mass, inertia);
            rb.position = glm::vec3(i%6*1.05f, j%6*1.05f, 0);
            rb.dynamic = true;
            rb.atRest = true;
            rbs.push_back(rb);
            BoxColliders.push_back(BoxCollider(glm::vec3(0.5f,0.5f,0.5f)));
            BoxColliders[BoxColliders.size()-1].rb = &rbs[rbs.size()-1];
            inContact.push_back(false);
            }
        }
        inertia = (2.0f/5.0f)*10.0f*radius*radius;
        sphereRb = UniformRigidBody(10.0f, inertia);
        // SphereBody otherRb(mass, 0.5f);

        sphereCollider = SphereCollider(radius);
        sphereCollider.rb = &sphereRb;
        sphereRb.position = glm::vec3(2,8,10);

       // rb.rotation = glm::quat(glm::vec3(0.0f,0.0f, 0.0f));

        std::vector<Collider*> colliders = {  &sphereCollider};
        for(auto& col: BoxColliders)
            colliders.push_back(&col);
        world.gravity = glm::vec3(0,0.0f,0);
        world.enableResponse = true;
        world.setColliders(&colliders);
    }

    void update(float dt)
    {
        Scene::update(dt);
        for(auto& col: BoxColliders)
            col.rb->atRest = false;
        world.stepWorld(dt);
        selectRigidBody(world);

        if(gMainWindow->getKeyDown(Qt::Key_Space))
        {
            sphereRb.position = cam.getPosition();
            sphereRb.setVelocity(-cam.getFwd()*30.0f);
        }



    }

    void updateDraw(float dt)
    {
        std::vector<Collider*> contactColliders;
        if(world.contacts.size()>0)
        {
            //cube.meshes[1].setColor(glm::vec3(1,0,0));

            for(int i =0;i<world.contacts.size();i++)
            {
                for(int j =0;j<world.contacts[i].points.size();j++)
                {
                    contactColliders.push_back(world.contacts[i].a);
                    contactColliders.push_back(world.contacts[i].b);
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


        for(auto& col: BoxColliders)
        {
            bool found = false;
            for(Collider* c: contactColliders)
            {
                BoxCollider* cc = dynamic_cast<BoxCollider*>(c);
                if(cc)
                {
                    if(cc==&col)
                    {
                        found = true;
                        break;
                    }
                }
            }

            if(found)
                cube.meshes[1].setColor(glm::vec3(1,0,0));
            else
                cube.meshes[1].setColor(glm::vec3(0,1,0));;
            if(col.rb == selectedRb)
                cube.meshes[1].setColor(glm::vec3(0,0,1));
            cube.setPosition(col.rb->position);
            cube.setRotation(col.rb->rotation);
            cube.setScale(col.scale);
            cube.draw();
        }

        sphere.setPosition(sphereRb.position);
        sphere.setRotation(sphereRb.rotation);
        sphere.setScale(sphereCollider.scale);
        sphere.draw();


    }

};
