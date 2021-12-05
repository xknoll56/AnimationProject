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
    //SphereCollider sphereCollider;
    BoxCollider collider;
    UniformRigidBody boxRb;
    std::vector<bool> inContact;
    int widthBoxes = 10;


public:
    VaccumeScene()
    {

    }

    void start()
    {
        float mass = 1.0f;
        float radius = 1.0f;
        float inertia = 1.0f;
        BoxColliders.reserve(widthBoxes*widthBoxes);
        rbs.reserve(widthBoxes*widthBoxes);
        inContact.reserve(widthBoxes*widthBoxes);
        for(int i = 0;i<widthBoxes;i++)
        {
            for(int j = 0; j<widthBoxes;j++)
            {
            UniformRigidBody rb(mass, inertia);
            rb.position = glm::vec3(i%widthBoxes*1.05f, j%widthBoxes*1.05f, 0);
            rb.dynamic = true;
            rb.atRest = true;
            rbs.push_back(rb);
            BoxColliders.push_back(BoxCollider(glm::vec3(0.5f,0.5f,0.5f)));
            BoxColliders[BoxColliders.size()-1].rb = &rbs[rbs.size()-1];
            inContact.push_back(false);
            }
        }
        //inertia = (2.0f/5.0f)*10.0f*radius*radius;
        boxRb = UniformRigidBody(10.0f, 10.0f);
        // SphereBody otherRb(mass, 0.5f);

        collider = BoxCollider(glm::vec3(1.0f,1.0f,1.0f));
        collider.rb = &boxRb;
        boxRb.position = glm::vec3(2,8,10);

       // rb.rotation = glm::quat(glm::vec3(0.0f,0.0f, 0.0f));

        std::vector<Collider*> colliders = {  &collider};
        for(auto& col: BoxColliders)
            colliders.push_back(&col);
        world.gravity = glm::vec3(0,0.0f,0);
        world.enableResponse = true;
        world.setColliders(&colliders);
        cam.setPosition(glm::vec3((widthBoxes*0.5f)*1.05f, (widthBoxes*0.5f)*1.0f, 10));
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
            boxRb.position = cam.getPosition();
            boxRb.setVelocity(-cam.getFwd()*30.0f);
            boxRb.setAngularVelocity(glm::vec3(1.0f,1.0f,1.0f));
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

        cube.setPosition(boxRb.position);
        cube.setRotation(boxRb.rotation);
        cube.setScale(collider.scale);
        cube.draw();


    }

};
