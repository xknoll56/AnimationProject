#pragma once
#include "Scene.h"
extern void drawLine(Mesh& line, glm::vec3 from, glm::vec3 to);
extern MainWindow* gMainWindow;

class CollisionTestScene: public Scene
{
private:
    PhysicsWorld world;
    BoxCollider collider;
    BoxCollider otherCollider;
    UniformRigidBody rb;
    UniformRigidBody otherRb;
    SphereCollider sphereCollider;
    UniformRigidBody sphereRb;
    glm::vec3 castDir;
    glm::vec3 hitPosition;
    glm::vec3 worldCoords;
    bool collisionDetectedOnCollider;
    bool collisionDetectedOnOtherCollider;
    bool collisionDetectedOnSphereCollider;


public:
    CollisionTestScene()
    {

    }
    void start()
    {

        float mass = 1.0f;
        float radius = 0.5f;
        float inertia = 1.0f;
        rb = UniformRigidBody(mass, inertia);
        otherRb = UniformRigidBody(mass, inertia);
        // SphereBody otherRb(mass, 0.5f);
        collider = BoxCollider(glm::vec3(2.5f,0.5f,2.5f));
        otherCollider = BoxCollider(glm::vec3(1.0f,0.5f,0.5f));

        collider.rb = &rb;
        otherCollider.rb = &otherRb;
        rb.position = glm::vec3(0, 1.5,4);
        rb.dynamic = true;
        otherRb.position = glm::vec3(0,2.0f, -4);
       // otherRb.dynamic = false;
        //rb.rotation = glm::quat(glm::vec3(PI/3.0f,0.0f, PI/3.0f));
        rb.rotation = glm::quat(glm::vec3(0.0f,0.0f, 0.0f));

        sphereCollider = SphereCollider(1.0f);
        sphereRb = UniformRigidBody(mass, inertia);
        sphereCollider.rb = &sphereRb;
        sphereRb.position = glm::vec3(5.0f, 2.0f, 0.0f);
        sphereRb.setAngularVelocity(glm::vec3(0.1,0.2,0.3));
        castDir = glm::vec3(1,0,0);



        std::vector<Collider*> colliders = { &collider, &sphereCollider, &otherCollider};
        world.gravity = glm::vec3(0,0.0f,0);
        world.enableResponse = false;
        world.setColliders(&colliders);
        cam.setPosition(glm::vec3(0, 5, 8));

        selectedRb = &rb;

    }
    void update(float dt)
    {
        Scene::update(dt);
        selectedRb->setVelocity(glm::vec3());
        if(gMainWindow->getKey(Qt::Key_Right))
        {
            selectedRb->setVelocity(1.0f*cam.getRight());
            //rb.addForce(2.0f*cam.getRight());
        }
        if(gMainWindow->getKey(Qt::Key_Left))
        {
            selectedRb->setVelocity(-1.0f*cam.getRight());
            //rb.addForce(-2.0f*cam.getRight());
        }
        if(gMainWindow->getKey(Qt::Key_E))
        {
            selectedRb->setVelocity(1.0f*glm::vec3(0,1,0));
        }
        if(gMainWindow->getKey(Qt::Key_Q))
        {
            selectedRb->setVelocity(-1.0f*glm::vec3(0,1,0));
        }
        if(gMainWindow->getKey(Qt::Key_Up))
        {
            selectedRb->setVelocity(glm::cross(glm::vec3(0,1,0), cam.getRight()));
            //rb.addForce(glm::cross(glm::vec3(0,2,0), cam.getRight()));

        }
        if(gMainWindow->getKey(Qt::Key_Down))
        {
            selectedRb->setVelocity(glm::cross(glm::vec3(0,-1,0), cam.getRight()));
            //rb.addForce(glm::cross(glm::vec3(0,-2,0), cam.getRight()));

        }
        if(gMainWindow->getKeyDown(Qt::Key_Space))
        {
            selectedRb->addForce(glm::vec3(0,800,0));
        }
        if(gMainWindow->getKeyDown(Qt::Key_R))
        {
            selectedRb->setVelocity(glm::vec3(0,0,0));
            //rb.position = glm::vec3(0,1,0);
            selectedRb->setAngularVelocity(glm::vec3(0.01,0.02,0.03));
            selectedRb->rotation = glm::quat(glm::vec3(2*PI/(rand()%8+1), 2*PI/(rand()%8+1), 2*PI/(rand()%8+1)));

        }
        selectRigidBody(world);

        world.stepWorld(dt);


        collisionDetectedOnCollider = false;
        collisionDetectedOnOtherCollider = false;
        collisionDetectedOnSphereCollider = false;

        if(world.contacts.size()>0)
        {
            for(int i =0;i<world.contacts.size();i++)
            {
                for(int j =0;j<world.contacts[i].points.size();j++)
                {
                    point.setPosition(world.contacts[i].points[j]);
                    point.draw();
                    if(j == 0)
                    {

                        glm::vec3 normal = 2.0f*world.contacts[i].normal;
                        glm::vec3 toPosition = world.contacts[i].b->rb->position+normal;
                        drawLine(lineMesh, world.contacts[i].b->rb->position,world.contacts[i].b->rb->position+ normal);
                        drawLine(lineMesh, world.contacts[i].a->rb->position,world.contacts[i].a->rb->position -normal);
                    }

                    BoxCollider* bc = dynamic_cast<BoxCollider*>(world.contacts[i].a);
                    if(bc)
                    {
                        if(bc == &collider)
                            collisionDetectedOnCollider = true;
                        else if(bc == &otherCollider)
                            collisionDetectedOnOtherCollider = true;
                    }

                    bc = dynamic_cast<BoxCollider*>(world.contacts[i].b);
                    if(bc)
                    {
                        if(bc == &collider)
                            collisionDetectedOnCollider = true;
                        else if(bc == &otherCollider)
                            collisionDetectedOnOtherCollider = true;
                    }

                    SphereCollider* sc = dynamic_cast<SphereCollider*>(world.contacts[i].a);
                    if(sc)
                    {
                        if(sc == &sphereCollider)
                            collisionDetectedOnSphereCollider = true;;
                    }

                    sc = dynamic_cast<SphereCollider*>(world.contacts[i].b);
                    if(sc)
                    {
                        if(sc == &sphereCollider)
                            collisionDetectedOnSphereCollider = true;
                    }


                }
            }
        }

//        RayCastData data;
//        //qDebug() << elapsedTime;
//        castDir = glm::vec3(glm::cos(elapsedTime/10.0f), 0.0f, glm::sin(elapsedTime/10.0f));
//        glm::vec3 castPoint(0, 2, 0);
//        point.setPosition(castPoint);
//        point.draw();
//        if(world.raycastAll(castPoint, castDir, data))
//        {
//            lineMesh.setColor(glm::vec3(1,0,0));
//            drawLine(lineMesh,castPoint, data.point);
//            point.setPosition(data.point);
//            point.draw();
//        }
//        else
//        {
//            lineMesh.setColor(glm::vec3(0,0,1));
//            drawLine(lineMesh, castPoint, castPoint+castDir);
//        }



    }

    void updateDraw(float dt)
    {

        point.setPosition(hitPosition);
        point.draw();

        if(&rb==selectedRb)
        {
            cube.meshes[1].setColor(glm::vec3(0,0,1));
        }
        else
        {
            cube.meshes[1].setColor(glm::vec3(0,1,0));
        }
        //override the color if in contact
        if(collisionDetectedOnCollider)
            cube.meshes[1].setColor(glm::vec3(1,0,0));

        cube.setPosition(rb.position);
        cube.setRotation(rb.rotation);
        cube.setScale(collider.scale);
        cube.draw();

        if(&sphereRb==selectedRb)
        {
            sphere.meshes[1].setColor(glm::vec3(0,0,1));
        }
        else
        {
            sphere.meshes[1].setColor(glm::vec3(0,1,0));
        }
        //override the color if in contact
        if(collisionDetectedOnSphereCollider)
            sphere.meshes[1].setColor(glm::vec3(1,0,0));
        sphere.setPosition(sphereRb.position);
        sphere.setRotation(sphereRb.rotation);
        sphere.setScale(sphereCollider.scale);
        sphere.draw();


        if(&otherRb==selectedRb)
        {
            cube.meshes[1].setColor(glm::vec3(0,0,1));
        }
        else
        {
            cube.meshes[1].setColor(glm::vec3(0,1,0));
        }
        //override the color if in contact
        if(collisionDetectedOnOtherCollider)
            cube.meshes[1].setColor(glm::vec3(1,0,0));
        cube.setPosition(otherRb.position);
        cube.setRotation(otherRb.rotation);
        cube.setScale(otherCollider.scale);
        cube.draw();
        plane.draw();

        drawCrosshair();
    }
};
