#include "Scene.h"
#include "Debug.h"

extern Shader* modelShader;
extern Shader* gridShader;


Scene::Scene(MainWindow& window) : window(window)
{
    glm::mat4 projection = glm::perspective((float)PI*0.33f, (float)window.width()/window.height(), 0.1f, 100.0f);
    glm::mat4 trans(1.0f);
    cam = Camera(glm::vec3(0,2,5));
    cam.updateView();

    modelShader->setVec3("lightDir", glm::vec3(-0.5f, -1.0f, -0.75));
    modelShader->setMat4("model", trans);
    modelShader->setMat4("view", cam.view);
    modelShader->setMat4("projection", projection);
    gridShader->setMat4("model", trans);
    gridShader->setMat4("view", cam.view);
    gridShader->setMat4("projection", projection);

    Mesh::initializeStaticArrays();
    lineMesh = Mesh::createLine();
    lineMesh.setColor(glm::vec3(0,0,1));
    Mesh pointMesh = Mesh::createSphere();
    pointMesh.setColor(glm::vec3(0.3,0.45,0.3));
    point = Entity(pointMesh);
    point.setScale(glm::vec3(0.2,0.2,0.2));

    plane = createGridedPlaneEntity(10);
    unitDirs = createUnitDirs();
    cube = createBoundedCubeEntity();
    cone = createBoundedConeEntity();
    sphere = createBoundedSphereEntity();
    cylinder = createBoundedCylinderEntity();
    capsule = createBoundedCapsuleEntity();
    arrow = createArrow();
}

void Scene::start()
{

}

void Scene::update(float dt)
{
    if(window.getKey(Qt::Key_A))
    {
        cam.translateRight(-dt);
    }
    if(window.getKey(Qt::Key_D))
    {
        cam.translateRight(dt);
    }
    if(window.getKey(Qt::Key_W))
    {
        cam.translateFwd(-dt);
    }
    if(window.getKey(Qt::Key_S))
    {
        cam.translateFwd(dt);
    }
    if(window.getMouse(Qt::MouseButton::LeftButton))
    {
        QPointF deltaPos = QCursor::pos()-window.mousePos;
        window.mousePos = QCursor::pos();
        cam.smoothRotateYaw(-dt*deltaPos.x());
        cam.smoothRotatePitch(-dt*deltaPos.y());
    }
    cam.smoothUpdateView();
    modelShader->setMat4("view", cam.view);
    gridShader->setMat4("view", cam.view);
}

CubeDropScene::CubeDropScene(MainWindow& window): Scene(window)
{

}

//void CubeDropScene::start()
//{
//    float mass = 1.0f;
//    float radius = 1.0f;
//    float inertia = (2.0f/5.0f)*mass*radius*radius;
//    rb = UniformRigidBody(mass, inertia);
//    otherRb = UniformRigidBody(mass, inertia);
//    // SphereBody otherRb(mass, 0.5f);
//    collider = CubeCollider(glm::vec3(0.5f,0.5f,0.5f));
//    otherCollider = CubeCollider(glm::vec3(10.0f,0.1f,10.0f));
//    collider.rb = &rb;
//    otherCollider.rb = &otherRb;
//    rb.position = glm::vec3(0, 5, 0);
//    rb.dynamic = true;
//    otherRb.position = glm::vec3(0,-0.1f, 0);
//    otherRb.dynamic = false;
//    rb.rotation = glm::quat(glm::vec3(PI/3.0f,0.0f, PI/3.0f));

//    std::vector<Collider*> colliders = {&otherCollider, &collider};
//    world.gravity = glm::vec3(0,-1,0);
//    world.setColliders(&colliders);
//}

//void CubeDropScene::update(float dt)
//{
//    Scene::update(dt);
//    if(window.getKey(Qt::Key_Right))
//    {
//        //rb.setVelocity(1.0f*cam.getRight());
//        rb.addForce(2.0f*cam.getRight());
//    }
//    if(window.getKey(Qt::Key_Left))
//    {
//        //rb.setVelocity(-1.0f*cam.getRight());
//        rb.addForce(-2.0f*cam.getRight());
//    }
//    if(window.getKey(Qt::Key_E))
//    {
//        rb.setVelocity(1.0f*glm::vec3(0,1,0));
//    }
//    if(window.getKey(Qt::Key_Q))
//    {
//        rb.setVelocity(-1.0f*glm::vec3(0,1,0));
//    }
//    if(window.getKey(Qt::Key_Up))
//    {
//        //rb.setVelocity(glm::cross(glm::vec3(0,1,0), cam.getRight()));
//        rb.addForce(glm::cross(glm::vec3(0,2,0), cam.getRight()));

//    }
//    if(window.getKey(Qt::Key_Down))
//    {
//        //rb.setVelocity(glm::cross(glm::vec3(0,-1,0), cam.getRight()));
//        rb.addForce(glm::cross(glm::vec3(0,-2,0), cam.getRight()));

//    }
//    if(window.getGetDown(Qt::Key_Space))
//    {
//        rb.addForce(glm::vec3(0,800,0));
//    }
//    if(window.getGetDown(Qt::Key_R))
//    {
//        rb.setVelocity(glm::vec3(0,0,0));
//        rb.position = glm::vec3(0,2,0);
//        rb.setAngularVelocity(glm::vec3(0,0,0));
//        rb.rotation = glm::quat(glm::vec3(2*PI/(rand()%8+1), 2*PI/(rand()%8+1), 2*PI/(rand()%8+1)));

//        otherRb.setVelocity(glm::vec3(0,0,0));
//        otherRb.position = glm::vec3(0,2,2);
//        otherRb.setAngularVelocity(glm::vec3(0,0,0));
//    }
//    if(window.getGetDown(Qt::Key_1))
//    {
//        rb.setVelocity(glm::vec3(0,0,0));
//    }


//    //world.stepWorld(0.0009f);
//    world.stepWorld(dt);

//    if(collider.collisionDetected)
//    {
//        cube.meshes[1].setColor(glm::vec3(1,0,0));

//        for(int i =0;i<world.contacts.size();i++)
//        {
//            point.setPosition(world.contacts[i].points[i]);
//            point.draw();
//            drawLine(lineMesh, rb.position, rb.position+2.0f*world.contacts[i].normal);

//        }


//    }
//    else
//    {
//        cube.meshes[1].setColor(glm::vec3(0,1,0));
//    }
//    cube.setPosition(rb.position);
//    cube.setRotation(rb.rotation);
//    cube.setScale(collider.scale);
//    cube.draw();

//    cube.setPosition(otherRb.position);
//    cube.setRotation(otherRb.rotation);
//    cube.setScale(otherCollider.scale);
//    cube.draw();


//    plane.draw();

//}


void CubeDropScene::start()
{
    float mass = 1.0f;
    float radius = 1.0f;
    float inertia = (2.0f/5.0f)*mass*radius*radius;
    rb = UniformRigidBody(mass, inertia);
    otherRb = UniformRigidBody(mass, inertia);
    // SphereBody otherRb(mass, 0.5f);
    collider = CubeCollider(glm::vec3(0.5f,0.5f,0.5f));
    otherCollider = CubeCollider(glm::vec3(0.5f,0.5f,0.5f));
    collider.rb = &rb;
    otherCollider.rb = &otherRb;
    rb.position = glm::vec3(0, 2, 0);
    //rb.dynamic = false;
    otherRb.position = glm::vec3(0,2, 2);
    //otherRb.dynamic = false;
    rb.rotation = glm::quat(glm::vec3(PI/3.0f,0.0f, 0.0f));
    otherRb.rotation = glm::quat(glm::vec3(0,PI/3.0f, 0.0f));


    std::vector<Collider*> colliders = {&otherCollider, &collider};
    world.gravity = glm::vec3(0,0,0);
    world.setColliders(&colliders);
}

void CubeDropScene::update(float dt)
{
    Scene::update(dt);
    rb.setVelocity(glm::vec3());
    if(window.getKey(Qt::Key_Right))
    {
        rb.setVelocity(1.0f*cam.getRight());
        //rb.addForce(2.0f*cam.getRight());
    }
    if(window.getKey(Qt::Key_Left))
    {
        rb.setVelocity(-1.0f*cam.getRight());
        //rb.addForce(-2.0f*cam.getRight());
    }
    if(window.getKey(Qt::Key_E))
    {
        rb.setVelocity(1.0f*glm::vec3(0,1,0));
    }
    if(window.getKey(Qt::Key_Q))
    {
        rb.setVelocity(-1.0f*glm::vec3(0,1,0));
    }
    if(window.getKey(Qt::Key_Up))
    {
        rb.setVelocity(glm::cross(glm::vec3(0,1,0), cam.getRight()));
        //rb.addForce(glm::cross(glm::vec3(0,2,0), cam.getRight()));

    }
    if(window.getKey(Qt::Key_Down))
    {
        rb.setVelocity(glm::cross(glm::vec3(0,-1,0), cam.getRight()));
        //rb.addForce(glm::cross(glm::vec3(0,-2,0), cam.getRight()));

    }
    if(window.getGetDown(Qt::Key_Space))
    {
        rb.addForce(glm::vec3(0,800,0));
    }
    if(window.getGetDown(Qt::Key_R))
    {
        rb.setVelocity(glm::vec3(0,0,0));
        rb.position = glm::vec3(0,2,0);
        rb.setAngularVelocity(glm::vec3(0,0,0));
        rb.rotation = glm::quat(glm::vec3(2*PI/(rand()%8+1), 2*PI/(rand()%8+1), 2*PI/(rand()%8+1)));

        otherRb.setVelocity(glm::vec3(0,0,0));
        otherRb.position = glm::vec3(0,2,2);
        otherRb.setAngularVelocity(glm::vec3(0,0,0));
    }
    if(window.getGetDown(Qt::Key_1))
    {
        rb.setVelocity(glm::vec3(0,0,0));
    }


    //world.stepWorld(0.0009f);
    world.stepWorld(dt);

    if(collider.collisionDetected)
    {
        cube.meshes[1].setColor(glm::vec3(1,0,0));

        for(int i =0;i<world.contacts.size();i++)
        {
            point.setPosition(world.contacts[i].points[i]);
            point.draw();
            drawLine(lineMesh, rb.position, rb.position+2.0f*world.contacts[i].normal);

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

    cube.setPosition(otherRb.position);
    cube.setRotation(otherRb.rotation);
    cube.setScale(otherCollider.scale);
    cube.draw();


    plane.draw();

}
