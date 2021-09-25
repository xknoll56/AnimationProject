

//int main(int argc, char *argv[])
//{
//    QGuiApplication app(argc, argv);

//    QSurfaceFormat format;
//    format.setSamples(4);
//    format.setDepthBufferSize(24);
//    format.setMajorVersion(4);
//    format.setMinorVersion(5);
//    format.setSwapInterval(0);
//    format.setSwapBehavior(QSurfaceFormat::SwapBehavior::DefaultSwapBehavior);
//    format.setProfile(QSurfaceFormat::CoreProfile);

//    MainWindow window;
//    window.setTitle("Animation Project");
//    window.setFormat(format);
//    window.setSurfaceType(QWindow::OpenGLSurface);
//    window.resize(1280, 720);
//    window.setKeyboardGrabEnabled(true);
//    window.show();
//    CloseEventFilter closeFilter(&window);
//    window.installEventFilter(&closeFilter);


//    QOpenGLContext* context = new QOpenGLContext(&window);
//    context->setFormat(window.requestedFormat());
//    context->create();
//    context->makeCurrent(&window);

//    //app.processEvents();
//    QOpenGLPaintDevice* paintDevice = new QOpenGLPaintDevice;
//    paintDevice->setSize(window.size() * window.devicePixelRatio());
//    paintDevice->setDevicePixelRatio(window.devicePixelRatio());

//    //painter->setWorldMatrixEnabled(false);

//    openglFunctions = context->versionFunctions<QOpenGLFunctions_4_5_Core>();
//    if(!openglFunctions)
//    {
//        qDebug("Could not obtain required version of opengl");
//        app.exit();
//    }
//    openglFunctions->initializeOpenGLFunctions();



//    window.openglInitialized = true;
//    openglFunctions->glViewport(0, 0, window.width() * window.devicePixelRatio(), window.height() * window.devicePixelRatio());
//    openglFunctions->glEnable(GL_DEPTH_TEST);
//    openglFunctions->glEnable(GL_CULL_FACE);
//    openglFunctions->glEnable(GL_LINE_SMOOTH);
//    openglFunctions->glEnable(GL_LINE_WIDTH);
//    openglFunctions->glLineWidth(2.5f);
//    openglFunctions->glDisable(GL_LIGHTING);


//    Shader modelShaderObj("model.vert", "model.frag");
//    Shader gridShaderObj("grid.vert", "grid.frag");
//    modelShader = &modelShaderObj;
//    gridShader = &gridShaderObj;

//    glm::vec3 euler(0,0,0);
//    glm::mat4 trans(1.0f);
//    modelShader->insertUniform("model");
//    modelShader->insertUniform("view");
//    modelShader->insertUniform("projection");
//    modelShader->insertUniform("color");
//    modelShader->insertUniform("lightDir");
//    gridShader->insertUniform("model");
//    gridShader->insertUniform("view");
//    gridShader->insertUniform("projection");
//    gridShader->insertUniform("color");
//    //modelShader.setVec3("color", glm::vec3(1,1,1));
//    glm::mat4 projection = glm::perspective((float)PI*0.33f, (float)window.width()/window.height(), 0.1f, 100.0f);

//    Camera cam(glm::vec3(0,2,5));
//    cam.updateView();

//    modelShader->setVec3("lightDir", glm::vec3(-0.5f, -1.0f, -0.75));
//    modelShader->setMat4("model", trans);
//    modelShader->setMat4("view", cam.view);
//    modelShader->setMat4("projection", projection);
//    gridShader->setMat4("model", trans);
//    gridShader->setMat4("view", cam.view);
//    gridShader->setMat4("projection", projection);

//    Mesh::initializeStaticArrays();
//    lineMesh = Mesh::createLine();
//    lineMesh.setColor(glm::vec3(0,0,1));

//    Entity plane = createGridedPlaneEntity(10);

//    Entity unitDirs = createUnitDirs();
//    Entity cube = createBoundedCubeEntity();
//    Entity cone = createBoundedConeEntity();
//    Entity sphere = createBoundedSphereEntity();
//    Entity cylinder = createBoundedCylinderEntity();
//    Entity capsule = createBoundedCapsuleEntity();
//    cube.setPosition(glm::vec3(3,0,0));
//    capsule.setPosition(glm::vec3(-3, 0, 0));
//    sphere.setPosition(glm::vec3(0,0,3));
//    cylinder.setPosition(glm::vec3(0,0,-3));
//    unitDirs.addChild(cube);
//    unitDirs.addChild(capsule);
//    unitDirs.addChild(sphere);
//    unitDirs.addChild(cylinder);

//    unitDirs.setPosition(glm::vec3(0,3, 0));

//    //Entity sphere = createBoundedSphereEntity();
//    float mass = 1.0f;
//    float radius = 1.0f;
//    float inertia = (2.0f/5.0f)*mass*radius*radius;
//    UniformRigidBody rb(mass, inertia);
//    // SphereBody otherRb(mass, 0.5f);
//    SphereCollider collider(0.5f);
//    collider.rb = &rb;
//    std::vector<SphereCollider> scs;
//    std::vector<UniformRigidBody> rbs;
//    std::vector<Collider*> colliders;
//    scs.reserve(20);
//    rbs.reserve(20);
//    colliders.reserve(20);
//    colliders.push_back(&collider);
//    for(int i=0; i<15; i++)
//    {
//        rbs.push_back(UniformRigidBody(mass, inertia));
//        rbs[rbs.size()-1].position = glm::vec3(7-i, 5, -3);
//        rbs[rbs.size()-1].linearMomentum  = glm::vec3(0, 0.0f, 0);

//        scs.push_back(SphereCollider(0.5f));
//        colliders.push_back(&scs[scs.size()-1]);
//        colliders[colliders.size()-1]->rb = &rbs[rbs.size()-1];

//    }
////    for(auto it = rbs.begin();it!=rbs.end();it++)
////    {
////        SphereCollider sc(0.5f);
////        sc.rb = &(*it);
////        scs.push_back(sc);
////    }
////    for(auto it = scs.begin();it!=scs.end();it++)
////    {
////        colliders.push_back(&(*it));
////    }
//    PhysicsWorld world(&colliders, glm::vec3(0, -1.5f, 0));

//    PlaneCollider p1(glm::vec3(-10, 0, -10), glm::vec3(-10, 0, 10), glm::vec3(10, 0, 10));
//    PlaneCollider p2(glm::vec3(-10, 0, -10), glm::vec3(10, 0, 10), glm::vec3(10, 0, -10));
//    world.colliders.push_back(&p1);
//    world.colliders.push_back(&p2);


//    rb.position = glm::vec3(0, 5, 0);

//    QElapsedTimer elapsedTimer;
//    elapsedTimer.start();
//    long time = elapsedTimer.nsecsElapsed();
//    //app.
//    while(window.shouldRun())
//    {

//        long timeNow = elapsedTimer.nsecsElapsed();
//        dt = (timeNow-time)/1000000000.0f;
//        time = timeNow;

//        openglFunctions->glEnable(GL_DEPTH_TEST);
//        openglFunctions->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

//        if(window.windowResized())
//        {
//            paintDevice->setSize(window.size() * window.devicePixelRatio());
//            paintDevice->setDevicePixelRatio(window.devicePixelRatio());
//            projection = glm::perspective((float)PI*0.33f, (float)window.width()/window.height(), 0.1f, 100.0f);
//            modelShader->setMat4("projection", projection);
//            gridShader->setMat4("projection", projection);
//        }

//        if(window.getKey(Qt::Key_A))
//        {
//            cam.translateRight(-dt);
//        }
//        if(window.getKey(Qt::Key_D))
//        {
//            cam.translateRight(dt);
//        }
//        if(window.getKey(Qt::Key_W))
//        {
//            cam.translateFwd(-dt);
//        }
//        if(window.getKey(Qt::Key_S))
//        {
//            cam.translateFwd(dt);
//        }
//        if(window.getMouse(Qt::MouseButton::LeftButton))
//        {
//            QPointF deltaPos = QCursor::pos()-window.mousePos;
//            window.mousePos = QCursor::pos();
//            cam.smoothRotateYaw(-dt*deltaPos.x());
//            cam.smoothRotatePitch(-dt*deltaPos.y());
//            //            cam.rotateYaw(-(float)dt*deltaPos.x());
//            //            cam.rotatePitch(-(float)dt*deltaPos.y());
//        }
//        if(window.getKey(Qt::Key_Right))
//        {
//            rb.addForce(5.0f*cam.getRight());
//        }
//        if(window.getKey(Qt::Key_Left))
//        {
//            rb.addForce(-5.0f*cam.getRight());
//        }
//        if(window.getKey(Qt::Key_Up))
//        {
//            rb.addForce(glm::cross(glm::vec3(0,5,0), cam.getRight()));
//        }
//        if(window.getKey(Qt::Key_Down))
//        {
//            rb.addForce(glm::cross(glm::vec3(0,-5,0), cam.getRight()));
//        }
//        if(window.getGetDown(Qt::Key_Space))
//        {
//            rb.addForce(glm::vec3(0,800,0));
//        }
//        if(window.getGetDown(Qt::Key_R))
//        {
//            rb.setVelocity(glm::vec3(0,0,0));
//            rb.position = glm::vec3(0,5,0);
//        }
//        cam.smoothUpdateView();
//        //cam.updateView();
//        modelShader->setMat4("view", cam.view);
//        gridShader->setMat4("view", cam.view);

//        //unitDirs.rotate(glm::quat(glm::vec3(0,dt,0)));
//        //unitDirs.draw();

//        world.stepWorld(dt);

//        sphere.meshes[1].setColor(glm::vec3(1,0,0));
//        sphere.setPosition(rb.position);
//        sphere.setRotation(rb.rotation);
//        sphere.draw();


//        sphere.meshes[1].setColor(glm::vec3(0,1,0));
//        for(auto& rb : rbs)
//        {
//            sphere.setPosition(rb.position);
//            sphere.setRotation(rb.rotation);
//            sphere.draw();
//        }


//        plane.draw();
//        RayCastData rcd;
//        if(world.Raycast(rb.position, glm::vec3(0,-1,0), rcd))
//            drawLine(rb.position, rcd.point);
//        //drawLine(glm::vec3(0, 5.0, 0), glm::vec3(5, -5.0, -5));
//        //glm::mat4 ok = glm::translate(trans, glm::vec3(0, 2, 0));




//        openglFunctions->glDisable(GL_DEPTH_TEST);
//        QPainter painter(paintDevice);
//        painter.setWorldMatrixEnabled(false);
//        painter.setPen(Qt::white);
//        painter.setFont(QFont("Arial", 12));
//        QRectF rect(0.0f,0.0f,paintDevice->size().width(), paintDevice->size().height());
//        painter.beginNativePainting();
//        painter.drawText(rect, std::to_string(1.0/dt).c_str());
//        painter.endNativePainting();

//        window.resetInputs();
//        app.processEvents();
//        openglFunctions->glFinish();
//        context->makeCurrent(&window);
//        context->swapBuffers(&window);
//    }

//    app.quit();
//    return 0;
//}
