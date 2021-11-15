#include <MainApplication.h>
#include <Scenes/StackScene.h>
#include <Scenes/CollisionTestScene.h>
#include <Scenes/CubeDropScene.h>
#include <Scenes/DemoScene.h>
#include <Scenes/VaccumeScene.h>


int main(int argc, char *argv[])
{
    MainApplication app(argc, argv);
    app.setup(1280, 720);
    //CollisionTestScene scene;
   //ubeDropScene scene;
    //DemoScene scene;
    StackScene scene;
    //VaccumeScene scene;
    app.scene = &scene;
    app.execute();
}
