#include <MainApplication.h>



int main(int argc, char *argv[])
{
    MainApplication app(argc, argv);
    app.setup(1280, 720);
    app.execute();
}
