QT       += core gui
QT       += opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

INCLUDEPATH += \
    Dependencies

SOURCES += \
    Collider.cpp \
    Entity.cpp \
    MainApplication.cpp \
    MainDump.cpp \
    MainWindow.cpp \
    Camera.cpp \
    Mesh.cpp \
    PhysicsWorld.cpp \
    Scene.cpp \
    Shader.cpp \
    UniformRigidBody.cpp \
    main.cpp

HEADERS += \
    Collider.h \
    Common.h \
    Debug.h \
    Entity.h \
    MainApplication.h \
    MainWindow.h \
    Camera.h \
    Mesh.h \
    PhysicsWorld.h \
    Scene.h \
    Shader.h \
    UniformRigidBody.h

FORMS +=

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

DISTFILES += \
    Shaders/model.frag \
    Shaders/model.vert \
    Shaders/grid.vert \
    Shaders/grid.frag
