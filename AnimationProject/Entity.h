#ifndef ENTITY_H
#define ENTITY_H

#include <glm/glm.hpp>
#include <QOpenGLFunctions_4_5_Core>
#include <Mesh.h>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <Common.h>

struct Transform
{
    glm::vec3 position;
    glm::vec3 scale;
    glm::quat rotation;
    glm::vec3 euler;

    Transform();
    Transform(const glm::vec3& position, const glm::vec3& scale, const glm::vec3& euler);
    Transform(const glm::vec3& position, const glm::vec3& scale, const glm::quat& rotation);
};




struct Renderable
{
    std::map<Mesh*, glm::vec3> meshes;
};

class Entity
{
private:
    Transform transform;
    glm::mat4 positionMat;
    glm::mat4 scaleMat;
    glm::mat4 rotationMat;
    glm::mat4 model;
    std::vector<Entity> children;

    void setModel();
    void drawEntitiesRecursively(const glm::mat4& parentModel);

public:

    std::vector<Mesh> meshes;
    Entity(Mesh& mesh);
    Entity(Mesh& mesh, const Transform& transform);
    Entity(const Transform& transform);
    Entity();
    Entity(const std::vector<Mesh>& meshes);
    const Transform& getTransform();
    void setTransform(const Transform& transform);
    void translate(const glm::vec3& trans);
    void rotate(const glm::quat& rot);
    void scale(const glm::vec3 scale);
    void setPosition(const glm::vec3& position);
    void setScale(const glm::vec3& scale);
    void setRotation(const glm::quat& rotation);
    void addMeshWithColor(const Mesh& mesh, glm::vec3 color);
    void addChild(Entity& entity);
    void draw();
};


Entity createBoundedCubeEntity();
Entity createBoundedCapsuleEntity();
Entity createBoundedCylinderEntity();
Entity createBoundedSphereEntity();
Entity createBoundedConeEntity();
Entity createGridedPlaneEntity(int size);
Entity createBoundedPlaneEntity();
Entity createUnitDirs();

#endif // ENTITY_H
