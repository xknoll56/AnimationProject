#include "Entity.h"
glm::mat4 IDENTITY(1.0f);
extern Shader* modelShader;
extern Shader* gridShader;

Transform::Transform()
{
    position = glm::vec3();
    scale = glm::vec3(1,1,1);
    euler = glm::vec3();
    rotation = glm::quat(euler);
}

Transform::Transform(const glm::vec3& position, const glm::vec3& scale, const glm::vec3& euler)
{
    this->position = position;
    this->scale = scale;
    this->euler = euler;
    this->rotation = glm::quat(euler);
}

Transform::Transform(const glm::vec3& position, const glm::vec3& scale,const glm::quat& rotation)
{
    this->position = position;
    this->scale = scale;
    this->rotation = rotation;
    this->euler = glm::eulerAngles(rotation);
}


void Entity::setModel()
{
    model = positionMat*rotationMat*scaleMat;
}

void Entity::drawEntitiesRecursively(const glm::mat4& parentModel)
{
    glm::mat4 mat =  parentModel*model;
    for(auto& mesh: meshes)
    {
        switch(mesh.getType())
        {
        case GL_TRIANGLES:
            modelShader->setMat4("model", mat);
            mesh.draw(*modelShader);
            break;
        case GL_LINES:
            gridShader->setMat4("model", mat);
            mesh.draw(*gridShader);
            break;
        }
    }

    for(auto& child: children)
    {
        child.drawEntitiesRecursively(mat);
    }
}


Entity::Entity(Mesh& mesh)
{
    positionMat = glm::translate(IDENTITY, transform.position);
    scaleMat = glm::scale(IDENTITY, transform.scale);
    rotationMat = glm::toMat4(transform.rotation);
    setModel();
    meshes.push_back(mesh);
}

Entity::Entity(Mesh& mesh, const Transform& transform)
{
    setTransform(transform);
    meshes.push_back(mesh);
}

Entity::Entity(const Transform& transform)
{
    setTransform(transform);
}

Entity::Entity()
{
    positionMat = glm::translate(IDENTITY, transform.position);
    scaleMat = glm::scale(IDENTITY, transform.scale);
    rotationMat = glm::toMat4(transform.rotation);
    setModel();
}

Entity::Entity(const std::vector<Mesh>& meshes)
{
    positionMat = glm::translate(IDENTITY, transform.position);
    scaleMat = glm::scale(IDENTITY, transform.scale);
    rotationMat = glm::toMat4(transform.rotation);
    setModel();
    this->meshes = meshes;
}

const Transform& Entity::getTransform()
{
    return transform;
}

void Entity::setTransform(const Transform& transform)
{
    this->transform = transform;
    positionMat = glm::translate(IDENTITY, transform.position);
    scaleMat = glm::scale(IDENTITY, transform.scale);
    rotationMat = glm::toMat4(transform.rotation);
    setModel();
}

void Entity::translate(const glm::vec3& trans)
{
    transform.position += trans;
    positionMat = glm::translate(positionMat, trans);
    setModel();
}

void Entity::rotate(const glm::quat& rot)
{
    transform.rotation *= rot;
    rotationMat = glm::toMat4(transform.rotation);
    setModel();
}

void Entity::scale(const glm::vec3 scale)
{
    transform.scale += scale;
    scaleMat = glm::scale(IDENTITY, transform.scale);
    setModel();
}

void Entity::setPosition(const glm::vec3& position)
{
    transform.position = position;
    positionMat = glm::translate(IDENTITY, transform.position);
    setModel();
}

void Entity::setScale(const glm::vec3& scale)
{
    transform.scale = scale;
    scaleMat = glm::scale(IDENTITY, transform.scale);
    setModel();
}

void Entity::setRotation(const glm::quat& rotation)
{
    transform.rotation = rotation;
    transform.euler = glm::eulerAngles(rotation);
    rotationMat = glm::toMat4(rotation);
    setModel();
}

void Entity::addMeshWithColor(const Mesh& mesh, glm::vec3 color)
{
    Mesh toAdd = Mesh(mesh);
    toAdd.setColor(color);
    meshes.push_back(toAdd);
}
void Entity::addChild(Entity& entity)
{
    children.push_back(entity);
}

void Entity::draw()
{
    drawEntitiesRecursively(IDENTITY);
}

Entity createBoundedCubeEntity()
{
    std::vector<Mesh> meshes = {Mesh::createCube(), Mesh::createBoundingBox()};
    return Entity(meshes);
}

Entity createBoundedCapsuleEntity()
{
    std::vector<Mesh> capsuleMeshes = {Mesh::createCapsule(), Mesh::createBoundingCapsule()};
    return Entity(capsuleMeshes);
}

Entity createBoundedCylinderEntity()
{
    std::vector<Mesh> meshes = {Mesh::createCylinder(), Mesh::createBoundingCylinder()};
    return Entity(meshes);
}

Entity createBoundedSphereEntity()
{
    std::vector<Mesh> meshes = {Mesh::createSphere(), Mesh::createBoundingSphere()};
    return Entity(meshes);
}

Entity createBoundedConeEntity()
{
    std::vector<Mesh> meshes = {Mesh::createCone(), Mesh::createBoundingCone()};
    return Entity(meshes);
}

Entity createGridedPlaneEntity(int size)
{
    Mesh grid = Mesh::createGrid(size);
    grid.setColor(glm::vec3(0.9f,0.1f,0.3f));
    Entity parent(grid);
    std::vector<Mesh> meshes = {Mesh::createPlane(), Mesh::createBoundingPlane()};
    Entity child(meshes);
    child.setScale(glm::vec3(2*size, 1, 2*size));
    parent.addChild(child);
    return parent;
}

Entity createBoundedPlaneEntity()
{
    std::vector<Mesh> meshes = {Mesh::createPlane(), Mesh::createBoundingPlane()};
    return Entity(meshes);
}

Entity createArrow()
{
    Entity arrow;
    Entity cyl = createBoundedCylinderEntity();
    Entity cone = createBoundedConeEntity();
    cone.setPosition(glm::vec3(0,1.0f,0.0f));
    //cyl.setScale()
    arrow.addChild(cyl);
    arrow.addChild(cone);
    return arrow;
}

Entity createUnitDirs()
{
    Entity unitDirs;
    Mesh cone = Mesh::createCone();
    Mesh cyl = Mesh::createCylinder();

    Transform tCylX(glm::vec3(0.5f, 0, 0), glm::vec3(0.1f, 0.5f, 0.1f), glm::vec3(0, 0, -(float)PI*0.5f));
    Transform tConeX(glm::vec3(1.0f, 0, 0), glm::vec3(0.25f, 0.25f, 0.25f), glm::vec3(0, 0, -(float)PI*0.5f));
    Transform tCylY(glm::vec3(0, 0.5f, 0), glm::vec3(0.1f, 0.5f, 0.1f), glm::vec3(0, 0, 0));
    Transform tConeY(glm::vec3(0, 1.0f, 0), glm::vec3(0.25f, 0.25f, 0.25f), glm::vec3(0, 0, 0));
    Transform tCylZ(glm::vec3(0, 0, 0.5f), glm::vec3(0.1f, 0.5f, 0.1f), glm::vec3((float)PI*0.5f, 0, 0));
    Transform tConeZ(glm::vec3(0, 0, 1.0f), glm::vec3(0.25f, 0.25f, 0.25f), glm::vec3((float)PI*0.5f, 0, 0));

    Entity cylX(tCylX);
    Entity coneX(tConeX);
    Entity cylY(tCylY);
    Entity coneY(tConeY);
    Entity cylZ(tCylZ);
    Entity coneZ(tConeZ);

    cylX.addMeshWithColor(cyl, glm::vec3(1,0,0));
    coneX.addMeshWithColor(cone, glm::vec3(1,0,0));
    cylY.addMeshWithColor(cyl, glm::vec3(0,1,0));
    coneY.addMeshWithColor(cone, glm::vec3(0,1,0));
    cylZ.addMeshWithColor(cyl, glm::vec3(0,0,1));
    coneZ.addMeshWithColor(cone, glm::vec3(0,0,1));

    unitDirs.addChild(cylX);
    unitDirs.addChild(coneX);
    unitDirs.addChild(cylY);
    unitDirs.addChild(coneY);
    unitDirs.addChild(cylZ);
    unitDirs.addChild(coneZ);

    return unitDirs;
}
