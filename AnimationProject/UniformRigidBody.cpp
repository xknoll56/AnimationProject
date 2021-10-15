#include "UniformRigidBody.h"
#include <algorithm>


UniformRigidBody::UniformRigidBody(float _mass, float _inertia): mass(_mass), inertia(_inertia)
{
    massInv = 1.0f/mass;
    inertiaInv = 1.0f/inertia;
    position = glm::vec3();
    rotation = glm::quat(glm::vec3(0,0,0));
    linearMomentum = glm::vec3();
    angularMomentum = glm::vec3();
    velocity = glm::vec3();
    angularVelocity = glm::vec3();
    gravitionalForce = glm::vec3();
    torque = glm::vec3();
    rotation = glm::normalize(rotation);
    rotationMatrix = glm::toMat3(rotation);
}

UniformRigidBody::UniformRigidBody(const UniformRigidBody& other): mass(other.mass), inertia(other.inertia)
{
    massInv = 1.0f/other.mass;
    inertiaInv = 1.0f/other.inertia;
    position = other.position;
    rotation = other.rotation;
    linearMomentum = other.linearMomentum;
    angularMomentum = other.angularMomentum;
    velocity = other.velocity;
    angularVelocity = other.angularVelocity;
    gravitionalForce = other.gravitionalForce;
    torque = other.torque;
    rotation = glm::normalize(rotation);
    rotationMatrix = glm::toMat3(rotation);
}

UniformRigidBody& UniformRigidBody::operator= (const UniformRigidBody& other)
{
    massInv = 1.0f/other.mass;
    inertiaInv = 1.0f/other.inertia;
    position = other.position;
    rotation = other.rotation;
    linearMomentum = other.linearMomentum;
    angularMomentum = other.angularMomentum;
    velocity = other.velocity;
    angularVelocity = other.angularVelocity;
    gravitionalForce = other.gravitionalForce;
    torque = other.torque;
    rotation = glm::normalize(rotation);
    rotationMatrix = glm::toMat3(rotation);
    return *this;
}

UniformRigidBody::UniformRigidBody(): mass(1.0f), inertia(1.0f)
{
    massInv = 1.0f/mass;
    inertiaInv = 1.0f/inertia;
    position = glm::vec3();
    rotation = glm::quat(glm::vec3(0,0,0));
    linearMomentum = glm::vec3();
    angularMomentum = glm::vec3();
    velocity = glm::vec3();
    angularVelocity = glm::vec3();
    gravitionalForce = glm::vec3();
    torque = glm::vec3();
    rotation = glm::normalize(rotation);
    rotationMatrix = glm::toMat3(rotation);
}


UniformRigidBody::~UniformRigidBody()
{

}

void UniformRigidBody::addForce(const glm::vec3& force)
{
    appliedForces.push_back(force);
    applyForce = true;
}

void UniformRigidBody::addTorque(const glm::vec3& torque)
{
    appliedTorques.push_back(torque);
    applyTorque = true;
}

void UniformRigidBody::addForce(const glm::vec3& force, UniformRigidBody& other)
{
    appliedForces.push_back(force);
    applyForce = true;
    if(atRest)
    {
    if(!(std::find(appliedBodies.begin(), appliedBodies.end(), &other)!=appliedBodies.end()))
    {
        appliedBodies.push_back(&other);
        atRest = false;
    }
    }
}
void UniformRigidBody::addTorque(const glm::vec3& torque,  UniformRigidBody& other)
{
    appliedTorques.push_back(torque);
    applyTorque = true;
    if(!(std::find(appliedBodies.begin(), appliedBodies.end(), &other)!=appliedBodies.end()))
    {
        appliedBodies.push_back(&other);
    }
}

void UniformRigidBody::setVelocity(const glm::vec3& velocity)
{
    linearMomentum = velocity*mass;
}

void UniformRigidBody::setAngularVelocity(const glm::vec3& angularVelocity)
{
    angularMomentum = inertia*angularVelocity;
}

glm::vec3 UniformRigidBody::getLocalXAxis()
{
    return glm::vec3(rotationMatrix[0]);
}

glm::vec3 UniformRigidBody::getLocalYAxis()
{
    return glm::vec3(rotationMatrix[1]);
}

glm::vec3 UniformRigidBody::getLocalZAxis()
{
    return glm::vec3(rotationMatrix[2]);
}

glm::vec3 UniformRigidBody::peekNextPosition(float dt)
{
    glm::vec3 tempMomentum = linearMomentum+gravitionalForce*dt;
    return position + massInv*tempMomentum*dt;
}
void UniformRigidBody::stepQuantities(float dt)
{

    if(dynamic && !atRest)
    {
        if(applyForce)
        {
            for(glm::vec3 force: appliedForces)
                linearMomentum+=dt*force;
            applyForce = false;
            appliedForces.clear();
        }
        if(applyTorque)
        {
            for(glm::vec3 torque: appliedTorques)
                angularMomentum+=dt*torque;
            applyTorque = false;
            appliedTorques.clear();
        }
        angularMomentum += torque*dt;
        if(applyGravity)
            linearMomentum += gravitionalForce*dt;
        angularVelocity = inertiaInv*angularMomentum;
        velocity = massInv*linearMomentum;
        rotation+= dt*0.5f*glm::quat(0, angularVelocity.x, angularVelocity.y, angularVelocity.z)*rotation;
        rotation = glm::normalize(rotation);
        rotationMatrix = glm::toMat3(rotation);
        position+=dt*velocity;
    }
    else
    {
        angularMomentum = glm::vec3();
        linearMomentum = glm::vec3();
    }
}
