#include "UniformRigidBody.h"



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
    force = glm::vec3();
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
    return glm::normalize(glm::vec3(rotationMatrix[0]));
}

glm::vec3 UniformRigidBody::getLocalYAxis()
{
    return glm::normalize(glm::vec3(rotationMatrix[1]));
}

glm::vec3 UniformRigidBody::getLocalZAxis()
{
    return glm::normalize(glm::vec3(rotationMatrix[2]));
}

glm::vec3 UniformRigidBody::peekNextPosition(float dt)
{
    glm::vec3 tempMomentum = linearMomentum+force*dt;
    return position + massInv*tempMomentum*dt;
}
void UniformRigidBody::stepQuantities(float dt)
{

    if(dynamic)
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
        linearMomentum += force*dt;
        angularVelocity = inertiaInv*angularMomentum;
        velocity = massInv*linearMomentum;
        rotation+= dt*0.5f*glm::quat(0, angularVelocity.x, angularVelocity.y, angularVelocity.z)*rotation;
        rotation = glm::normalize(rotation);
        rotationMatrix = glm::toMat3(rotation);
        position+=dt*velocity;
    }
}
