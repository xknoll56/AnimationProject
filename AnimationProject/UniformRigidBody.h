#ifndef UNIFORMRIGIDBODY_H
#define UNIFORMRIGIDBODY_H

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <vector>

struct UniformRigidBody
{

    //constants
    const float mass;
    const float inertia;
    float massInv;
    float inertiaInv;
    float elasticity = 0.25f;

    //state variables
    glm::vec3 position;
    glm::quat rotation;
    glm::mat3 rotationMatrix;
    glm::vec3 linearMomentum;
    glm::vec3 angularMomentum;

    //derived quantities
    glm::vec3 velocity;
    glm::vec3 angularVelocity;

    //known quantities
    glm::vec3 force;
    glm::vec3 torque;

    //applied force/torque will be applie for a single step
    std::vector<glm::vec3> appliedForces;
    std::vector<glm::vec3> appliedTorques;
    bool applyForce = false;
    bool applyTorque = false;
    bool dynamic = true;
    bool applyGravity = true;


    UniformRigidBody(float _mass, float _inertia);
    UniformRigidBody();
    UniformRigidBody(const UniformRigidBody& other);
    UniformRigidBody& operator= (const UniformRigidBody& other);
    virtual ~UniformRigidBody();
    void addForce(const glm::vec3& force);
    void addTorque(const glm::vec3& torque);
    void setVelocity(const glm::vec3& velocity);
    void setAngularVelocity(const glm::vec3& angularVelocity);
    glm::vec3 getLocalXAxis();
    glm::vec3 getLocalYAxis();
    glm::vec3 getLocalZAxis();
    glm::vec3 peekNextPosition(float dt);
    void stepQuantities(float dt);
};
#endif // UNIFORMRIGIDBODY_H
