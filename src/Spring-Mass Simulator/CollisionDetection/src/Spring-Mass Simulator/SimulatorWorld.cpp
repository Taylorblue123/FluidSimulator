// filepath: d:\2025Spring\CMSC838M\Homework1\ProjectSourceCode\CollisionDetection\src\Spring-Mass Simulator\SimulatorWorld.cpp
#include "SimulatorWorld.h"
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/matrix_cross_product.hpp>
#include <glm/gtx/orthonormalize.hpp>
#include <vector>
#include <ctime>
#include <algorithm>

SimulatorWorld::SimulatorWorld(
    const int minComplexity,
    const int maxComplexity,
    const int numSpheres,
    const float minRadius,
    const float maxRadius,
    const float minVelocity,
    const float maxVelocity,
    const float minL,
    const float maxL,
    const float minMass,
    const float maxMass,
    const float worldSize
) : minComplexity(minComplexity),
    maxComplexity(maxComplexity),
    numSpheres(numSpheres),
    minRadius(minRadius),
    maxRadius(maxRadius),
    minVelocity(minVelocity),
    maxVelocity(maxVelocity),
    minL(minL),
    maxL(maxL),
    minMass(minMass),
    maxMass(maxMass),
    worldSize(worldSize) {

    srand(static_cast<unsigned int>(time(0)));
    spheres = new SphereBV[numSpheres];
    initializeWorld();
}

SimulatorWorld::~SimulatorWorld() {
    delete[] spheres;
}

void SimulatorWorld::initializeWorld() {
    for (int i = 0; i < numSpheres; i++) {
        float radius = Utils::randomFloat(minRadius, maxRadius);
        glm::vec3 position = glm::vec3(Utils::randomFloat(-worldSize, worldSize), 
                                        Utils::randomFloat(-worldSize, worldSize), 
                                        0.0f);
        glm::vec3 velocity = glm::vec3(Utils::randomFloat(minVelocity, maxVelocity), 
                                        Utils::randomFloat(minVelocity, maxVelocity), 
                                        0.0f);
        float mass = Utils::randomFloat(minMass, maxMass);
        spheres[i] = SphereBV(position, radius, velocity, glm::vec3(0.0f), mass, 8, glm::vec3(1.0f, 0.0f, 0.0f), i);
    }

    // Apply an impulse to the first sphere
    spheres[0].velocity += glm::vec3(5.0f, 0.0f, 0.0f); // Impulse to the right
}

void SimulatorWorld::stepSimulation(float deltaTime) {
    std::vector<std::pair<SphereBV*, SphereBV*>> collisionPairs;

    CollisionDetection collisionDetection(spheres, numSpheres, worldSize, &collisionPairs);
    collisionDetection.broadCollisionDetection();
    collisionDetection.narrowCollisionDetection();
    collisionDetection.handleCollision();

    for (int i = 0; i < numSpheres; i++) {
        spheres[i].center += spheres[i].velocity * deltaTime;

        glm::mat3 omegaSkew = glm::matrixCross3(spheres[i].omega);
        glm::mat3 deltaRotation = omegaSkew * deltaTime;
        spheres[i].rotation = (glm::mat3(1.0f) + deltaRotation) * spheres[i].rotation;
        spheres[i].rotation = glm::orthonormalize(spheres[i].rotation);

        glm::mat4 rotation4(spheres[i].rotation);
        spheres[i].transform = glm::translate(glm::mat4(1.0f), spheres[i].center) *
            rotation4 *
            glm::scale(glm::mat4(1.0f), glm::vec3(spheres[i].radius));
    }
}

void SimulatorWorld::stopSimulation() {
    delete[] spheres;
    spheres = nullptr;
}