#pragma once
#include <glm/glm.hpp>
#include "SphereBV.h"
#include "SphereMesh.h"
#include <vector>

class SimulatorWorld
{
public:
    // Initializes the simulator world with specified parameters
    SimulatorWorld(
        const int numSpheres,
        const float minRadius,
        const float maxRadius,
        const float minVelocity,
        const float maxVelocity,
        const float minMass,
        const float maxMass,
        const float worldSize
    );

    ~SimulatorWorld();

    void initializeWorld(); // Initialize the simulation world with spheres and their properties
    void stepSimulation(float deltaTime); // Update the simulation state
    void stopSimulation(); // Stop the simulation and clean up resources
    void resetSimulation(); // Reset the simulation to its initial state
    void render(); // Render the simulation world

private:
    SphereBV* spheres; // Array of spheres in the simulation
    int numSpheres; // Number of spheres
    float worldSize; // Size of the simulation world

    // Utility functions
    void applyImpulse(int sphereIndex, const glm::vec3& impulse); // Apply an impulse to a specific sphere
    void initializeWorldBoundary(); // Initialize the world boundary
};