// filepath: d:\2025Spring\CMSC838M\Homework1\ProjectSourceCode\CollisionDetection\src\Spring-Mass Simulator\PerformanceAnalysis.cpp
#include <iostream>
#include <fstream>
#include <chrono>
#include <vector>
#include <string>
#include <iomanip>
#include "SphereBV.h"
#include "CollisionDetection.h"
#include "Utils.h"

// Function to create spheres with specified parameters
void createSpheres(SphereBV* spheres, int numSpheres, int complexity, float radius, float velocity, float mass, float worldSize) {
    Utils utils;
    for (int i = 0; i < numSpheres; i++) {
        glm::vec3 position = glm::vec3(utils.randomFloat(-worldSize, worldSize), utils.randomFloat(-worldSize, worldSize), 0.0f);
        glm::vec3 initVel = glm::vec3(utils.randomFloat(minVelocity, maxVelocity), utils.randomFloat(minVelocity, maxVelocity), 0.0f);
        spheres[i] = SphereBV(position, radius, initVel, glm::vec3(0.0f), mass, complexity, glm::vec3(1.0f, 0.0f, 0.0f), i);
    }
}

// Function to measure collision detection performance
void measurePerformance(int numSpheres, int complexity, float radius, float velocity, 
                        float mass, float worldSize, int method, std::ofstream& outputFile) {
    
    // Create spheres with specified parameters
    SphereBV* spheres = new SphereBV[numSpheres];
    createSpheres(spheres, numSpheres, complexity, radius, velocity, mass, worldSize);

    // Vector to store collision pairs
    std::vector<std::pair<SphereBV*, SphereBV*>> collisionPairs;
    
    // Create collision detection object
    CollisionDetection collisionDetection(spheres, numSpheres, worldSize, &collisionPairs, method);
    
    // Timing variables
    auto startBroad = std::chrono::high_resolution_clock::now();
    auto endBroad = std::chrono::high_resolution_clock::now();
    auto startNarrow = std::chrono::high_resolution_clock::now();
    auto endNarrow = std::chrono::high_resolution_clock::now();
    auto startHandle = std::chrono::high_resolution_clock::now();
    auto endHandle = std::chrono::high_resolution_clock::now();

    // Timing for broad phase
    startBroad = std::chrono::high_resolution_clock::now();
    collisionDetection.broadCollisionDetection();
    endBroad = std::chrono::high_resolution_clock::now();

    // Count potential collisions after broad phase
    int potentialCollisions = collisionPairs.size();

    // Timing for narrow phase
    startNarrow = std::chrono::high_resolution_clock::now();
    collisionDetection.narrowCollisionDetection();
    endNarrow = std::chrono::high_resolution_clock::now();

    // Count actual collisions after narrow phase
    int actualCollisions = collisionPairs.size();

    // Timing for collision handling
    startHandle = std::chrono::high_resolution_clock::now();
    collisionDetection.handleCollision();    
    endHandle = std::chrono::high_resolution_clock::now();

    // Calculate durations in microseconds
    auto broadDuration = std::chrono::duration_cast<std::chrono::microseconds>(endBroad - startBroad).count();
    auto narrowDuration = std::chrono::duration_cast<std::chrono::microseconds>(endNarrow - startNarrow).count();
    auto handleDuration = std::chrono::duration_cast<std::chrono::microseconds>(endHandle - startHandle).count();
    auto totalDuration = broadDuration + narrowDuration + handleDuration;

    // Convert microseconds to milliseconds for output
    double broadMs = broadDuration / 1000.0;
    double narrowMs = narrowDuration / 1000.0;
    double handleMs = handleDuration / 1000.0;
    double totalMs = totalDuration / 1000.0;

    // Get method name
    std::string methodName = (method == 0) ? "Sweep and Prune" : "Other Method";

    // Output to file: numSpheres,complexity,radius,velocity,mass,worldSize,method,
    // broadTime(ms),narrowTime(ms),handleTime(ms),totalTime(ms),potentialCollisions,actualCollisions
    outputFile << numSpheres << ","
               << complexity << ","
               << radius << ","
               << velocity << ","
               << mass << ","
               << worldSize << ","
               << methodName << ","
               << std::fixed << std::setprecision(3) << broadMs << ","
               << std::fixed << std::setprecision(3) << narrowMs << ","
               << std::fixed << std::setprecision(3) << handleMs << ","
               << std::fixed << std::setprecision(3) << totalMs << ","
               << potentialCollisions << ","
               << actualCollisions << std::endl;

    // Clean up
    delete[] spheres;

    std::cout << "Completed test: " << numSpheres << " spheres, complexity " << complexity 
              << ", radius " << radius << ", method " << methodName 
              << ", velocity " << velocity << ", mass " << mass
              << ", broad Collisions Detection Time " << broadMs << " ms"
              << ", narrow Collisions Detection Time " << narrowMs << " ms"
              << ", handle Collisions Time " << handleMs << " ms"
              << ", time " << totalMs << " ms" 
              << ", potential collisions " << potentialCollisions 
              << ", actual collisions " << actualCollisions << std::endl;
}