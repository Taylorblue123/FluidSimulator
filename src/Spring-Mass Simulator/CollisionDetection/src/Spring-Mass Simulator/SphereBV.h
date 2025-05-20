// filepath: d:\2025Spring\CMSC838M\Homework1\ProjectSourceCode\CollisionDetection\src\Spring-Mass Simulator\SphereBV.h
#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include "SphereMesh.h"
#include <cmath>
#include <vector>

class SphereBV
{
public:
    float mass;       // Mass of the sphere
    glm::vec2 center; // Center of the sphere in 2D
    glm::vec2 velocity; // Linear velocity in 2D
    float radius;     // Radius of the sphere
    SphereMesh* mesh; // Mesh representation of the sphere
    glm::mat4 transform; // Transformation matrix for the sphere

    // Default constructor
    SphereBV() 
        : center(0.0f), 
          velocity(0.0f), 
          mass(1.0f), 
          radius(1.0f), 
          mesh(nullptr) {}

    // Constructor
    SphereBV(const glm::vec2& center, float radius, const glm::vec2& initVel, float mass)
        : center(center), 
          radius(radius), 
          velocity(initVel), 
          mass(mass), 
          mesh(nullptr) {}

    // Destructor
    ~SphereBV() {
        delete mesh;
    }

    // Check if a point is inside the sphere
    bool contains(const glm::vec2& point) const {
        return glm::length(point - center) <= radius;
    }

    // Check if another sphere intersects with this one
    bool intersects(const SphereBV& other) const {
        float distance = glm::length(center - other.center);
        return distance <= (radius + other.radius);
    }

    // Update the position of the sphere based on its velocity
    void updatePosition(float deltaTime) {
        center += velocity * deltaTime;
    }

    // Update the transformation matrix for rendering
    void updateTransform() {
        transform = glm::translate(glm::mat4(1.0f), glm::vec3(center, 0.0f)) *
                     glm::scale(glm::mat4(1.0f), glm::vec3(radius));
    }
};