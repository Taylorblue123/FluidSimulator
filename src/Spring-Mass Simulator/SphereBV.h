#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp> // Required for glm::translate and glm::scale
#include "SphereMesh.h"
#include <cmath>
#include <vector>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/matrix_cross_product.hpp> // Include for matrixCross3
#include <glm/gtx/orthonormalize.hpp>      // Include for orthonormalize

//Implement SPM fluid simulation

class SphereBV
{
public:
    //Constant quantities
    float mass;       // Mass of the sphere

    //State Variables
    alignas(64) glm::vec3 center; // X(t), center of the body
    char padding1[64 - sizeof(glm::vec3)]; // Padding to ensure 64-byte alignment for SIMD operations

    alignas(64) glm::vec3 predictedPosition; 
    char padding2[64 - sizeof(glm::vec3)]; // Padding to ensure 64-byte alignment for SIMD operations

    //Derived quantities
    alignas(64) glm::vec3 velocity; // linear velocity in world coordinates
    char padding3[64 - sizeof(glm::vec3)]; // Padding to ensure 64-byte alignment for SIMD operations

    alignas(64) float density;
    char padding_density[64 - sizeof(float)];

    //Computed quantities
    glm::vec3 P; // Linear momentum
    
    glm::vec3 viscosity; // Viscosity in the fluid, computed from density
    glm::vec3 force;

    //Other related variables 
    float radius;     // Radius of the sphere
    SphereMesh* mesh; // Mesh representation of the sphere
    glm::mat4 transform; // Transformation matrix for the sphere, decide where to place the sphere in the cubic world
    
    
    int complexityLevel; // Complexity level of the sphere, use to generate the sphere mesh by lathing and longhitude

    glm::vec3 color;  // Color of the sphere

    int id;

    // Default constructor
    SphereBV() 
        : center(0.0f), 
          predictedPosition(0.0f),
          P(0.0f), // initialize momentum to zero 
          velocity(0.0f), 
          mass(0.0f), 
          complexityLevel(0), 
          radius(0.0f), 
          color(0.0f), 
          id(-1), 
          mesh(nullptr) {}

    // Constructor
    SphereBV(const glm::vec3& center, float radius, const glm::vec3& initVel, float mass, int complexityLevel, const glm::vec3& color, int id)
        : center(center),
            predictedPosition(center), 
          mass(mass), 
          radius(radius), 
          color(color), 
          id(id), 
          mesh(nullptr) {

        // compute momentum and derived velocity
        this->P = initVel * mass;
        this->velocity = initVel;

        this->velocity = P / mass; // Linear velocity in world coordinates

        //Set force to zero at now
        force = glm::vec3(0.0f); // No external force

        // Ensure minimum complexity of 1 to avoid empty mesh data.
        int effectiveComplexity = (complexityLevel < 6 ? 6 : complexityLevel);
        this->complexityLevel = effectiveComplexity;
        mesh = new SphereMesh(effectiveComplexity, color); // Create a new sphere mesh with the given complexity level
        transform = glm::mat4(1.0f); // Initialize the transformation matrix to identity
        transform = glm::translate(transform, center); // Translate the sphere to its center position
        transform = glm::scale(transform, glm::vec3(radius)); // Scale the sphere to its radius
    }

    
    // Destructor
    ~SphereBV() {
        // We don't delete mesh here as ownership is managed elsewhere
    }

    // Check if a point is inside the sphere
    bool contains(const glm::vec3& point) const {
        return glm::length(point - center) <= radius;
    }

    // Check if another sphere intersects with this one
    bool intersects(const SphereBV& other) const {
        float distance = glm::length(other.center - center);
        return distance <= (radius + other.radius);
    }

    // Function to reset the force acting on the sphere
    void resetForce() {
        this->force = glm::vec3(0.0f);
    }

    // Function to apply gravity acting on the sphere
    void applyGravity(float gravityCoeff) {
        glm::vec3 gravityForce = glm::vec3(0.0f, -gravityCoeff * mass, 0.0f); // Gravity force
        force += gravityForce; // Add gravity force to the sphere
    }
    void applyDamping(float linearDampingCoeff) {
        // Apply linear damping force: F_damp = -coeff * velocity
        if (linearDampingCoeff > 0.0f) {
            force -= linearDampingCoeff * velocity;
        }
    }

    //Derivative function to calculate the derivative variable of all state variables, return a new SphereBV that just represents the derivative variables.
    SphereBV derivative() const { // Make const
        // Calculate the derivative of the state variables
        glm::vec3 derivative_center = velocity; // dX/dt = V
        glm::vec3 derivative_P = force; // dP/dt = F

        // Construct a new SphereBV object to hold these derivatives in the state variable slots
        SphereBV derivState;
        derivState.center = derivative_center;
        derivState.P = derivative_P;

        // Copy constant properties needed for potential intermediate calculations if any
        // (though derivative() itself doesn't use them directly for the return value)
        derivState.mass = mass;
        derivState.radius = radius;
        derivState.id = id; // Keep track of which sphere this derivative belongs to if needed

        return derivState;
    }

    // Friend declarations for operator overloads defined in Integrators.h
    // These allow the operators to access private members if needed (though currently not necessary)
    friend SphereBV operator+(const SphereBV& lhs, const SphereBV& rhs);
    friend SphereBV operator*(const SphereBV& s, float scalar);
    friend SphereBV operator*(float scalar, const SphereBV& s);

};




inline bool operator==(const SphereBV &lhs, const SphereBV &rhs) {
    return lhs.id == rhs.id;
}