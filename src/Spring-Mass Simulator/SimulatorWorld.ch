#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp> // For glm::translate, glm::rotate, glm::scale
#include <glm/gtc/quaternion.hpp> // For quaternions if preferred for rotation
#include "SphereBV.h"
#include "SphereMesh.h"
#include <vector>
#include "CollisionDetection.h"
#include <omp.h>
#include <glm/gtc/constants.hpp> // For glm::pi
#include <algorithm> // For std::sort
#include <thrust/device_vector.h>
#include <thrust/sort.h>
#include <thrust/copy.h>

// Forward declare Entry if CompareEntriesFunctor needs it and Entry is complex,
// but for simple structs, defining Entry first is cleaner.
// struct Entry; 

// Define Entry struct before CompareEntriesFunctor
struct Entry {
    int particleIndex;
    int cellKey;

    // Add a constructor, make it __host__ __device__
    __host__ __device__ Entry(int index = -1, int key = -1)
        : particleIndex(index), cellKey(key) {}
};

// CompareEntriesFunctor
struct CompareEntriesFunctor {
    __host__ __device__ bool operator()(const Entry& a, const Entry& b) const {
        return a.cellKey < b.cellKey;
    }
};

class SimulatorWorld
{
public:
    // Member variables declared first
    SphereBV* spheres;
    glm::vec3* CubeWorldPosition; // These are local cube vertices
    std::vector<vertice> cubicWorldVertices; 
    std::vector<int> indices;
    int numSpheres;

    // Parameters that will be initialized by constructor
    int complexity; 
    float radius;
    float velocity; // Initial velocity magnitude
    float mass;     // Default mass for particles
    float worldSize; // This is the half-extent of the local bounding box
    float spheresSpacing;
    float influenceRadius;
    float targetDensity;
    float pressureCoefficient;
    float viscosityCoefficient; // Added missing member variable

    // World transformation
    glm::vec3 worldRotationEuler;   // Euler angles (degrees) for UI control
    glm::vec3 worldTranslationVec; // Translation vector for UI control
    glm::mat4 worldTransformMatrix;
    glm::mat4 inverseWorldTransformMatrix;

    std::vector<Entry> spatialLookup; // Changed from Entry* to Entry
    std::vector<int> startIndices;
    // int numGridCells; // If using a fixed grid size not tied to numSpheres


    SimulatorWorld(
        const int complexity,
        const int numSpheres,
        const float radius,
        const float velocity,
        const float mass,
        const float worldSize, 
        const float spheresSpacing,
        const float influenceRadius,
        const float targetDensity,
        const float pressureCoefficient,
        const float viscosityCoefficient
    );
    
    ~SimulatorWorld();
    
    void initializeWorld(float spheresSpacingParam); 
    void stepSimulation(float deltaTime, float damping, int integratorType, float gravityCoeff);
    void stopSimulation();
    void resetSimulation();
    void render(); 
    void initializeWorldBoundary();
    void shoot(glm::vec3 cameraFront);

    void updateWorldTransform(); // Updates matrices from Euler angles and translation

    // SPH related methods, defined inline (original versions, kept for reference or other uses)
    float calculateDensity(const glm::vec3& position) const {
        float currentDensity = 0.0f; // Renamed from density to avoid conflict
        for (int i = 0; i < this->numSpheres; ++i) {
            float distance = glm::length(this->spheres[i].center - position);
            // Use this->influenceRadius and this->mass for member access
            float influence = SmoothingKernel(this->influenceRadius, distance);
            currentDensity += this->mass * influence; // Assuming a uniform mass for this calculation context
                                                 // If spheres[i].mass is intended, use that.
        }
        return currentDensity;
    }

    float convertDensityToPressure(float currentDensity) const { // Renamed from density
        // Use this->targetDensity and this->pressureCoefficient
        float densityErr = currentDensity - this->targetDensity; 
        float pressure = densityErr * this->pressureCoefficient; 
        return pressure;
    } 

    float calculateProperty(const glm::vec3& position, const glm::vec3& property) const {
        float propertyValue = 0.0f;
        for (int i = 0; i < this->numSpheres; ++i) {
            float distance = glm::length(this->spheres[i].center - position);
            float influence = SmoothingKernel(this->influenceRadius, distance);
            float sphere_density = calculateDensity(this->spheres[i].center); // Recalculate or use stored if up-to-date
            if (std::abs(sphere_density) < 1e-6f) continue; // Avoid division by zero
            propertyValue += this->spheres[i].mass * influence * glm::length(property) / sphere_density;
        }
        return propertyValue;
    }

    glm::vec3 calculatePropertyGradient(const glm::vec3& position) const {
        glm::vec3 gradient(0.0f);
        for (int i = 0; i < this->numSpheres; ++i) {
            if (glm::length(this->spheres[i].center - position) < 1e-6f) continue; // Avoid issues at same point
            float distance = glm::length(this->spheres[i].center - position);
            glm::vec3 direction = glm::normalize(this->spheres[i].center - position);
            // Use this->influenceRadius
            float influence = SmoothingKernelDerivative(this->influenceRadius, distance);
            float density_val = this->spheres[i].density; 
            if (std::abs(density_val) < 1e-6f) continue; // Avoid division by zero
            gradient += -this->spheres[i].mass * influence * direction / density_val;
        }
        return gradient;
    } 

    glm::vec3 calculatePressureForce(const glm::vec3& position) const {
        glm::vec3 totalForce(0.0f); // Renamed from gradient
        for (int i = 0; i < this->numSpheres; ++i) {
            if(glm::length(position - this->spheres[i].center) < 1e-6f) { // Check distance for float comparison
                continue; 
            }
            float distance = glm::length(this->spheres[i].center - position);
            if (distance < 1e-6f) continue; // Avoid division by zero if direction becomes undefined
            glm::vec3 direction = glm::normalize(this->spheres[i].center - position);
            
            float influence = SmoothingKernelDerivative(this->influenceRadius, distance);
            float density_val = this->spheres[i].density; 
            if (std::abs(density_val) < 1e-6f) continue; // Avoid division by zero

            totalForce += -convertDensityToPressure(density_val) * this->spheres[i].mass * influence * direction / density_val;
        }
        return totalForce;
    } 

    // Spatial Grid and Optimized SPH
    void updateSpatialLookup(); // Populates and sorts spatialLookup, and populates startIndices

    // Hash position to an integer cell index (grid coordinates based)
    glm::ivec3 getCellCoords(const glm::vec3& position, float cellSizeparam) const {
        // Normalize position relative to world origin if necessary, or assume origin is 0,0,0
        // For simplicity, assuming world coordinates are directly mapped
        return glm::ivec3(
            static_cast<int>(std::floor(position.x / cellSizeparam)),
            static_cast<int>(std::floor(position.y / cellSizeparam)),
            static_cast<int>(std::floor(position.z / cellSizeparam))
        );
    }
    
    // Convert 3D cell coordinates to a single integer hash value
    // This needs to be consistent and ideally map to a manageable range for startIndices
    // The user's original getCellHash was: 1000*x + 1000*y + 1000000*z;
    // Let's use a more common prime number based hashing for 3D grid cell coordinates
    int getCellHashFromCoords(const glm::ivec3& cellCoords) const {
        // Large prime numbers for hashing
        const int p1 = 73856093;
        const int p2 = 19349663;
        const int p3 = 83492791;
        // Ensure positive hash value, then modulo
        int hash = (cellCoords.x * p1) ^ (cellCoords.y * p2) ^ (cellCoords.z * p3);
        return std::abs(hash); // Make it positive before modulo
    }

    // Convert a raw hash to a key that fits into startIndices array
    int getCellKeyFromHash(int hash) const {
        // Assuming startIndices size is numSpheres (as per previous logic)
        // or a fixed numGridCells if that's defined.
        // If startIndices.size() is 0, this will crash. Ensure it's sized.
        if (startIndices.empty()) return 0; // Should not happen if initialized
        return hash % startIndices.size(); 
    }

    void updateDensitiesAndPressureForces(); // New function for optimized SPH calculation

    void updateViscosity() {
        #pragma omp parallel for
        for (int i = 0; i < numSpheres; i++) {
            SphereBV& particle_i = spheres[i];
            glm::vec3 totalViscosityForce(0.0f);

            for (int dz = -1; dz <= 1; dz++) {
                for (int dy = -1; dy <= 1; dy++) {
                    for (int dx = -1; dx <= 1; dx++) {
                        // Use predictedPosition for neighborhood search, consistent with density/pressure
                        glm::ivec3 neighborCellCoords = getCellCoords(particle_i.predictedPosition, this->influenceRadius) + glm::ivec3(dx, dy, dz);
                        int neighborCellHash = getCellHashFromCoords(neighborCellCoords);
                        int neighborCellKey = getCellKeyFromHash(neighborCellHash);
                        if (neighborCellKey < 0 || neighborCellKey >= startIndices.size()) continue;
                        
                        int cellStartIndex = startIndices[neighborCellKey];
                        for (int k = cellStartIndex; k < numSpheres; ++k) {
                            const Entry& entry = spatialLookup[k];
                            if (entry.cellKey != neighborCellKey) break;

                            int particle_j_idx = entry.particleIndex;
                            if (i == particle_j_idx) continue; // Skip self

                            const SphereBV& particle_j = spheres[particle_j_idx];
                            // Use predicted positions for distance calculation, consistent with density/pressure
                            glm::vec3 r_ij_vec = particle_i.predictedPosition - particle_j.predictedPosition;
                            float distance = glm::length(r_ij_vec);

                            if (distance < this->influenceRadius && distance > 1e-6f) {
                                // Standard SPH viscosity formulation (e.g., Müller et al. 2003)
                                // F_visc_i = sum_j mu * m_j * (v_j - v_i) / rho_j * Laplacian_W_viscosity(r_ij, h)
                                float laplacian_W_visc = ViscosityKernelLaplacian(this->influenceRadius, distance);
                                

                                glm::vec3 viscosity_contribution = this->viscosityCoefficient * particle_j.mass *
                                                                  (particle_j.velocity - particle_i.velocity)  *
                                                                  laplacian_W_visc;
                                totalViscosityForce += viscosity_contribution;
                            }
                        }
                    }
                }
            }
            particle_i.force += totalViscosityForce; // Add accumulated viscosity force to particle_i
        }
    }

    void resetForces() {
        #pragma omp parallel for
        for (int i = 0; i < this->numSpheres; ++i) {
            this->spheres[i].resetForce();
        }
    }

    // Renamed from updateDensity to updateDensities (original non-optimized version)
    void updateDensities() { // Changed name to avoid conflict if old declaration was updateDensity()
        #pragma omp parallel for
        for (int i = 0; i < this->numSpheres; ++i) {
            this->spheres[i].density = calculateDensity(this->spheres[i].center);
        }
    }

    // Renamed from updateGravity to applyGravityForces
    void applyGravityForces(float gravityCoeff, float deltaTime) { // Changed name
        #pragma omp parallel for
        for (int i = 0; i < this->numSpheres; ++i) {
            this->spheres[i].applyGravity(gravityCoeff); // This adds gravity to spheres[i].force
            //Update their velocity for prediction
            //Predicted velocity and position
            // Use the passed deltaTime for prediction consistency
            glm::vec3 predictedVelocity = this->spheres[i].velocity + glm::vec3(0.0f, -gravityCoeff, 0.0f) * deltaTime; // Gravity force applied
            glm::vec3 predictedPosition = this->spheres[i].center + predictedVelocity * deltaTime; // Use deltaTime for position update
            // Update the sphere's predicted state
            this->spheres[i].predictedPosition = predictedPosition; // Store predicted position
        }

    }

    // Renamed from updatePressure to applyPressureForces (original non-optimized version)
    void applyPressureForces() { // Changed name
        #pragma omp parallel for
        for (int i = 0; i < this->numSpheres; ++i) {
            this->spheres[i].force += calculatePressureForce(this->spheres[i].center);
        }
    }


    static float SmoothingKernel(float h, float r){ // h is influenceRadius, r is distance
        if (r >= 0 && r < h) // Ensure r is not negative
        {
            // Poly6 kernel
            float h_sq = h * h;
            float r_sq = r * r;
            float factor = 315.0f / (64.0f * glm::pi<float>() * glm::pow(h, 9.0f));
            return factor * glm::pow(h_sq - r_sq, 3.0f);
        }
        return 0.0f;
    }

    static float SmoothingKernelDerivative(float h, float r){ // h is influenceRadius, r is distance
        if (r > 0 && r < h) // Ensure r is not negative and not zero to avoid issues with normalization
        {
            // Spiky kernel gradient magnitude (sign applied by direction vector)
            // Note: This is for the gradient, not the kernel itself. The original was a power of 2.
            // Using a common Spiky kernel derivative form:
            float factor = -45.0f / (glm::pi<float>() * glm::pow(h, 6.0f));
            return factor * glm::pow(h - r, 2.0f); // This is magnitude, direction is (x_i - x_j)/r
        }
        return 0.0f;
    }

    // Viscosity Kernel Laplacian (Müller et al. 2003)
    // W_viscosity(r, h) = (15 / (2 * pi * h^3)) * (-r^3 / (2*h^3) + r^2/h^2 + h/(2*r) - 1) -- This is the kernel itself
    // Laplacian_W_viscosity(r, h) = (45 / (pi * h^6)) * (h - r)
    static float ViscosityKernelLaplacian(float h, float r) {
        if (r >= 0 && r < h) { // Ensure r is not negative
            return (45.0f / (glm::pi<float>() * std::pow(h, 6.0f))) * (h - r);
        }
        return 0.0f;
    }
};

