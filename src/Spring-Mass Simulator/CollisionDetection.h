#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp> // For glm::inverse
#include "SphereBV.h"
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>
#include <unordered_set>
#include "utils.h"
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/norm.hpp>
#include <omp.h> // Include OpenMP for parallel processing
#include <algorithm> // For std::sort

// Represent the endpoint of beginning and end
struct Point {
    float value;
    bool isBeginning;
    int id;

    bool operator<(const Point &other) const {
        return value < other.value || (value == other.value && isBeginning && !other.isBeginning);
    }
}; 

class CollisionDetection
{
public:
    // Constructor
    CollisionDetection(SphereBV* spheres, int numSpheres, float localWorldBoundarySize, const glm::mat4& worldTransform, std::vector<std::pair<SphereBV*, SphereBV*>>* collisionPairs, int method = 0)
        : spheres_(spheres), numSpheres_(numSpheres), localWorldSize_(localWorldBoundarySize), worldTransform_(worldTransform), collisionPairs_(collisionPairs), method_(method) {
        collisionPairs_->clear();
        inverseWorldTransform_ = glm::inverse(worldTransform_);
    }

    // Destructor - Modified to not delete spheres, since it doesn't own them
    ~CollisionDetection() {
        // Remove the delete[] spheres line since this class doesn't own the memory
    }

    // Broad Collision Detection
    void broadCollisionDetection(float damping = 0.9f) {
        // collisionPairs_->clear(); // This is for inter-sphere, boundary is handled directly

        #pragma omp parallel for
        for(int i = 0; i < numSpheres_; i++){
            SphereBV& s = spheres_[i];
            glm::vec3 sphereCenterWorld = s.center;
            
            glm::vec3 sphereCenterLocal = glm::vec3(inverseWorldTransform_ * glm::vec4(sphereCenterWorld, 1.0f));

            bool collisionWithBoundary = false;
            glm::vec3 cumulativeLocalCorrection(0.0f);
            glm::vec3 finalLocalNormal(0.0f); // To store the normal of the last collision for reflection

            // X-axis
            if (sphereCenterLocal.x + s.radius > localWorldSize_) {
                float overlap = (sphereCenterLocal.x + s.radius) - localWorldSize_;
                cumulativeLocalCorrection.x -= overlap; // Move left
                finalLocalNormal = glm::vec3(-1.0f, 0.0f, 0.0f);
                collisionWithBoundary = true;
            } else if (sphereCenterLocal.x - s.radius < -localWorldSize_) {
                float overlap = (-localWorldSize_) - (sphereCenterLocal.x - s.radius);
                cumulativeLocalCorrection.x += overlap; // Move right
                finalLocalNormal = glm::vec3(1.0f, 0.0f, 0.0f);
                collisionWithBoundary = true;
            }

            // Y-axis
            if (sphereCenterLocal.y + s.radius > localWorldSize_) {
                float overlap = (sphereCenterLocal.y + s.radius) - localWorldSize_;
                cumulativeLocalCorrection.y -= overlap; // Move down
                finalLocalNormal = glm::vec3(0.0f, -1.0f, 0.0f);
                collisionWithBoundary = true;
            } else if (sphereCenterLocal.y - s.radius < -localWorldSize_) {
                float overlap = (-localWorldSize_) - (sphereCenterLocal.y - s.radius);
                cumulativeLocalCorrection.y += overlap; // Move up
                finalLocalNormal = glm::vec3(0.0f, 1.0f, 0.0f);
                collisionWithBoundary = true;
            }

            // Z-axis
            if (sphereCenterLocal.z + s.radius > localWorldSize_) {
                float overlap = (sphereCenterLocal.z + s.radius) - localWorldSize_;
                cumulativeLocalCorrection.z -= overlap; // Move backward
                finalLocalNormal = glm::vec3(0.0f, 0.0f, -1.0f);
                collisionWithBoundary = true;
            } else if (sphereCenterLocal.z - s.radius < -localWorldSize_) {
                float overlap = (-localWorldSize_) - (sphereCenterLocal.z - s.radius);
                cumulativeLocalCorrection.z += overlap; // Move forward
                finalLocalNormal = glm::vec3(0.0f, 0.0f, 1.0f);
                collisionWithBoundary = true;
            }

            if (collisionWithBoundary) {
                // Apply positional correction: transform local correction vector to world space and add
                glm::vec3 worldCorrection = glm::mat3(worldTransform_) * cumulativeLocalCorrection;
                s.center += worldCorrection;
                
                // Transform the dominant local normal (or last one encountered) to world space
                glm::vec3 worldNormal = glm::normalize(glm::mat3(worldTransform_) * finalLocalNormal);
                
                float dotPN = glm::dot(s.P, worldNormal);
                // Only reflect if moving towards the boundary plane that was hit
                // A more robust check would be if dot(s.velocity, worldNormal) < 0
                if (glm::dot(s.velocity, worldNormal) < 0.0f) { 
                    s.P -= (1.0f + damping) * dotPN * worldNormal; 
                }
                
                s.velocity = s.P / s.mass;
            }
        }
        
        // Inter-sphere collision detection
        if (method_ != 2) { 
            for(int i = 0; i < numSpheres_; i++){
                for(int j = i + 1; j < numSpheres_; j++){
                    if(spheres_[i].intersects(spheres_[j])){ 
                        collisionPairs_->push_back({&spheres_[i], &spheres_[j]});
                    }
                }
            }
        }
    }

    //Narrow Collision Detection using the standard GJK algorithm
    void narrowCollisionDetection() {
        for(auto &pair : *collisionPairs_){
            SphereBV* sphereA = pair.first;
            SphereBV* sphereB = pair.second;

            // Check if the spheres are colliding using GJK algorithm
            if(!GJK(sphereA, sphereB)){
                // If not colliding, remove the pair from the collision pairs
                collisionPairs_->erase(std::remove(collisionPairs_->begin(), collisionPairs_->end(), pair), collisionPairs_->end());
            }
        }

    }

    //Simply reverse the velocity between two possible spheres
    void handleCollision() {
        for(auto &pair : *collisionPairs_){
            SphereBV* A = pair.first;
            SphereBV* B = pair.second;

            // 1) positional correction to resolve penetration
            glm::vec3 delta = B->center - A->center;
            float dist = glm::length(delta);
            if(dist > 1e-6f) {
                float penetration = A->radius + B->radius - dist;
                if(penetration > 0.0f) {
                    glm::vec3 normal = delta / dist;
                    A->center -= normal * (penetration * 0.3f);
                    B->center += normal * (penetration * 0.3f);
                }
            }

            // --- existing impulse-based velocity response ---
            glm::vec3 vA = A->P / A->mass;
            glm::vec3 vB = B->P / B->mass;
            float mA = A->mass, mB = B->mass;
            glm::vec3 vA_after = ((mA - mB)/(mA + mB))*vA + (2*mB/(mA+mB))*vB;
            glm::vec3 vB_after = (2*mA/(mA + mB))*vA + ((mB - mA)/(mA + mB))*vB;

            // update momentum (state) and keep velocity in sync
            A->P = vA_after * mA;
            B->P = vB_after * mB;
            A->velocity = vA_after;
            B->velocity = vB_after;
        }
    }

    // GJK main function
    // Check if two spheres are colliding using the GJK algorithm
    bool GJK(SphereBV* A, SphereBV* B) {
        // Initial search direction
        glm::vec3 d = A->center - B->center;
        if(glm::length(d) < 1e-6)
            d = glm::vec3(1.0f, 0.0f, 0.0f);
        
        std::vector<glm::vec3> simplex;
        glm::vec3 supportPoint = support(A, B, d);
        simplex.push_back(supportPoint);
        d = -supportPoint;
        
        // Add maximum iterations to prevent infinite loops
        const int MAX_ITERATIONS = 20;
        int iterations = 0;
        
        while (iterations < MAX_ITERATIONS) {
            iterations++;
            glm::vec3 newPoint = support(A, B, d);
            
            if (glm::dot(newPoint, d) < 0)
                return false;  // New point did not pass the origin in direction d
                
            // Check if new point is not significantly different from existing points
            bool duplicate = false;
            for (const auto& p : simplex) {
                if (glm::length(p - newPoint) < 1e-6) {
                    duplicate = true;
                    break;
                }
            }
            
            if (duplicate)
                return false; 
                
            simplex.push_back(newPoint);
            if (handleSimplex(simplex, d))
                return true;   // Simplex contains origin, collision
        }
        return false;
    }



private:
    SphereBV* spheres_; 
    int numSpheres_;
    float localWorldSize_; 
    glm::mat4 worldTransform_;
    glm::mat4 inverseWorldTransform_;
    std::vector<std::pair<SphereBV*, SphereBV*>>* collisionPairs_;
    int method_;

    // Support function for GJK algorithm
    glm::vec3 support(SphereBV* A, SphereBV* B, const glm::vec3 &d);

    // Triple cross product: (a x b) x c
    glm::vec3 tripleCross(const glm::vec3 &a, const glm::vec3 &b, const glm::vec3 &c);

    // Simplex handling for a line (2 points)
    bool handleLine(std::vector<glm::vec3>& simplex, glm::vec3 &d);

    // Simplex handling for a triangle (3 points)
    bool handleTriangle(std::vector<glm::vec3>& simplex, glm::vec3 &d) {
        // Simplex points are C, B, A (A is newest)
        glm::vec3 A = simplex[2];
        glm::vec3 B = simplex[1];
        glm::vec3 C = simplex[0];
        glm::vec3 AO = -A;
        glm::vec3 AB = B - A;
        glm::vec3 AC = C - A;
        glm::vec3 ABC = glm::cross(AB, AC); // Normal to triangle ABC
        
        // Check if origin is in region of AB edge (outside triangle, towards AB)
        if (glm::dot(glm::cross(AB, ABC), AO) > 0) { // AB_perp towards outside
            simplex.erase(simplex.begin()); // Remove C
            d = tripleCross(AB, AO, AB);    // New direction towards origin from line AB
            return false;
        }
        // Check if origin is in region of AC edge (outside triangle, towards AC)
        if (glm::dot(glm::cross(ABC, AC), AO) > 0) { // AC_perp towards outside
            simplex.erase(simplex.begin() + 1); // Remove B
            d = tripleCross(AC, AO, AC);    // New direction towards origin from line AC
            return false;
        }
        // Origin is above or below the triangle
        if (glm::dot(ABC, AO) > 0) { // Origin is on the side of ABC normal (e.g. "above")
            d = ABC; // New direction is normal itself
        } else { // Origin is on the other side (e.g. "below")
            std::swap(simplex[0], simplex[1]); // Swap C and B to reverse normal (ACB)
            d = -ABC; // New direction is opposite normal
        }
        return false;
    }

    // Simplex handling for a tetrahedron (4 points)
    bool handleTetrahedron(std::vector<glm::vec3>& simplex, glm::vec3 &d) {
        // Simplex points D, C, B, A (A is newest)
        glm::vec3 A = simplex[3];
        glm::vec3 B = simplex[2];
        glm::vec3 C = simplex[1];
        glm::vec3 D = simplex[0];
        glm::vec3 AO = -A; // Vector from A to origin

        glm::vec3 ABC = glm::cross(B - A, C - A); // Normal of face ABC, pointing outwards if B-A, C-A are CCW from outside
        glm::vec3 ACD = glm::cross(C - A, D - A); // Normal of face ACD
        glm::vec3 ADB = glm::cross(D - A, B - A); // Normal of face ADB

        // Check face ABC
        if (glm::dot(ABC, AO) > 0) { // If origin is outside face ABC (in direction of its normal)
            simplex.erase(simplex.begin()); // Remove D (point not on face ABC)
            d = ABC; // New direction is normal of ABC
            return false; // Reduce to triangle case
        }
        // Check face ACD
        if (glm::dot(ACD, AO) > 0) { // If origin is outside face ACD
            simplex.erase(simplex.begin() + 2); // Remove B (point not on face ACD)
            d = ACD; // New direction is normal of ACD
            return false; // Reduce to triangle case
        }
        // Check face ADB
        if (glm::dot(ADB, AO) > 0) { // If origin is outside face ADB
            simplex.erase(simplex.begin() + 1); // Remove C (point not on face ADB)
            d = ADB; // New direction is normal of ADB
            return false; // Reduce to triangle case
        }
        // Origin is inside the tetrahedron
        return true;
    }

    // Selects the appropriate handler based on the number of points in the simplex.
    bool handleSimplex(std::vector<glm::vec3>& simplex, glm::vec3 &d) {
        if (simplex.size() == 2)
            return handleLine(simplex, d);
        else if (simplex.size() == 3)
            return handleTriangle(simplex, d);
        else if (simplex.size() == 4)
            return handleTetrahedron(simplex, d);
        return false; // Should not be reached in a correct GJK
    }
};