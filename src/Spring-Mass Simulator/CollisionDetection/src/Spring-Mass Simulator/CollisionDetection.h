// filepath: d:\2025Spring\CMSC838M\Homework1\ProjectSourceCode\CollisionDetection\src\Spring-Mass Simulator\CollisionDetection.h
#pragma once
#include <glm/glm.hpp>
#include "SphereBV.h"
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>
#include <unordered_set>
#include "Utils.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/norm.hpp>

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
    CollisionDetection(SphereBV* spheres, int numSpheres, float worldSize, std::vector<std::pair<SphereBV*, SphereBV*>>* collisionPairs, int method = 0)
        : spheres(spheres), numSpheres(numSpheres), worldSize(worldSize), collisionPairs(collisionPairs), method(method) {
        collisionPairs->clear();
    }

    ~CollisionDetection() {}

    void broadCollisionDetection() {
        collisionPairs->clear();
        for (int i = 0; i < numSpheres; i++) {
            for (int j = i + 1; j < numSpheres; j++) {
                if (spheres[i].intersects(spheres[j])) {
                    collisionPairs->emplace_back(&spheres[i], &spheres[j]);
                }
            }
        }
    }

    void narrowCollisionDetection() {
        for (auto &pair : *collisionPairs) {
            if (!GJK(pair.first, pair.second)) {
                collisionPairs->erase(std::remove(collisionPairs->begin(), collisionPairs->end(), pair), collisionPairs->end());
            }
        }
    }

    void handleCollision() {
        const float e = 0.8f; // Coefficient of restitution
        for (auto &pr : *collisionPairs) {
            SphereBV* A = pr.first;
            SphereBV* B = pr.second;

            glm::vec3 delta = B->center - A->center;
            float dist = glm::length(delta);
            if (dist > 1e-6f) {
                float penetration = A->radius + B->radius - dist;
                if (penetration > 0.0f) {
                    glm::vec3 normal = glm::normalize(delta);
                    A->center -= normal * (penetration * 0.5f);
                    B->center += normal * (penetration * 0.5f);
                }
            }

            glm::vec3 vA = A->P / A->mass;
            glm::vec3 vB = B->P / B->mass;
            float mA = A->mass, mB = B->mass;
            glm::vec3 vA_after = ((mA - mB) / (mA + mB)) * vA + (2 * mB / (mA + mB)) * vB;
            glm::vec3 vB_after = (2 * mA / (mA + mB)) * vA + ((mB - mA) / (mA + mB)) * vB;

            A->P = vA_after * mA;
            B->P = vB_after * mB;
            A->velocity = vA_after;
            B->velocity = vB_after;
        }
    }

    bool GJK(SphereBV* A, SphereBV* B) {
        glm::vec3 d = A->center - B->center;
        if (glm::length(d) < 1e-6) return false;

        std::vector<glm::vec3> simplex;
        glm::vec3 supportPoint = support(A, B, d);
        simplex.push_back(supportPoint);
        d = -supportPoint;

        const int MAX_ITERATIONS = 20;
        int iterations = 0;

        while (iterations < MAX_ITERATIONS) {
            supportPoint = support(A, B, d);
            if (glm::dot(supportPoint, d) < 0) return false;

            simplex.push_back(supportPoint);
            if (handleSimplex(simplex, d)) return true;

            iterations++;
        }
        return false;
    }

private:
    SphereBV* spheres;
    int numSpheres;
    float worldSize;
    std::vector<std::pair<SphereBV*, SphereBV*>>* collisionPairs;
    int method;

    glm::vec3 support(SphereBV* A, SphereBV* B, const glm::vec3 &d) {
        glm::vec3 dir = glm::normalize(d);
        return A->center + dir * A->radius; // Simplified for spheres
    }

    bool handleSimplex(std::vector<glm::vec3>& simplex, glm::vec3 &d) {
        // Handle simplex cases (line, triangle, tetrahedron)
        // Implementation omitted for brevity
        return false; // Placeholder
    }
};