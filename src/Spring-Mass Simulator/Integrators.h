#pragma once

#include "SphereBV.h"
#include <glm/glm.hpp>
#include <functional>
#include <stdexcept> // For runtime_error

// Helper operator overloads for SphereBV state arithmetic needed for RK4

// Addition: Adds the state variables of two SphereBV objects.
// Note: Constant quantities like mass, radius, etc., are taken from the left-hand side operand.
inline SphereBV operator+(const SphereBV& lhs, const SphereBV& rhs) {
    SphereBV result = lhs; // Copy constant quantities and initial state
    result.center += rhs.center; // Add derivative contribution (velocity * dt)
    result.P += rhs.P;           // Add derivative contribution (force * dt)
    return result;
}

// Scalar Multiplication: Scales the state variables of a SphereBV object.
// Note: Constant quantities are not scaled.
inline SphereBV operator*(const SphereBV& s, float scalar) {
    SphereBV result = s; // Copy constant quantities
    // Scale only the derivative parts (which are stored in the state variables of the 'derivative' SphereBV)
    result.center *= scalar; // Represents velocity * scalar
    result.P *= scalar;      // Represents force * scalar
    return result;
}

inline SphereBV operator*(float scalar, const SphereBV& s) {
    return s * scalar; // Commutative
}

//Euler Integrator for SphereBV state
inline SphereBV EulerIntegrator(SphereBV currentState, float dt) {
    // Calculate the derivative of the current state
    SphereBV derivative = currentState.derivative();

    // Update the state using the derivative and time step
    SphereBV newState = currentState + derivative * dt;

    return newState;
}

// Midpoint Integrator for SphereBV state
inline SphereBV MidpointIntegrator(SphereBV currentState, float dt) {
    // Calculate the derivative at the current state
    SphereBV k1 = currentState.derivative();

    // Calculate the state at the midpoint using the derivative
    SphereBV midState = currentState + k1 * (dt / 2.0f);

    // Calculate the derivative at the midpoint state
    SphereBV k2 = midState.derivative();

    // Update the state using the midpoint derivative
    SphereBV newState = currentState + k2 * dt;


    return newState;
}

// RK4 Integrator for SphereBV state
inline SphereBV RK4Integrator(SphereBV currentState, float dt) {
    // Calculate the derivatives at different time points
    SphereBV k1 = currentState.derivative();
    SphereBV k2 = (currentState + k1 * (dt / 2.0f)).derivative();
    SphereBV k3 = (currentState + k2 * (dt / 2.0f)).derivative();
    SphereBV k4 = (currentState + k3 * dt).derivative();

    // Combine the derivatives to get the final state update
    // Note: The operator+ and operator* handle adding/scaling the relevant state variables
    SphereBV weightedSum = (k1 + 2.0f * k2 + 2.0f * k3 + k4);
    SphereBV finalState = currentState + weightedSum * (dt / 6.0f);


    return finalState;
}

// Velocity Verlet Integrator (Simplified: Verlet for linear, Euler for angular)
// Note: Uses force/torque from the beginning of the step (t) for updates.
// A full Velocity Verlet requires force at t+dt for the velocity update.
inline SphereBV VelocityVerletIntegrator(SphereBV currentState, float dt) {
    if (currentState.mass <= 0.0f) {
        // Handle static or invalid objects if necessary, maybe return currentState
         // Or throw an error if mass should always be positive
         throw std::runtime_error("VelocityVerletIntegrator: Mass must be positive.");
         // return currentState; // Alternative: treat as static
    }

    SphereBV newState = currentState; // Start with current state

    // --- Linear Update (Velocity Verlet using a(t)) ---
    glm::vec3 currentAcceleration = currentState.force / currentState.mass;

    // 1. Update position
    newState.center = currentState.center + currentState.velocity * dt + 0.5f * currentAcceleration * dt * dt;

    // 2. Update velocity (Simplified: using only a(t))
    //    A more accurate version would need a(t+dt) calculated based on newState.center
    //    v(t+dt) = v(t) + 0.5 * (a(t) + a(t+dt)) * dt
    //    Here we use: v(t+dt) = v(t) + a(t) * dt
    newState.velocity = currentState.velocity + currentAcceleration * dt;

    // 3. Update linear momentum
    newState.P = newState.velocity * currentState.mass;


    return newState;
}

