#pragma once

#include "Vec2.h"

// Kinematics: The study of motion (position, velocity, acceleration).
// Kinetics: The study of the forces that cause motion.
// Our Particle class contains both kinematic properties and kinetic methods.

class Particle
{
public:
    // Kinematic Properties
    Vec2 position;
    Vec2 velocity;

    // Kinetic Properties
    Vec2 acceleration;
    Vec2 sumForces;    // Accumulates all forces applied in a frame
    float inverseMass; // 1.0f / mass. A value of 0.0f means infinite mass (static object).

    float radius;

    // Constructors
    Particle() = default;
    Particle(float x, float y, float mass);

    // Methods
    void SetMass(float mass);
    void AddForce(const Vec2 &force);
    void ClearForces();

    // The "Particle Integrate Function" from Section 10
    // This function updates the particle's state using numerical integration.
    void Integrate(float dt);
};