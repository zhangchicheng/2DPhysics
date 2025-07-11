#include <Particle.h> // Or "Particle.h" if your include path is set up for it
#include <Vec2.h>     // or "Vec2.h"

Particle::Particle(float x, float y, float mass)
{
    this->position = Vec2(x, y);
    this->velocity = Vec2(0.0f, 0.0f);
    this->acceleration = Vec2(0.0f, 0.0f);
    this->sumForces = Vec2(0.0f, 0.0f);
    SetMass(mass);
    this->radius = 5.0f;
}

// Safely sets the mass and inverse mass
void Particle::SetMass(float mass)
{
    if (mass <= 1e-6f)
    {
        this->inverseMass = 0.0f; // An inverse mass of 0 means the object is static
    }
    else
    {
        this->inverseMass = 1.0f / mass;
    }
}

// Function to Add Forces (Section 11)
// This function accumulates forces for the current frame.
void Particle::AddForce(const Vec2 &force)
{
    this->sumForces += force;
}

// Clears the force accumulator for the next frame
void Particle::ClearForces()
{
    this->sumForces = Vec2(0.0f, 0.0f);
}

// Integration & Simulation of Movement (Section 10)
// We use Semi-implicit Euler integration, which is a simple and stable method.
// It's a "discrete" simulation of a "continuous" physical process.
void Particle::Integrate(float dt)
{
    // If inverseMass is zero, the object is static and should not move.
    if (this->inverseMass == 0.0f)
    {
        return;
    }

    // 1. Calculate acceleration from forces (Newton's 2nd Law: a = F/m or a = F * inv_m)
    this->acceleration = this->sumForces * this->inverseMass;

    // 2. Integrate acceleration to find new velocity
    this->velocity += this->acceleration * dt;

    // 3. Integrate velocity to find new position
    this->position += this->velocity * dt;

    // 4. Clear the force accumulator for the next integration step.
    ClearForces();
}