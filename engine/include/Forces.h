#pragma once

#include <Vec2.h>
#include <Particle.h>
#include <cmath>

namespace Forces
{

    // Weight Force (constant downward gravity)
    inline Vec2 GenerateWeightForce(const Particle &particle, float gravityConstant)
    {
        if (particle.inverseMass == 0.0f)
            return Vec2(0.0f, 0.0f);
        float mass = 1.0f / particle.inverseMass;
        return Vec2(0.0f, mass * gravityConstant);
    }

    // Drag Force (opposes velocity)
    inline Vec2 GenerateDragForce(const Particle &particle, float k)
    {
        if (particle.velocity.MagnitudeSq() > 0)
        {
            Vec2 dragForce = particle.velocity.Normalized() * -k * particle.velocity.MagnitudeSq();
            return dragForce;
        }
        return Vec2(0.0f, 0.0f);
    }

    //------------------------------------------------------------------------------------
    // NEW FORCES
    //------------------------------------------------------------------------------------

    // Gravitational Attraction Force (Section 13)
    // A force that pulls two particles together based on their mass.
    // F = G * (m1 * m2) / r^2
    inline Vec2 GenerateGravitationalForce(const Particle &a, const Particle &b, float G)
    {
        // --- THIS IS THE FIX ---
        // If either particle is static, gravity can't be calculated with this model.
        if (a.inverseMass == 0.0f || b.inverseMass == 0.0f)
        {
            return Vec2(0.0f, 0.0f);
        }
        // --- END FIX ---

        Vec2 d = b.position - a.position;
        float distanceSq = d.MagnitudeSq();

        if (distanceSq < 1.0f)
        {
            distanceSq = 1.0f;
        }

        float forceMagnitude = G * (1.0f / a.inverseMass) * (1.0f / b.inverseMass) / distanceSq;

        return d.Normalized() * forceMagnitude;
    }

    // Spring Force (Section 14)
    // A force that connects two particles, like a spring.
    // F = -k * x  (Hooke's Law)
    inline Vec2 GenerateSpringForce(const Particle &a, const Particle &b, float restLength, float k)
    {
        Vec2 d = b.position - a.position;
        float currentLength = d.Magnitude();

        // The displacement from the spring's rest length
        float displacement = currentLength - restLength;

        // The force is along the direction vector, scaled by displacement and spring constant
        return d.Normalized() * (k * displacement);
    }

} // namespace Forces