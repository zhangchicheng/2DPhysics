#pragma once
#include <Shape.h>

class CircleShape : public Shape
{
public:
    float radius;

    CircleShape(float radius, float mass) : radius(radius) { this->mass = mass; }

    float GetMomentOfInertia() const override
    {
        // For a solid disk, I = 0.5 * m * r^2
        return 0.5f * mass * radius * radius;
    }

    Type GetType() const override
    {
        return CIRCLE;
    }

    std::unique_ptr<Shape> Clone() const override
    {
        return std::make_unique<CircleShape>(radius, mass);
    }
};