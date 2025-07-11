#pragma once
#include <memory>

// Forward declare the Body class
class Body;

class Shape
{
public:
    enum Type
    {
        CIRCLE,
        POLYGON
    };

    virtual ~Shape() = default;

    // Pure virtual functions that all derived shapes MUST implement
    virtual float GetMomentOfInertia() const = 0;
    virtual Type GetType() const = 0;

    // The "virtual constructor" pattern is a very useful C++ trick
    virtual std::unique_ptr<Shape> Clone() const = 0;

    float mass; // The shape is responsible for its mass
};