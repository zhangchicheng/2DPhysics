#pragma once
#include <Vec2.h>
#include <Shape.h>
#include <memory>

class Body
{
public:
    // Linear motion
    Vec2 position;
    Vec2 velocity;
    Vec2 sumForces;

    // Angular motion
    float angle = 0.0f; // Orientation in radians
    float angularVelocity = 0.0f;
    float sumTorque = 0.0f;

    // Physical properties
    float inverseMass;
    float inverseInertia;

    std::unique_ptr<Shape> shape;

    Body(const Shape &shape, float x, float y);

    void AddForce(const Vec2 &force);
    void AddTorque(float torque);
    void ClearForces();
    void ClearTorque();

    void Integrate(float dt);

    // Helper to transform shape vertices to world space
    void UpdateWorldVertices();
};