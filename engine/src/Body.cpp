#include <Body.h>
#include <PolygonShape.h> // We need the full definition here
#include <iostream>

Body::Body(const Shape &shape, float x, float y)
{
    this->position = Vec2(x, y);
    this->velocity = Vec2(0.0, 0.0);
    this->sumForces = Vec2(0.0, 0.0);

    this->angle = 0.0;
    this->angularVelocity = 0.0;
    this->sumTorque = 0.0;

    this->shape = shape.Clone();

    if (this->shape->mass <= 1e-6f)
    {
        this->inverseMass = 0.0f;
    }
    else
    {
        this->inverseMass = 1.0f / this->shape->mass;
    }

    float inertia = this->shape->GetMomentOfInertia();
    if (inertia <= 1e-6f)
    {
        this->inverseInertia = 0.0f;
    }
    else
    {
        this->inverseInertia = 1.0f / inertia;
    }

    // If it's a polygon, initialize world vertices
    if (this->shape->GetType() == Shape::POLYGON)
    {
        UpdateWorldVertices();
    }
}

void Body::AddForce(const Vec2 &force)
{
    sumForces += force;
}

void Body::AddTorque(float torque)
{
    sumTorque += torque;
}

void Body::ClearForces()
{
    sumForces = Vec2(0.0, 0.0);
}

void Body::ClearTorque()
{
    sumTorque = 0.0;
}

void Body::Integrate(float dt)
{
    // Linear motion integration
    if (inverseMass > 0.0f)
    {
        Vec2 acceleration = sumForces * inverseMass;
        velocity += acceleration * dt;
        position += velocity * dt;
    }

    // Angular motion integration
    if (inverseInertia > 0.0f)
    {
        float angularAcceleration = sumTorque * inverseInertia;
        angularVelocity += angularAcceleration * dt;
        angle += angularVelocity * dt;
    }

    // Update polygon vertices if needed
    if (shape->GetType() == Shape::POLYGON)
    {
        UpdateWorldVertices();
    }

    ClearForces();
    ClearTorque();
}

void Body::UpdateWorldVertices()
{
    PolygonShape *polygonShape = static_cast<PolygonShape *>(shape.get());
    polygonShape->worldVertices.clear();
    for (const auto &v : polygonShape->localVertices)
    {
        polygonShape->worldVertices.push_back(position + v.Rotated(angle));
    }
}