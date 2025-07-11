#pragma once
#include <Shape.h>
#include <Vec2.h>
#include <vector>

class PolygonShape : public Shape
{
public:
    std::vector<Vec2> localVertices;
    std::vector<Vec2> worldVertices;

    PolygonShape(const std::vector<Vec2> &vertices, float mass)
    {
        this->mass = mass;
        localVertices = vertices;
    }

    float GetMomentOfInertia() const override
    {
        // For simplicity, we'll approximate the moment of inertia for a box.
        // A more accurate calculation would involve complex polygon integration.
        // Let's assume the vertices define a box-like shape.
        return 100000.0f; // A placeholder value.
    }

    Type GetType() const override
    {
        return POLYGON;
    }

    std::unique_ptr<Shape> Clone() const override
    {
        return std::make_unique<PolygonShape>(localVertices, mass);
    }
};