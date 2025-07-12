#pragma once

#include <Body.h>
#include <CircleShape.h>
#include <PolygonShape.h>
#include <limits>

// Section 17: Collision Contact Information
// This struct holds all the information about a collision event.
struct CollisionInfo
{
    Body *a;
    Body *b;
    float penetrationDepth;
    Vec2 collisionNormal;
    Vec2 contactPoint;
};

class Collision
{
private:
    // Helper for SAT: Projects the vertices of a polygon onto an axis
    static void ProjectVertices(const PolygonShape *polygon, const Vec2 &axis, float &min, float &max)
    {
        min = std::numeric_limits<float>::max();
        max = std::numeric_limits<float>::min();
        for (const auto &v : polygon->worldVertices)
        {
            float projection = v.Dot(axis);
            if (projection < min)
                min = projection;
            if (projection > max)
                max = projection;
        }
    }

public:
    static bool PolygonPolygonCollision(CollisionInfo &info)
    {
        PolygonShape *polyA = static_cast<PolygonShape *>(info.a->shape.get());
        PolygonShape *polyB = static_cast<PolygonShape *>(info.b->shape.get());

        float minOverlap = std::numeric_limits<float>::max();
        Vec2 smallestAxis;

        // Loop through axes of Polygon A
        for (size_t i = 0; i < polyA->worldVertices.size(); ++i)
        {
            Vec2 v1 = polyA->worldVertices[i];
            Vec2 v2 = polyA->worldVertices[(i + 1) % polyA->worldVertices.size()];
            Vec2 edge = v2 - v1;
            Vec2 axis = edge.Perpendicular().Normalized();

            float minA, maxA, minB, maxB;
            ProjectVertices(polyA, axis, minA, maxA);
            ProjectVertices(polyB, axis, minB, maxB);

            if (maxA < minB || maxB < minA)
            {
                return false; // Found a separating axis, no collision
            }

            // Calculate overlap and update if it's the smallest
            float overlap = std::min(maxA, maxB) - std::max(minA, minB);
            if (overlap < minOverlap)
            {
                minOverlap = overlap;
                smallestAxis = axis;
            }
        }

        // Loop through axes of Polygon B
        for (size_t i = 0; i < polyB->worldVertices.size(); ++i)
        {
            Vec2 v1 = polyB->worldVertices[i];
            Vec2 v2 = polyB->worldVertices[(i + 1) % polyB->worldVertices.size()];
            Vec2 edge = v2 - v1;
            Vec2 axis = edge.Perpendicular().Normalized();

            float minA, maxA, minB, maxB;
            ProjectVertices(polyA, axis, minA, maxA);
            ProjectVertices(polyB, axis, minB, maxB);

            if (maxA < minB || maxB < minA)
            {
                return false; // Found a separating axis, no collision
            }

            float overlap = std::min(maxA, maxB) - std::max(minA, minB);
            if (overlap < minOverlap)
            {
                minOverlap = overlap;
                smallestAxis = axis;
            }
        }

        // If we get here, there is a collision. Populate the info struct.
        info.penetrationDepth = minOverlap;
        info.collisionNormal = smallestAxis;

        // Ensure normal points from A to B
        Vec2 dir = info.b->position - info.a->position;
        if (info.collisionNormal.Dot(dir) < 0)
        {
            info.collisionNormal = -info.collisionNormal;
        }

        // A simple (but not perfect) way to find contact point
        info.contactPoint = info.a->position; // Placeholder

        return true;
    }
    // Section 17: Coding the Circle-Circle Collision Information
    // This is the "Narrow Phase" of collision detection for two circles.
    static bool CircleCircleCollision(CollisionInfo &info)
    {
        Body *a = info.a;
        Body *b = info.b;
        CircleShape *circleA = static_cast<CircleShape *>(a->shape.get());
        CircleShape *circleB = static_cast<CircleShape *>(b->shape.get());

        const float sumRadii = circleA->radius + circleB->radius;
        const Vec2 distanceVec = b->position - a->position;
        const float distance = distanceVec.Magnitude();

        // If the distance is greater than the sum of radii, there is no collision.
        if (distance > sumRadii)
        {
            return false;
        }

        // Collision has occurred. Populate the rest of the info struct.
        info.penetrationDepth = sumRadii - distance;
        info.collisionNormal = distanceVec.Normalized();
        return true;
    }

    static bool CirclePolygonCollision(CollisionInfo &info)
    {
        // ... (This is complex, we can add it later. For now, we return false) ...
        return false;
    }

    // Section 18: Coding the Linear Impulse Function
    // This function resolves a collision by applying an impulse.
    static void ResolveCollision(CollisionInfo &info)
    {
        // Separate the colliding bodies
        const float percent = 0.8f;
        Vec2 separation = info.collisionNormal * (info.penetrationDepth / (info.a->inverseMass + info.b->inverseMass)) * percent;
        if (info.a->inverseMass != 0)
            info.a->position -= separation * info.a->inverseMass;
        if (info.b->inverseMass != 0)
            info.b->position += separation * info.b->inverseMass;

        // For now, we will use the old linear impulse resolution.
        // Implementing the full angular resolution is the next step.
        const Vec2 relativeVelocity = info.b->velocity - info.a->velocity;
        const float relativeSpeed = relativeVelocity.Dot(info.collisionNormal);
        const float e = 0.5f; // Bounciness

        float numerator = -(1.0f + e) * relativeSpeed;
        float denominator = info.a->inverseMass + info.b->inverseMass;
        float j = (denominator == 0) ? 0 : numerator / denominator;

        Vec2 impulse = info.collisionNormal * j;
        if (info.a->inverseMass != 0)
            info.a->velocity -= impulse * info.a->inverseMass;
        if (info.b->inverseMass != 0)
            info.b->velocity += impulse * info.b->inverseMass;
    }

    static void DetectAndResolveCollisions(std::vector<std::unique_ptr<Body>> &bodies)
    {
        for (size_t i = 0; i < bodies.size(); ++i)
        {
            for (size_t j = i + 1; j < bodies.size(); ++j)
            {
                Body *a = bodies[i].get();
                Body *b = bodies[j].get();
                CollisionInfo info = {a, b};

                bool collided = false;
                if (a->shape->GetType() == Shape::CIRCLE && b->shape->GetType() == Shape::CIRCLE)
                {
                    collided = CircleCircleCollision(info);
                }
                else if (a->shape->GetType() == Shape::POLYGON && b->shape->GetType() == Shape::POLYGON)
                {
                    collided = PolygonPolygonCollision(info);
                }
                else
                {
                    // collided = CirclePolygonCollision(info);
                }

                if (collided)
                {
                    ResolveCollision(info);
                }
            }
        }
    }
};