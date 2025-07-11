#pragma once

#include <cmath>
#include <iostream>

// A structure to represent a 2D vector or a point.
struct Vec2 {
    float x = 0.0f;
    float y = 0.0f;

    // --- Constructors ---
    Vec2() = default;
    Vec2(float x, float y) : x(x), y(y) {}

    // --- Basic Arithmetic Operators ---
    Vec2 operator+(const Vec2& other) const { return Vec2(x + other.x, y + other.y); }
    Vec2 operator-(const Vec2& other) const { return Vec2(x - other.x, y - other.y); }
    Vec2 operator-() const { return Vec2(-x, -y); }
    Vec2 operator*(float scalar) const { return Vec2(x * scalar, y * scalar); }
    Vec2 operator/(float scalar) const { return Vec2(x / scalar, y / scalar); }

    // --- Compound Assignment Operators ---
    Vec2& operator+=(const Vec2& other) { x += other.x; y += other.y; return *this; }
    Vec2& operator-=(const Vec2& other) { x -= other.x; y -= other.y; return *this; }
    Vec2& operator*=(float scalar) { x *= scalar; y *= scalar; return *this; }
    Vec2& operator/=(float scalar) { x /= scalar; y /= scalar; return *this; }

    // --- Vector Properties and Methods ---

    // Calculate the magnitude (length) of the vector
    float Magnitude() const { return std::sqrt(x * x + y * y); }
    // Calculate the squared magnitude (avoids sqrt)
    float MagnitudeSq() const { return x * x + y * y; }

    // Corresponds to Section 5: Vectors Normalization
    // Normalize the vector to unit length (1)
    void Normalize() {
        float mag = Magnitude();
        if (mag > 1e-5f) { // Avoid division by zero or very small numbers
            *this /= mag;
        }
    }
    // Return a normalized copy
    Vec2 Normalized() const {
        Vec2 result = *this;
        result.Normalize();
        return result;
    }

    // --- Dot and Cross Product ---
    float Dot(const Vec2& other) const { return (x * other.x) + (y * other.y); }
    float Cross(const Vec2& other) const { return (x * other.y) - (y * other.x); }
    
    // --- Vector Transformations ---
    // Corresponds to Section 6: Vector Transformations
    // Rotate the vector by an angle in radians
    void Rotate(float angleRadians) {
        const float cosAngle = std::cos(angleRadians);
        const float sinAngle = std::sin(angleRadians);
        const float newX = x * cosAngle - y * sinAngle;
        const float newY = x * sinAngle + y * cosAngle;
        x = newX;
        y = newY;
    }
    // Return a rotated copy of the vector
    Vec2 Rotated(float angleRadians) const {
        Vec2 result = *this;
        result.Rotate(angleRadians);
        return result;
    }

    // --- Equality Check ---
    bool operator==(const Vec2& other) const {
        return std::abs(x - other.x) < 1e-5f && std::abs(y - other.y) < 1e-5f;
    }
};

// --- Helper Functions for Vec2 ---
inline Vec2 operator*(float scalar, const Vec2& vec) { return vec * scalar; }
inline std::ostream& operator<<(std::ostream& os, const Vec2& vec) {
    os << "Vec2(" << vec.x << ", " << vec.y << ")";
    return os;
}