#pragma once

#include <vector>

// Positions are pixels; velocities are pixels per second.
struct Vec2
{
    float x;
    float y;

    Vec2& operator+=(Vec2 other)
    {
        x += other.x;
        y += other.y;
        return *this;
    }

    Vec2 operator*(float scalar) const
    {
        return {x * scalar, y * scalar};
    }

    Vec2 operator-(Vec2 other) const
    {
        return {x - other.x, y - other.y};
    }

    Vec2& operator-=(Vec2 other)
    {
        x -= other.x;
        y -= other.y;
        return *this;
    }
};

inline float Dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
inline float LengthSquared(Vec2 v) { return Dot(v, v); }

struct Body
{
    Vec2 position;
    Vec2 velocity;
};

struct Arena
{
    float left;
    float top;
    float right;
    float bottom;
};

// Sampled before positional correction. Pair normals point A -> B;
// wall normals point into the arena. No rendering types belong here.
struct Contact
{
    Vec2 position;
    Vec2 normal;
};

// Assumes a positive radius and an arena wider/taller than the diameter.
// Acceleration is pixels/second squared. Optional contacts replace the last step.
void UpdatePhysics(std::vector<Body>& bodies, const Arena& arena,
                   float radius, float dt, Vec2 acceleration,
                   std::vector<Contact>* contacts = nullptr);
