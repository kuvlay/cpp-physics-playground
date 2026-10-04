#include "physics.h"

#include <cmath>

namespace
{
constexpr float restitution = 1.0f;

void ResolveWalls(Body& body, const Arena& arena, float radius,
                  std::vector<Contact>* contacts)
{
    const Vec2 detectedPosition = body.position;
    const float minX = arena.left + radius;
    const float maxX = arena.right - radius;
    const float minY = arena.top + radius;
    const float maxY = arena.bottom - radius;

    if (body.position.x < minX)
    {
        if (contacts) contacts->push_back({{arena.left, detectedPosition.y}, {1, 0}});
        body.position.x = minX;
        if (body.velocity.x < 0.0f) body.velocity.x = -body.velocity.x;
    }
    else if (body.position.x > maxX)
    {
        if (contacts) contacts->push_back({{arena.right, detectedPosition.y}, {-1, 0}});
        body.position.x = maxX;
        if (body.velocity.x > 0.0f) body.velocity.x = -body.velocity.x;
    }

    if (body.position.y < minY)
    {
        if (contacts) contacts->push_back({{detectedPosition.x, arena.top}, {0, 1}});
        body.position.y = minY;
        if (body.velocity.y < 0.0f) body.velocity.y = -body.velocity.y;
    }
    else if (body.position.y > maxY)
    {
        if (contacts) contacts->push_back({{detectedPosition.x, arena.bottom}, {0, -1}});
        body.position.y = maxY;
        if (body.velocity.y > 0.0f) body.velocity.y = -body.velocity.y;
    }
}

void ResolvePair(Body& a, Body& b, float radius, std::vector<Contact>* contacts)
{
    const Vec2 delta = b.position - a.position;
    const float distanceSquared = LengthSquared(delta);
    const float diameter = 2.0f * radius;
    if (distanceSquared > diameter * diameter) return;

    const float distance = std::sqrt(distanceSquared);
    constexpr float minNormalDistance = 0.0001f;
    // Deterministic fallback: A (lower index) goes left, B goes right.
    const Vec2 normal = distance > minNormalDistance
        ? delta * (1.0f / distance) : Vec2{1.0f, 0.0f};
    if (contacts)
    {
        Vec2 point = a.position;
        point += normal * radius;
        contacts->push_back({point, normal});
    }
    const float penetration = diameter - distance;
    const Vec2 correction = normal * (penetration * 0.5f);
    a.position -= correction;
    b.position += correction;

    // Position repair is needed even if the bodies are already separating.
    const float vn = Dot(b.velocity - a.velocity, normal);
    if (vn >= 0.0f) return;

    // Equal unit masses: inverse masses sum to 2. No tangential impulse.
    const float impulse = -(1.0f + restitution) * vn / 2.0f;
    a.velocity -= normal * impulse;
    b.velocity += normal * impulse;
}
}

void UpdatePhysics(std::vector<Body>& bodies, const Arena& arena,
                   float radius, float dt, Vec2 acceleration,
                   std::vector<Contact>* contacts)
{
    if (contacts) contacts->clear();
    for (Body& body : bodies)
    {
        body.velocity += acceleration * dt;
        body.position += body.velocity * dt;
    }
    for (Body& body : bodies) ResolveWalls(body, arena, radius, contacts);
    for (std::size_t i = 0; i < bodies.size(); ++i)
        for (std::size_t j = i + 1; j < bodies.size(); ++j)
            ResolvePair(bodies[i], bodies[j], radius, contacts);
    for (Body& body : bodies) ResolveWalls(body, arena, radius, contacts);
    // One pass can leave residual overlaps in dense groups or near walls.
}
