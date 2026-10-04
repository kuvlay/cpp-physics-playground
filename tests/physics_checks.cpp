#include "physics.h"

#include <cassert>
#include <cmath>
#include <iostream>

#ifdef NDEBUG
#error Physics checks require assertions; compile without NDEBUG.
#endif

namespace
{
constexpr float dt = 1.0f / 120.0f;
const Arena arena{40.0f, 130.0f, 960.0f, 660.0f};
constexpr float radius = 25.0f;

bool Near(float a, float b) { return std::fabs(a - b) < 0.001f; }

void CheckVector(Vec2 actual, Vec2 expected)
{
    assert(Near(actual.x, expected.x));
    assert(Near(actual.y, expected.y));
}

void CheckAccelerationAndContacts()
{
    std::vector<Body> bodies{{{300, 300}, {120, 60}}};
    UpdatePhysics(bodies, arena, radius, dt, {0, 0});
    CheckVector(bodies[0].velocity, {120, 60});
    CheckVector(bodies[0].position, {301, 300.5f});

    bodies = {{{300, 300}, {0, 0}}};
    UpdatePhysics(bodies, arena, radius, dt, {0, 300});
    CheckVector(bodies[0].velocity, {0, 2.5f});
    CheckVector(bodies[0].position, {300, 300 + 2.5f * dt});
    UpdatePhysics(bodies, arena, radius, dt, {0, 0});
    CheckVector(bodies[0].velocity, {0, 2.5f});
    CheckVector(bodies[0].position, {300, 300 + 5.0f * dt});

    bodies = {{{300, 300}, {100, 0}}, {{340, 300}, {-50, 0}}};
    auto withoutDebug = bodies;
    std::vector<Contact> contacts;
    UpdatePhysics(bodies, arena, radius, 0, {0, 0}, &contacts);
    UpdatePhysics(withoutDebug, arena, radius, 0, {0, 0});
    assert(contacts.size() == 1);
    CheckVector(contacts[0].position, {325, 300});
    CheckVector(contacts[0].normal, {1, 0});
    for (std::size_t i = 0; i < bodies.size(); ++i)
    {
        CheckVector(bodies[i].position, withoutDebug[i].position);
        CheckVector(bodies[i].velocity, withoutDebug[i].velocity);
    }
    bodies = {{{300, 300}, {0, 0}}};
    UpdatePhysics(bodies, arena, radius, dt, {0, 0}, &contacts);
    assert(contacts.empty());

    bodies = {{{60, 150}, {-100, -100}}};
    UpdatePhysics(bodies, arena, radius, 0, {0, 0}, &contacts);
    assert(contacts.size() == 2);
    CheckVector(contacts[0].position, {40, 150});
    CheckVector(contacts[0].normal, {1, 0});
    CheckVector(contacts[1].position, {60, 130});
    CheckVector(contacts[1].normal, {0, 1});
}

void CheckCircleCollisions()
{
    // dt=0 isolates contact response from movement; all pairs are away from walls.
    std::vector<Body> bodies{{{300, 300}, {100, 20}}, {{400, 300}, {-50, -30}}};
    UpdatePhysics(bodies, arena, radius, 0.0f, {0, 0});
    CheckVector(bodies[0].velocity, {100, 20});
    CheckVector(bodies[1].velocity, {-50, -30});
    CheckVector(bodies[0].position, {300, 300});
    CheckVector(bodies[1].position, {400, 300});

    bodies = {{{300, 300}, {100, 0}}, {{350, 300}, {-50, 0}}};
    UpdatePhysics(bodies, arena, radius, 0.0f, {0, 0});
    CheckVector(bodies[0].velocity, {-50, 0});
    CheckVector(bodies[1].velocity, {100, 0});

    // Oblique contact: normal=(0.6,0.8), tangent=(-0.8,0.6).
    bodies = {{{300, 300}, {100, 20}}, {{330, 340}, {-50, -30}}};
    const auto before = bodies;
    const Vec2 tangent{-0.8f, 0.6f};
    const float energyBefore = 0.5f * (LengthSquared(before[0].velocity)
                                      + LengthSquared(before[1].velocity));
    UpdatePhysics(bodies, arena, radius, 0.0f, {0, 0});
    CheckVector(bodies[0].velocity, {22, -84});
    CheckVector(bodies[1].velocity, {28, 74});
    for (std::size_t i = 0; i < bodies.size(); ++i)
        assert(Near(Dot(bodies[i].velocity, tangent), Dot(before[i].velocity, tangent)));
    assert(Near(bodies[0].velocity.x + bodies[1].velocity.x,
                before[0].velocity.x + before[1].velocity.x));
    assert(Near(bodies[0].velocity.y + bodies[1].velocity.y,
                before[0].velocity.y + before[1].velocity.y));
    const float energyAfter = 0.5f * (LengthSquared(bodies[0].velocity)
                                     + LengthSquared(bodies[1].velocity));
    assert(std::fabs(energyAfter - energyBefore) <= 0.00001f * energyBefore);

    bodies = {{{300, 300}, {-50, 10}}, {{340, 300}, {100, 10}}};
    UpdatePhysics(bodies, arena, radius, 0.0f, {0, 0});
    CheckVector(bodies[0].position, {295, 300});
    CheckVector(bodies[1].position, {345, 300});
    CheckVector(bodies[0].velocity, {-50, 10});
    CheckVector(bodies[1].velocity, {100, 10});

    // Zero and tiny reversed offsets exercise the deterministic fallback.
    for (float offset : {0.0f, -0.00003f})
    {
        bodies = {{{300, 300}, {100, 0}}, {{300 + offset, 300}, {-50, 0}}};
        UpdatePhysics(bodies, arena, radius, 0.0f, {0, 0});
        for (const Body& body : bodies)
        {
            assert(std::isfinite(body.position.x) && std::isfinite(body.position.y));
            assert(std::isfinite(body.velocity.x) && std::isfinite(body.velocity.y));
        }
        assert(bodies[0].position.x < bodies[1].position.x);
        assert(LengthSquared(bodies[1].position - bodies[0].position) >= 2500.0f - 0.01f);
        CheckVector(bodies[0].velocity, {-50, 0});
        CheckVector(bodies[1].velocity, {100, 0});
    }

    bodies = {{{65, 300}, {0, 0}}, {{100, 300}, {0, 0}}};
    UpdatePhysics(bodies, arena, radius, 0.0f, {0, 0});
    assert(bodies[0].position.x == 65.0f);

    bodies = {{{180, 270}, {100, 0}}, {{420, 270}, {-50, 0}}};
    for (int step = 0; step < 180; ++step)
        UpdatePhysics(bodies, arena, radius, dt, {0, 0});
    CheckVector(bodies[0].velocity, {-50, 0});
    CheckVector(bodies[1].velocity, {100, 0});
}

void CheckBounce(Body start, Vec2 position, Vec2 velocity)
{
    std::vector<Body> bodies{start};
    UpdatePhysics(bodies, arena, radius, dt, {0, 0});
    const Body& result = bodies[0];
    assert(Near(result.position.x, position.x));
    assert(Near(result.position.y, position.y));
    assert(result.velocity.x == velocity.x);
    assert(result.velocity.y == velocity.y);
    assert(Near(std::hypot(start.velocity.x, start.velocity.y),
                std::hypot(result.velocity.x, result.velocity.y)));
}
}

void CheckCrowdedSimulation()
{
    // One simulated minute at the UI cap, with gravity both off and on.
    for (Vec2 acceleration : {Vec2{0, 0}, Vec2{0, 300}})
    {
        std::vector<Body> bodies;
        for (int i = 0; i < 80; ++i)
            bodies.push_back({{90.0f + (i % 16) * 55.0f,
                               190.0f + (i / 16) * 80.0f},
                              {i % 2 ? 80.0f : -70.0f, i % 3 ? 50.0f : -60.0f}});
        auto withoutDebug = bodies;
        std::vector<Contact> contacts;
        for (int step = 0; step < 7200; ++step)
        {
            UpdatePhysics(bodies, arena, radius, dt, acceleration, &contacts);
            UpdatePhysics(withoutDebug, arena, radius, dt, acceleration);
            for (std::size_t i = 0; i < bodies.size(); ++i)
            {
                const Body& body = bodies[i];
                assert(std::isfinite(body.position.x) && std::isfinite(body.position.y));
                assert(std::isfinite(body.velocity.x) && std::isfinite(body.velocity.y));
                assert(body.position.x >= 65 && body.position.x <= 935);
                assert(body.position.y >= 155 && body.position.y <= 635);
                CheckVector(body.position, withoutDebug[i].position);
                CheckVector(body.velocity, withoutDebug[i].velocity);
            }
            for (const Contact& contact : contacts)
            {
                assert(std::isfinite(contact.position.x) && std::isfinite(contact.position.y));
                assert(Near(LengthSquared(contact.normal), 1.0f));
            }
        }
    }
}

int main()
{
    CheckCrowdedSimulation();
    CheckAccelerationAndContacts();
    CheckCircleCollisions();
    CheckBounce({{180, 220}, {120, 60}}, {181, 220.5f}, {120, 60});
    CheckBounce({{66, 300}, {-240, 60}}, {65, 300.5f}, {240, 60});
    CheckBounce({{934, 300}, {240, 60}}, {935, 300.5f}, {-240, 60});
    CheckBounce({{300, 156}, {60, -240}}, {300.5f, 155}, {60, 240});
    CheckBounce({{300, 634}, {60, 240}}, {300.5f, 635}, {60, -240});

    for (int sx : {-1, 1})
        for (int sy : {-1, 1})
            CheckBounce({{sx < 0 ? 66.0f : 934.0f, sy < 0 ? 156.0f : 634.0f},
                         {sx * 240.0f, sy * 240.0f}},
                        {sx < 0 ? 65.0f : 935.0f, sy < 0 ? 155.0f : 635.0f},
                        {-sx * 240.0f, -sy * 240.0f});

    CheckBounce({{60, 300}, {120, 0}}, {65, 300}, {120, 0});
    CheckBounce({{940, 300}, {-120, 0}}, {935, 300}, {-120, 0});
    CheckBounce({{300, 150}, {0, 120}}, {300, 155}, {0, 120});
    CheckBounce({{300, 640}, {0, -120}}, {300, 635}, {0, -120});

    std::vector<Body> a{{{180, 220}, {120, 60}}, {{820, 550}, {-180, -60}}};
    auto b = a;
    for (int step = 0; step < 1200; ++step)
    {
        UpdatePhysics(a, arena, radius, dt, {0, 0});
        for (const Body& body : a)
        {
            assert(body.position.x >= 65 && body.position.x <= 935);
            assert(body.position.y >= 155 && body.position.y <= 635);
        }
    }
    // Same 1200 steps grouped as two per hypothetical rendering frame.
    for (int frame = 0; frame < 600; ++frame)
        for (int step = 0; step < 2; ++step)
            UpdatePhysics(b, arena, radius, dt, {0, 0});
    for (std::size_t i = 0; i < a.size(); ++i)
    {
        assert(a[i].position.x == b[i].position.x);
        assert(a[i].position.y == b[i].position.y);
        assert(a[i].velocity.x == b[i].velocity.x);
        assert(a[i].velocity.y == b[i].velocity.y);
    }
    std::cout << "Stage 4 physics checks passed.\n";
}
