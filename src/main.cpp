#include "raylib.h"
#include "physics.h"

#include <algorithm>
#include <cmath>
#include <iterator>

namespace
{
constexpr float radius = 25.0f;
constexpr std::size_t maxBodies = 80;
constexpr Vec2 gravityAcceleration{0.0f, 300.0f}; // Pixels/second squared.
constexpr float velocityArrowScale = 0.2f; // Seconds: velocity -> drawn pixels.
constexpr float contactArrowLength = 30.0f;
constexpr float feedbackDuration = 2.0f;

std::vector<Body> MakeInitialScene()
{
    return {
        {{180.0f, 270.0f}, {100.0f, 0.0f}},
        {{420.0f, 270.0f}, {-50.0f, 0.0f}},
        {{680.0f, 450.0f}, {90.0f, -120.0f}},
        {{820.0f, 550.0f}, {-180.0f, -60.0f}},
    };
}

// A null result means success; otherwise return a short rejection message.
const char* TrySpawn(std::vector<Body>& bodies, Vec2 position,
                     const Arena& arena, std::size_t& spawnIndex)
{
    if (bodies.size() >= maxBodies) return "Spawn rejected: 80-circle limit.";
    if (position.x < arena.left + radius || position.x > arena.right - radius ||
        position.y < arena.top + radius || position.y > arena.bottom - radius)
        return "Spawn rejected: the whole circle must fit inside the arena.";
    for (const Body& body : bodies)
        if (LengthSquared(position - body.position) < 4.0f * radius * radius)
            return "Spawn rejected: overlaps an existing circle.";

    constexpr Vec2 velocities[]{{80, -60}, {-70, 50}, {60, 80}, {-90, -40}};
    bodies.push_back({position, velocities[spawnIndex % std::size(velocities)]});
    ++spawnIndex;
    return nullptr;
}

void DrawArrow(Vec2 origin, Vec2 displacement, Color color)
{
    const float length = std::sqrt(LengthSquared(displacement));
    if (length < 0.001f) return;
    const Vec2 direction = displacement * (1.0f / length);
    Vec2 tip = origin;
    tip += displacement;
    const float headLength = std::min(7.0f, length * 0.4f);
    Vec2 base = tip;
    base -= direction * headLength;
    const Vec2 side{-direction.y * headLength * 0.5f,
                     direction.x * headLength * 0.5f};
    DrawLineEx({origin.x, origin.y}, {tip.x, tip.y}, 2.0f, color);
    DrawLineEx({tip.x, tip.y}, {base.x + side.x, base.y + side.y}, 2.0f, color);
    DrawLineEx({tip.x, tip.y}, {base.x - side.x, base.y - side.y}, 2.0f, color);
}
}

int main()
{
    InitWindow(1000, 700, "C++ Collision Playground");
    SetTargetFPS(60);

    const Arena arena{40.0f, 130.0f, 960.0f, 660.0f};
    std::vector<Body> bodies = MakeInitialScene();
    const Color colors[]{SKYBLUE, ORANGE, LIME, PINK};
    constexpr double fixedDt = 1.0 / 120.0;
    constexpr int maxStepsPerFrame = 8;
    double accumulator = 0.0;
    bool paused = false;
    bool gravityEnabled = false;
    bool debugEnabled = false;
    std::size_t spawnIndex = 0;
    std::vector<Contact> contacts;
    const char* feedback = nullptr;
    float feedbackRemaining = 0.0f;

    while (!WindowShouldClose())
    {
        const float frameTime = GetFrameTime();
        feedbackRemaining = std::max(0.0f, feedbackRemaining - frameTime);
        bool skipSimulation = false;

        // Reset wins over other input on this frame, so its scene is drawn intact.
        if (IsKeyPressed(KEY_R))
        {
            bodies = MakeInitialScene();
            accumulator = 0.0;
            contacts.clear();
            spawnIndex = 0;
            feedback = nullptr;
            feedbackRemaining = 0.0f;
            gravityEnabled = false;
            debugEnabled = false;
            paused = false;
            skipSimulation = true;
        }
        else
        {
            if (IsKeyPressed(KEY_P))
            {
                paused = !paused;
                accumulator = 0.0;
                skipSimulation = true; // Neither transition consumes this frame's time.
            }
            if (IsKeyPressed(KEY_G)) gravityEnabled = !gravityEnabled;
            if (IsKeyPressed(KEY_C))
            {
                debugEnabled = !debugEnabled;
                contacts.clear();
            }
            if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT))
            {
                const Vector2 mouse = GetMousePosition();
                feedback = TrySpawn(bodies, {mouse.x, mouse.y}, arena, spawnIndex);
                feedbackRemaining = feedback ? feedbackDuration : 0.0f;
            }
        }

        if (!paused && !skipSimulation)
        {
            accumulator += frameTime;
            int steps = 0;
            const Vec2 acceleration = gravityEnabled ? gravityAcceleration : Vec2{0, 0};
            while (accumulator >= fixedDt && steps < maxStepsPerFrame)
            {
                UpdatePhysics(bodies, arena, radius, static_cast<float>(fixedDt),
                              acceleration, debugEnabled ? &contacts : nullptr);
                accumulator -= fixedDt;
                ++steps;
            }
            if (steps == maxStepsPerFrame)
            {
                // Drop overdue whole steps, retaining only the fractional remainder.
                accumulator = std::fmod(accumulator, fixedDt);
            }
        }

        BeginDrawing();
        ClearBackground(Color{24, 28, 36, 255});
        DrawText("C++ Collision Playground", 40, 16, 30, RAYWHITE);
        DrawText("Stage 4 | Click: spawn   P: pause   R: reset   G: gravity   C: debug   Esc: exit",
                 40, 54, 18, LIGHTGRAY);
        DrawText(TextFormat("%s   |   Gravity: %s   |   Debug: %s   |   Circles: %i / %i",
                            paused ? "PAUSED" : "RUNNING", gravityEnabled ? "ON" : "OFF",
                            debugEnabled ? "ON" : "OFF", static_cast<int>(bodies.size()),
                            static_cast<int>(maxBodies)), 40, 80, 18, RAYWHITE);
        if (feedback && feedbackRemaining > 0.0f)
            DrawText(feedback, 40, 106, 16, ORANGE);
        DrawRectangleLinesEx(
            Rectangle{arena.left, arena.top, arena.right - arena.left,
                      arena.bottom - arena.top}, 2.0f, LIGHTGRAY);
        for (std::size_t i = 0; i < bodies.size(); ++i)
        {
            const Vec2& position = bodies[i].position;
            DrawCircleV({position.x, position.y}, radius, colors[i % std::size(colors)]);
        }
        if (debugEnabled)
        {
            BeginScissorMode(static_cast<int>(arena.left), static_cast<int>(arena.top),
                             static_cast<int>(arena.right - arena.left),
                             static_cast<int>(arena.bottom - arena.top));
            for (const Body& body : bodies)
                DrawArrow(body.position, body.velocity * velocityArrowScale, RAYWHITE);
            for (const Contact& contact : contacts)
            {
                DrawCircleV({contact.position.x, contact.position.y}, 3.0f, YELLOW);
                DrawArrow(contact.position, contact.normal * contactArrowLength, YELLOW);
            }
            EndScissorMode();
            DrawText("White: velocity   Yellow: sampled normals from the last physics step",
                     40, 675, 16, LIGHTGRAY);
        }
        EndDrawing();
    }

    CloseWindow();
    return 0;
}
