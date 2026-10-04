#include "raylib.h"
#include "physics.h"

#include <cmath>

int main()
{
    // Create the native window and initialize the graphics context.
    InitWindow(1000, 700, "C++ Collision Playground");
    SetTargetFPS(60); // Let raylib pace the rendering loop toward 60 FPS.

    const Arena arena{40.0f, 130.0f, 960.0f, 660.0f};
    constexpr float radius = 25.0f;
    std::vector<Body> bodies{
        // Blue and orange meet after about 1.27 seconds.
        {{180.0f, 270.0f}, {100.0f, 0.0f}},
        {{420.0f, 270.0f}, {-50.0f, 0.0f}},
        {{680.0f, 450.0f}, {90.0f, -120.0f}},
        {{820.0f, 550.0f}, {-180.0f, -60.0f}},
    };
    // Rendering data stays outside Body; colors follow the bodies' order.
    const Color colors[]{SKYBLUE, ORANGE, LIME, PINK};

    constexpr double fixedDt = 1.0 / 120.0; // Seconds per physics step.
    constexpr int maxStepsPerFrame = 8;
    double accumulator = 0.0;

    // By default, either the close button or Escape requests an exit.
    while (!WindowShouldClose())
    {
        accumulator += GetFrameTime(); // Elapsed seconds from raylib.
        int steps = 0;
        while (accumulator >= fixedDt && steps < maxStepsPerFrame)
        {
            UpdatePhysics(bodies, arena, radius, static_cast<float>(fixedDt));
            accumulator -= fixedDt;
            ++steps;
        }
        if (steps == maxStepsPerFrame)
        {
            // Drop overdue whole steps, but keep progress toward the next one.
            // Under overload the simulation loses time instead of catching up.
            accumulator = std::fmod(accumulator, fixedDt);
        }

        BeginDrawing(); // Start drawing this frame.
        ClearBackground(Color{24, 28, 36, 255}); // Red, green, blue, alpha.

        // Positions and font sizes are in pixels; (0, 0) is the top left.
        DrawText("C++ Collision Playground", 40, 35, 32, RAYWHITE);
        DrawText("Stage 3: circle collisions", 40, 80, 20, LIGHTGRAY);
        DrawRectangleLinesEx(
            Rectangle{arena.left, arena.top, arena.right - arena.left,
                      arena.bottom - arena.top},
            2.0f, LIGHTGRAY);
        for (std::size_t i = 0; i < bodies.size(); ++i)
        {
            const Vec2& position = bodies[i].position;
            DrawCircleV(Vector2{position.x, position.y}, radius, colors[i]);
        }

        EndDrawing(); // Present the frame, pace it, and poll input events.
    }

    CloseWindow(); // Release graphics resources and destroy the window.
    return 0;
}
