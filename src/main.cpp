#include "raylib.h"

int main()
{
    // Create the native window and initialize the graphics context.
    InitWindow(1000, 700, "C++ Collision Playground");
    SetTargetFPS(60); // Let raylib pace the rendering loop toward 60 FPS.

    // By default, either the close button or Escape requests an exit.
    while (!WindowShouldClose())
    {
        BeginDrawing(); // Start drawing this frame.
        ClearBackground(Color{24, 28, 36, 255}); // Red, green, blue, alpha.

        // Positions and font sizes are in pixels; (0, 0) is the top left.
        DrawText("C++ Collision Playground", 40, 35, 32, RAYWHITE);
        DrawText("Stage 1: window and rendering", 40, 80, 20, LIGHTGRAY);
        DrawCircle(500, 350, 60.0f, SKYBLUE); // Center x, center y, radius, color.

        EndDrawing(); // Present the frame, pace it, and poll input events.
    }

    CloseWindow(); // Release graphics resources and destroy the window.
    return 0;
}

vector<vector<int>> distv;
