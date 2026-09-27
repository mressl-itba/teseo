/**
 * Teseo Micromouse Virtual Competition
 * UI module
 *
 * @brief Provides functions to create a raylib window, render the maze and mouse state,
 *        and handle user input (reset command).
 * @author Theseús the hero
 */

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "raylib.h"

#include "sim/mouse.h"
#include "sim/sim.h"

#include "ui.h"

// Layout constants

#define WINDOW_WIDTH 1280
#define WINDOW_HEIGHT 720

#define PIXELS_PER_METER (WINDOW_HEIGHT / MAZE_SIZE)
#define CELL_SIZE_PIXELS (CELL_SIZE * PIXELS_PER_METER)
#define MAZE_PX (CELL_SIZE_PIXELS * GRID_SIZE)
#define WALL_HALF_THICKNESS_PIXELS (WALL_THICKNESS * PIXELS_PER_METER / 2.0f)
#define WALL_THICKNESS_PIXELS (2 * WALL_HALF_THICKNESS_PIXELS)

#define MOUSE_WIDTH_PIXELS (MOUSE_WIDTH * PIXELS_PER_METER)
#define MOUSE_LENGTH_PIXELS (MOUSE_LENGTH * PIXELS_PER_METER)
#define MOUSE_HALF_WIDTH_PIXELS (MOUSE_HALF_WIDTH * PIXELS_PER_METER)
#define MOUSE_HALF_LENGTH_PIXELS (MOUSE_HALF_LENGTH * PIXELS_PER_METER)
#define MOUSE_ARROW_THICKNESS_PIXELS WALL_THICKNESS_PIXELS
#define ESTIMATE_THICKNESS_PIXELS WALL_HALF_THICKNESS_PIXELS

#define SENSOR_THICKNESS_PIXELS WALL_THICKNESS_PIXELS
#define SENSOR_HIT_RADIUS_PIXELS SENSOR_THICKNESS_PIXELS

#define FONT_SIZE_LARGE (4 * FONT_SIZE_SMALL)
#define FONT_SIZE_SMALL (10 * ((int)(WINDOW_HEIGHT / 360)))

#define PANEL_LEFT MAZE_PX
#define PANEL_WIDTH (WINDOW_WIDTH - MAZE_PX)
#define PANEL_PADDING 15
#define PANEL_SPACING 10
#define PANEL_TAB1 (6 * FONT_SIZE_SMALL)
#define PANEL_TAB2 (12 * FONT_SIZE_SMALL)
#define PANEL_BAR_WIDTH (PANEL_WIDTH - PANEL_TAB2 - PANEL_PADDING * 2)

// Colors

#define COLOR_BACKGROUND {0, 0, 0, 255}
#define COLOR_CHECKER_ALPHA 0.9f
#define COLOR_WALL {180, 192, 215, 255}
#define COLOR_GOAL {40, 160, 80, 60}
#define COLOR_GOAL_RING {60, 220, 100, 200}

#define COLOR_MOUSE {240, 80, 50, 255}
#define COLOR_SENSOR_HIT {255, 200, 60, 140}
#define COLOR_SENSOR_MISS {255, 200, 60, 70}
#define COLOR_MOUSE_ARROW {255, 200, 60, 255}
#define COLOR_ESTIMATE {60, 200, 255, 255}
#define COLOR_STATUS_BACKGROUND {15, 18, 28, 230}
#define COLOR_STATUS_TEXT {255, 90, 80, 255}

#define COLOR_PANEL_BACKGROUND {15, 18, 28, 255}
#define COLOR_TITLE {60, 160, 255, 255}
#define COLOR_TEXT COLOR_WALL
#define COLOR_MUTED {90, 100, 120, 255}
#define COLOR_BAR_GREEN {50, 130, 80, 255}
#define COLOR_BAR_YELLOW {180, 150, 40, 255}
#define COLOR_BAR_RED {148, 55, 65, 255}

// UI struct

static struct
{
    const Maze *maze;

    Sim *sim;
    float time_accumulator; // Real time not yet simulated (seconds)
} ui;

// Coordinate helpers

static inline Vector2 WorldToScreen(Vector2 position)
{
    return Vector2{position.x * PIXELS_PER_METER, WINDOW_HEIGHT - position.y * PIXELS_PER_METER};
}

static inline Vector2 CellToWorld(Cell cell)
{
    return Vector2(cell.x * CELL_SIZE, cell.y * CELL_SIZE);
}

static inline Vector2 CellToScreen(Cell cell)
{
    return WorldToScreen(CellToWorld(cell));
}

// Drawing helpers

static void DrawMaze()
{
    // Cell fills
    for (int32_t x = 0; x < GRID_SIZE; x++)
    {
        for (int32_t y = 0; y < GRID_SIZE; y++)
        {
            Vector2 top_left = CellToScreen({x, y + 1});
            Vector2 bottom_right = CellToScreen({x + 1, y});
            Vector2 size = Vector2Subtract(bottom_right, top_left);

            Color cell_color;
            if (IsGoalCell({x, y}))
                cell_color = COLOR_GOAL;
            else
            {
                uint32_t cell_hexvalue = (GetCellColor(ui.sim, {x, y}) << 8) | 0xff;
                cell_color = GetColor(cell_hexvalue);
            }

            // Subtle checker pattern
            if ((x + y) % 2 == 0)
                cell_color.a *= COLOR_CHECKER_ALPHA;

            DrawRectangleV(top_left, size, cell_color);
        }
    }

    // Goal ring
    {
        Vector2 top_left = CellToScreen({GRID_SIZE / 2 - 1, GRID_SIZE / 2 + 1});
        Vector2 bottom_right = CellToScreen({GRID_SIZE / 2 + 1, GRID_SIZE / 2 - 1});
        Vector2 size = Vector2Subtract(bottom_right, top_left);
        size.x += WALL_THICKNESS_PIXELS;
        size.y += WALL_THICKNESS_PIXELS;

        DrawRectangleLinesEx({top_left.x - WALL_HALF_THICKNESS_PIXELS, top_left.y - WALL_HALF_THICKNESS_PIXELS, size.x, size.y},
                             WALL_THICKNESS_PIXELS, COLOR_GOAL_RING);
    }

    // Walls
    for (int32_t x = 0; x < GRID_SIZE; x++)
    {
        for (int32_t y = 0; y < GRID_SIZE; y++)
        {
            Cell cell = {x, y};

            Vector2 top_left = CellToScreen({x, y + 1});
            Vector2 bottom_right = CellToScreen({x + 1, y});
            Vector2 size = Vector2Subtract(bottom_right, top_left);
            Vector2 size_horizontal = {size.x + WALL_THICKNESS_PIXELS, WALL_THICKNESS_PIXELS};
            Vector2 size_vertical = {WALL_THICKNESS_PIXELS, size.y + WALL_THICKNESS_PIXELS};

            if (HasWall(ui.maze, cell, WALL_NORTH))
                DrawRectangleV({top_left.x - WALL_HALF_THICKNESS_PIXELS, top_left.y - WALL_HALF_THICKNESS_PIXELS},
                               size_horizontal, COLOR_WALL);

            if (HasWall(ui.maze, cell, WALL_SOUTH))
                DrawRectangleV({top_left.x - WALL_HALF_THICKNESS_PIXELS, bottom_right.y - WALL_HALF_THICKNESS_PIXELS},
                               size_horizontal, COLOR_WALL);

            if (HasWall(ui.maze, cell, WALL_WEST))
                DrawRectangleV({top_left.x - WALL_HALF_THICKNESS_PIXELS, top_left.y - WALL_HALF_THICKNESS_PIXELS},
                               size_vertical, COLOR_WALL);

            if (HasWall(ui.maze, cell, WALL_EAST))
                DrawRectangleV({bottom_right.x - WALL_HALF_THICKNESS_PIXELS, top_left.y - WALL_HALF_THICKNESS_PIXELS},
                               size_vertical, COLOR_WALL);
        }
    }
}

static void DrawMouseSensors()
{
    const SimState *sim_state = GetSimState(ui.sim);

    for (uint32_t i = 0; i < IR_SENSOR_NUM; i++)
    {
        float sensor_distance = sim_state->ir_sensors[i];
        bool sensor_hit = sensor_distance < (IR_SENSOR_RANGE_MAX - 1E-6);

        Vector2 start = WorldToScreen(GetMousePosition(ui.sim));

        float angle = GetMouseRotation(ui.sim) + IR_SENSOR_ANGLES[i];
        Vector2 direction = {std::cos(-angle), std::sin(-angle)};
        Vector2 end = Vector2Add(start, Vector2Scale(direction, sensor_distance * PIXELS_PER_METER));

        if (sensor_hit)
        {
            DrawLineEx(start, end, SENSOR_THICKNESS_PIXELS, COLOR_SENSOR_HIT);
            DrawCircleV(end, SENSOR_HIT_RADIUS_PIXELS, COLOR_SENSOR_HIT);
        }
        else
            DrawLineEx(start, end, SENSOR_THICKNESS_PIXELS, COLOR_SENSOR_MISS);
    }
}

static void DrawMouse()
{
    Vector2 position = WorldToScreen(GetMousePosition(ui.sim));
    float rotation = GetMouseRotation(ui.sim);

    // Body
    DrawRectanglePro({position.x, position.y,
                      MOUSE_LENGTH_PIXELS, MOUSE_WIDTH_PIXELS},
                     {MOUSE_HALF_LENGTH_PIXELS, MOUSE_HALF_WIDTH_PIXELS},
                     -rotation * RAD2DEG,
                     COLOR_MOUSE);

    // Direction arrow
    Vector2 direction = {std::cos(-rotation), std::sin(-rotation)};
    Vector2 arrow_end = Vector2Add(position, Vector2Scale(direction, MOUSE_HALF_LENGTH_PIXELS));

    DrawLineEx(position, arrow_end, MOUSE_ARROW_THICKNESS_PIXELS, COLOR_MOUSE_ARROW);
}

static void DrawMouseEstimate()
{
    Vector2 estimated_position;
    float estimated_rotation;

    if (!GetEstimatedPose(ui.sim, &estimated_position, &estimated_rotation))
        return;

    // Body outline
    Vector2 forward = Vector2FromAngle(estimated_rotation, MOUSE_HALF_LENGTH);
    Vector2 left = Vector2FromAngle(estimated_rotation + TURN_CCW, MOUSE_HALF_WIDTH);

    Vector2 corners[4] = {
        Vector2Add(Vector2Add(estimated_position, forward), left),
        Vector2Subtract(Vector2Add(estimated_position, forward), left),
        Vector2Subtract(Vector2Subtract(estimated_position, forward), left),
        Vector2Add(Vector2Subtract(estimated_position, forward), left),
    };

    for (int i = 0; i < 4; i++)
        DrawLineEx(WorldToScreen(corners[i]), WorldToScreen(corners[(i + 1) % 4]),
                   ESTIMATE_THICKNESS_PIXELS, COLOR_ESTIMATE);

    // Direction arrow
    DrawLineEx(WorldToScreen(estimated_position), WorldToScreen(Vector2Add(estimated_position, forward)),
               ESTIMATE_THICKNESS_PIXELS, COLOR_ESTIMATE);
}

static void DrawStatus()
{
    const char *text = GetStatusText(ui.sim);
    if (!text[0])
        return;

    int width = MeasureText(text, FONT_SIZE_LARGE / 2);
    int x = (int)(MAZE_PX - width) / 2;
    int y = (WINDOW_HEIGHT - FONT_SIZE_LARGE / 2) / 2;

    DrawRectangle(x - PANEL_PADDING, y - PANEL_PADDING, width + 2 * PANEL_PADDING, FONT_SIZE_LARGE / 2 + 2 * PANEL_PADDING,
                  COLOR_STATUS_BACKGROUND);
    DrawText(text, x, y, FONT_SIZE_LARGE / 2, COLOR_STATUS_TEXT);
}

static void DrawPanelText(const char *label, float cx, float &cy, int size = FONT_SIZE_SMALL, Color color = COLOR_TEXT)
{
    DrawText(label, cx, cy, size, color);
    cy += size + PANEL_SPACING;
}

static void DrawPanelTab(const char *label, const char *value, float cx, float &cy)
{
    DrawText(label, cx, cy, FONT_SIZE_SMALL, COLOR_TEXT);
    DrawText(value, cx + PANEL_TAB1, cy, FONT_SIZE_SMALL, COLOR_TEXT);
    cy += FONT_SIZE_SMALL + PANEL_SPACING;
}

static void DrawPanelLine(float &cy)
{
    DrawLine(PANEL_LEFT, cy, PANEL_LEFT + PANEL_WIDTH, cy, COLOR_MUTED);
    cy += PANEL_SPACING;
}

static void DrawPanel()
{
    Vector2 position = {PANEL_LEFT + PANEL_PADDING, PANEL_PADDING};

    const SimState *state = GetSimState(ui.sim);
    Cell cell = PositionToCell(GetMousePosition(ui.sim));

    const char *sensor_labels[] = {"Left", "FwdLeft", "Forward", "FwdRight", "Right"};
    const char *run_state_labels[] = {"Idle", "Running", "Returning"};

    const char *run_state_label;

    if (state->time >= RUN_TIME_MAX)
        run_state_label = "Out of time";
    else
        run_state_label = run_state_labels[state->run_state];

    // Background

    DrawRectangle(PANEL_LEFT, 0, PANEL_WIDTH, WINDOW_HEIGHT, COLOR_PANEL_BACKGROUND);
    DrawRectangle(PANEL_LEFT, 0, 1, WINDOW_HEIGHT, COLOR_MUTED);

    // Title

    DrawPanelText("TESEO", position.x, position.y, FONT_SIZE_LARGE, COLOR_TITLE);
    DrawPanelText("Micromouse Virtual Competition", position.x, position.y, FONT_SIZE_SMALL, COLOR_MUTED);
    DrawPanelText("", position.x, position.y);

    // Mouse name

    DrawPanelText(GetMouseName(), position.x, position.y);

    DrawPanelLine(position.y);

    // Stats

    DrawPanelText("STATS", position.x, position.y, FONT_SIZE_SMALL, COLOR_MUTED);
    DrawPanelTab("Run", TextFormat("%d", state->run_number), position.x, position.y);
    DrawPanelTab("Run state", run_state_label, position.x, position.y);
    DrawPanelTab("Time", TextFormat("%.3f s", state->time), position.x, position.y);
    DrawPanelTab("Run time", TextFormat("%.3f s", state->run_time), position.x, position.y);
    if (state->run_time_best > 0.0f)
        DrawPanelTab("Best time", TextFormat("%.3f s", state->run_time_best), position.x, position.y);
    else
        DrawPanelTab("Best time", "-", position.x, position.y);

    Vector2 estimated_position;
    float estimated_rotation;
    if (GetEstimatedPose(ui.sim, &estimated_position, &estimated_rotation))
    {
        float position_error = Vector2Distance(estimated_position, GetMousePosition(ui.sim));
        float rotation_error = AngleDiff(GetMouseRotation(ui.sim), estimated_rotation);

        DrawPanelTab("Est. error", TextFormat("%.1f cm  %+.1f°", position_error * 100.0f, rotation_error * RAD2DEG),
                     position.x, position.y);
    }
    else
        DrawPanelTab("Est. error", "-", position.x, position.y);

    DrawPanelLine(position.y);

    // Sensors

    DrawPanelText("SENSORS", position.x, position.y, FONT_SIZE_SMALL, COLOR_MUTED);

    for (uint32_t i = 0; i < IR_SENSOR_NUM; i++)
    {
        float sensor_distance = state->ir_sensors[i];
        float sensor_ratio = (sensor_distance - IR_SENSOR_RANGE_MIN) / (IR_SENSOR_RANGE_MAX - IR_SENSOR_RANGE_MIN);
        float sensor_pixels = sensor_ratio * PANEL_BAR_WIDTH;

        Color color;
        if (sensor_distance < CELL_HALF_SIZE)
            color = ColorLerp(COLOR_BAR_RED, COLOR_BAR_YELLOW,
                              (sensor_distance - IR_SENSOR_RANGE_MIN) / (CELL_HALF_SIZE - IR_SENSOR_RANGE_MIN));
        else
            color = ColorLerp(COLOR_BAR_YELLOW, COLOR_BAR_GREEN,
                              (sensor_distance - CELL_HALF_SIZE) / (IR_SENSOR_RANGE_MAX - CELL_HALF_SIZE));

        DrawRectangleLines(position.x + PANEL_TAB2 + 1, position.y, PANEL_BAR_WIDTH - 1, FONT_SIZE_SMALL - 1, COLOR_MUTED);
        DrawRectangle(position.x + PANEL_TAB2, position.y, sensor_pixels, FONT_SIZE_SMALL, color);

        DrawPanelTab(sensor_labels[i], TextFormat("%5.3f m", sensor_distance), position.x, position.y);
    }

    DrawPanelLine(position.y);

    // Controls

    DrawPanelText("CONTROLS", position.x, position.y, FONT_SIZE_SMALL, COLOR_MUTED);
    DrawPanelText("[R] Run / Reset", position.x, position.y);
    DrawPanelText("[F11] Toggle fullscreen", position.x, position.y);
}

// Public API

void CreateUI(const Maze *maze)
{
    ui.maze = maze;

    ui.sim = CreateSim(maze);

    // Monitor queries need an initialized window, so the target FPS is set afterwards.
    InitWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Teseo — Micromouse Virtual Competition");
    SetTargetFPS(GetMonitorRefreshRate(GetCurrentMonitor()));
}

void DestroyUI()
{
    DestroySim(ui.sim);

    CloseWindow();
}

bool UpdateUI()
{
    if (WindowShouldClose())
        return false;

    // Advance simulation in fixed steps, as many as fit in the elapsed real time
    ui.time_accumulator += std::min(GetFrameTime(), 0.1f);

    while (ui.time_accumulator >= SIM_TIMESTEP)
    {
        if (IsSimRunning(ui.sim))
        {
            UpdateMouse(ui.sim);
            UpdateSim(ui.sim);
        }

        ui.time_accumulator -= SIM_TIMESTEP;
    }

    // Run / reset
    if (IsKeyPressed(KEY_R))
    {
        if (ResetSim(ui.sim))
            ResetMouse(ui.sim);
    }

    // Toggle fullscreen
    if (IsKeyPressed(KEY_F11))
        ToggleFullscreen();

    // Render
    BeginDrawing();

    ClearBackground(COLOR_BACKGROUND);
    DrawMaze();
    DrawMouseSensors();
    DrawMouse();
    DrawMouseEstimate();
    DrawStatus();
    DrawPanel();

    EndDrawing();

    return true;
}
