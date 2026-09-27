/**
 * Teseo Micromouse Virtual Competition
 * Keyboard mouse
 *
 * @details Implements a keyboard-controlled mouse. Drive it with the WASD keys.
 * @author Theseús the hero
 */

#include "raylib.h"

#include "sim/mouse.h"

#define LINEAR_VELOCITY 0.5f        // m/s
#define ANGULAR_VELOCITY (1.0f * PI) // rad/s

const char *GetMouseName()
{
    return "Keyboard Mouse [WASD]";
}

void ResetMouse(Sim *sim)
{
}

void UpdateMouse(Sim *sim)
{
    float linear = 0.0f;
    float angular = 0.0f;

    if (IsKeyDown(KEY_W))
        linear += LINEAR_VELOCITY;
    if (IsKeyDown(KEY_S))
        linear -= LINEAR_VELOCITY;
    if (IsKeyDown(KEY_A))
        angular += ANGULAR_VELOCITY;
    if (IsKeyDown(KEY_D))
        angular -= ANGULAR_VELOCITY;

    SetMouseVelocity(sim, linear, angular);
}
