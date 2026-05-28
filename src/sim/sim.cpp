/**
 * Teseo Micromouse Virtual Competition
 * Physics simulation module
 *
 * @brief Implements the physics simulation using Box2D.
 * @author Theseús the hero
 */

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>

#include <box2d/box2d.h>

#include "sim.h"

// Sensor angles

float IR_SENSOR_ANGLES[5] = {
    TURN_CCW,        // Left       (90° left)
    TURN_CCW / 2.0f, // Front-left (45° left)
    0,               // Front
    TURN_CW / 2.0f,  // Front-right (45° right)
    TURN_CW,         // Right      (90° right)
};

// Sim state

struct Sim
{
    // Maze
    const Maze *maze;
    uint32_t maze_colors[GRID_SIZE][GRID_SIZE];

    // Box2D world and bodies
    b2WorldId world;
    b2BodyId mouse_body;

    // Sim state
    Vector2 mouse_position;      // World position (meters, north/east coordinates)
    float mouse_rotation;        // Rotation (radians, CCW+, 0 = East)
    Vector2 mouse_velocity_last; // World-frame velocity, kept across steps for acceleration computation

    SimState state;

    // Mouse controller
    Vector2 reference_position;    // Reference position when setpoint was issued
    float reference_rotation;      // Reference rotation when setpoint was issued
    float target_distance;         // Target distance
    float target_rotation;         // Target rotation
    float odometry_distance_error; // Odometric error distance
    float odometry_rotation_error; // Odometric error rotation
};

// Math helpers

Vector2 Vector2FromAngle(float angle, float length)
{
    return Vector2{cosf(angle) * length, sinf(angle) * length};
}

float AngleDiff(float source, float target)
{
    float diff = target - source;

    diff = atan2f(sinf(diff), cosf(diff));

    return diff;
}

// Maze

Cell PositionToCell(Vector2 position)
{
    return {
        (int32_t)floorf(position.x / CELL_SIZE),
        (int32_t)floorf(position.y / CELL_SIZE)};
}

void PaintCell(Sim *sim, Cell cell, uint32_t color)
{
    if (!ValidateCell(cell))
        return;

    sim->maze_colors[cell.x][cell.y] = color;
}

uint32_t GetCellColor(Sim *sim, Cell cell)
{
    if (!ValidateCell(cell))
        return COLOR_CELL_DEFAULT;

    return sim->maze_colors[cell.x][cell.y];
}

void ResetCellColors(Sim *sim)
{
    for (int x = 0; x < GRID_SIZE; x++)
        for (int y = 0; y < GRID_SIZE; y++)
            PaintCell(sim, {x, y}, COLOR_CELL_DEFAULT);
}

static void CreateMazePhysics(Sim *sim)
{
    // One static body at the origin.
    b2BodyDef body_def = b2DefaultBodyDef();
    body_def.type = b2_staticBody;
    b2BodyId walls_body = b2CreateBody(sim->world, &body_def);

    b2ShapeDef shape_def = b2DefaultShapeDef();
    shape_def.material.restitution = 0.05f; // Nearly inelastic (foam-tipped ABS walls)
    shape_def.material.friction = 0.4f;

    // Horizontal wall segments
    for (int32_t y = 0; y <= GRID_SIZE; y++)
    {
        for (int32_t x = 0; x < GRID_SIZE; x++)
        {
            bool has_wall;
            if (y == 0)
                has_wall = HasWall(sim->maze, {x, y}, WALL_SOUTH);
            else
                has_wall = HasWall(sim->maze, {x, y - 1}, WALL_NORTH);

            if (has_wall)
            {
                Vector2 center = {(x + 0.5f) * CELL_SIZE, y * CELL_SIZE};

                b2Polygon box = b2MakeOffsetBox(WALL_HALF_WIDTH, WALL_HALF_THICKNESS,
                                                b2Vec2(center.x, center.y), b2MakeRot(0.0f));
                b2CreatePolygonShape(walls_body, &shape_def, &box);
            }
        }
    }

    // Vertical wall segments
    for (int32_t x = 0; x <= GRID_SIZE; x++)
    {
        for (int32_t y = 0; y < GRID_SIZE; y++)
        {
            bool has_wall;
            if (x == 0)
                has_wall = HasWall(sim->maze, {x, y}, WALL_WEST);
            else
                has_wall = HasWall(sim->maze, {x - 1, y}, WALL_EAST);

            if (has_wall)
            {
                Vector2 center = {x * CELL_SIZE, (y + 0.5f) * CELL_SIZE};

                b2Polygon box = b2MakeOffsetBox(WALL_HALF_THICKNESS, WALL_HALF_HEIGHT,
                                                b2Vec2(center.x, center.y), b2MakeRot(0.0f));
                b2CreatePolygonShape(walls_body, &shape_def, &box);
            }
        }
    }
}

// Mouse

static void CreateMousePhysics(Sim *sim)
{
    b2BodyDef body_def = b2DefaultBodyDef();
    body_def.type = b2_dynamicBody;
    body_def.position = b2Vec2(0.5f * CELL_SIZE, 0.5f * CELL_SIZE); // Start in center of cell (0,0)
    body_def.rotation = b2MakeRot(ROTATION_NORTH);
    body_def.linearDamping = 0.0f;
    body_def.angularDamping = 0.0f;
    body_def.fixedRotation = false;
    sim->mouse_body = b2CreateBody(sim->world, &body_def);

    b2ShapeDef shape_def = b2DefaultShapeDef();
    shape_def.density = MOUSE_DENSITY;
    shape_def.material.friction = 0.2f;
    shape_def.material.restitution = 0.05f;

    b2Polygon box = b2MakeBox(MOUSE_HALF_LENGTH, MOUSE_HALF_WIDTH);
    b2CreatePolygonShape(sim->mouse_body, &shape_def, &box);
}

static void UpdateIMU(Sim *sim, float dt)
{
    // Get velocity
    b2Vec2 b2_velocity = b2Body_GetLinearVelocity(sim->mouse_body);
    Vector2 velocity = {b2_velocity.x, b2_velocity.y};

    // Transform velocity to body frame
    Vector2 mouse_velocity = Vector2Rotate(velocity, TURN_CCW - sim->mouse_rotation);

    // Compute delta-v
    Vector2 delta_v = Vector2Subtract(mouse_velocity, sim->mouse_velocity_last);
    sim->mouse_velocity_last = mouse_velocity;

    // Compute acceleration
    sim->state.accelerometer = Vector2Scale(delta_v, 1.0f / dt);
}

static void UpdateMouseState(Sim *sim)
{
    b2Vec2 position = b2Body_GetPosition(sim->mouse_body);
    float rotation = b2Rot_GetAngle(b2Body_GetRotation(sim->mouse_body));
    float angular_velocity = b2Body_GetAngularVelocity(sim->mouse_body);

    sim->mouse_position = {position.x, position.y};
    sim->mouse_rotation = rotation;
    sim->state.gyroscope = angular_velocity;

    b2QueryFilter filter = b2DefaultQueryFilter();

    for (int i = 0; i < IR_SENSOR_NUM; i++)
    {
        float ray_angle = rotation + IR_SENSOR_ANGLES[i];
        Vector2 direction = Vector2FromAngle(ray_angle, IR_SENSOR_RANGE_MAX);

        b2Vec2 start = b2Vec2{position.x, position.y};
        b2Vec2 translation = b2Vec2{direction.x, direction.y};

        b2RayResult rayResult = b2World_CastRayClosest(sim->world, start, translation, filter);

        if (rayResult.hit)
            sim->state.ir_sensors[i] = rayResult.fraction * IR_SENSOR_RANGE_MAX;
        else
            sim->state.ir_sensors[i] = IR_SENSOR_RANGE_MAX;
    }
}

static void ResetMouseController(Sim *sim, Vector2 position, float rotation)
{
    sim->reference_position = position;
    sim->reference_rotation = rotation;

    sim->target_distance = 0.0f;
    sim->odometry_distance_error = 1.0f;

    sim->target_rotation = rotation;
    sim->odometry_rotation_error = 1.0f;

    sim->state.setpoint_distance = 0.0f;
    sim->state.setpoint_rotation = 0.0f;
}

static void ResetMousePhysics(Sim *sim)
{
    // Reset mouse state
    Vector2 position = {0.5f * CELL_SIZE, 0.5f * CELL_SIZE};
    float rotation = ROTATION_NORTH;

    b2Body_SetTransform(sim->mouse_body, b2Vec2(position.x, position.y), b2MakeRot(rotation));
    b2Body_SetLinearVelocity(sim->mouse_body, b2Vec2(0.0f, 0.0f));
    b2Body_SetAngularVelocity(sim->mouse_body, 0.0f);

    UpdateMouseState(sim);
    sim->mouse_velocity_last = {0.0f, 0.0f};
    sim->state.accelerometer = {0.0f, 0.0f};

    // Reset controller
    ResetMouseController(sim, position, rotation);
}

static bool StartRun(Sim *sim)
{
    if (sim->state.time >= RUN_TIME_MAX)
        return false;

    if (sim->state.run_number >= RUN_TOTAL)
        return false;

    sim->state.run_number += 1;
    sim->state.run_state = RUNSTATE_IDLE;
    sim->state.run_time = 0.0f;

    return true;
}

// Controller

static void ApplyDrive(Sim *sim, float left_wheel_target_velocity, float right_wheel_target_velocity)
{
    // Get current velocity and rotation
    b2Vec2 b2_velocity = b2Body_GetLinearVelocity(sim->mouse_body);
    Vector2 velocity = {b2_velocity.x, b2_velocity.y};
    float rotation = b2Rot_GetAngle(b2Body_GetRotation(sim->mouse_body));
    float angular_velocity = b2Body_GetAngularVelocity(sim->mouse_body);

    // Decompose velocity into forward and lateral components
    Vector2 forward = Vector2FromAngle(rotation);
    Vector2 right = Vector2FromAngle(rotation + TURN_CW);
    float forward_velocity = Vector2DotProduct(velocity, forward);
    float lateral_velocity = Vector2DotProduct(velocity, right);

    // Actual wheel speeds
    float left_wheel_current_velocity = forward_velocity - angular_velocity * MOUSE_WHEEL_HALF_TRACK;
    float right_wheel_current_velocity = forward_velocity + angular_velocity * MOUSE_WHEEL_HALF_TRACK;

    // Back-EMF motor model: F = (k_t·k_e / R·r²) × (v_target − v_wheel)
    float left_force = MOUSE_MOTOR_K * (left_wheel_target_velocity - left_wheel_current_velocity);
    float right_force = MOUSE_MOTOR_K * (right_wheel_target_velocity - right_wheel_current_velocity);

    // Apply forward force
    float force_magnitude = left_force + right_force;
    Vector2 forward_force = Vector2Scale(forward, force_magnitude);
    b2Body_ApplyForceToCenter(sim->mouse_body, b2Vec2(forward_force.x, forward_force.y), true);

    // Apply lateral no-slip impulse: cancel velocity perpendicular to heading.
    float lateral_impulse_magnitude = -0.5f * MOUSE_MASS * lateral_velocity;
    Vector2 lateral_impulse = Vector2Scale(right, lateral_impulse_magnitude);
    b2Body_ApplyLinearImpulseToCenter(sim->mouse_body, b2Vec2(lateral_impulse.x, lateral_impulse.y), true);

    // Apply torque
    float tau = (right_force - left_force) * MOUSE_WHEEL_HALF_TRACK;
    b2Body_ApplyTorque(sim->mouse_body, tau, true);
}

static void UpdateMouseController(Sim *sim)
{
    Vector2 position = sim->mouse_position;
    float rotation = sim->mouse_rotation;

    // Current distance
    Vector2 position_error = Vector2Subtract(position, sim->reference_position);

    // Project position_error onto forward direction
    Vector2 forward = Vector2FromAngle(rotation);
    float distance_current = Vector2DotProduct(position_error, forward);

    // Distance error
    float distance_error = sim->target_distance - distance_current;

    // Rotation error
    float rotation_error = AngleDiff(rotation, sim->target_rotation);

    // Current body velocities for D term
    b2Vec2 b2_velocity = b2Body_GetLinearVelocity(sim->mouse_body);
    float forward_velocity = Vector2DotProduct({b2_velocity.x, b2_velocity.y}, forward);
    float angular_velocity = b2Body_GetAngularVelocity(sim->mouse_body);

    // PD control: P drives toward setpoint, D damps velocity to prevent overshoot
    float velocity_target = MOUSE_KP_DISTANCE * distance_error - MOUSE_KD_DISTANCE * forward_velocity;
    float angular_velocity_target = MOUSE_KP_ROTATION * rotation_error - MOUSE_KD_ROTATION * angular_velocity;

    // Clamp speeds to physical maximum
    velocity_target = std::clamp(velocity_target,
                                 -MOUSE_WHEEL_VELOCITY_MAX, MOUSE_WHEEL_VELOCITY_MAX);
    angular_velocity_target = std::clamp(angular_velocity_target,
                                         -WHEEL_ANGULAR_VELOCITY_MAX, WHEEL_ANGULAR_VELOCITY_MAX);

    // Differential drive: clamp target wheel speeds to physical maximum
    float left_velocity_target = velocity_target - angular_velocity_target;
    float right_velocity_target = velocity_target + angular_velocity_target;

    sim->state.setpoint_distance = distance_error / sim->odometry_distance_error;
    sim->state.setpoint_rotation = rotation_error / sim->odometry_rotation_error;

    ApplyDrive(sim, left_velocity_target, right_velocity_target);
}

// Public API

Sim *CreateSim(const Maze *maze)
{
    Sim *sim = new Sim();
    sim->maze = maze;

    ResetCellColors(sim);

    b2WorldDef wd = b2DefaultWorldDef();
    wd.gravity = b2Vec2(0.0f, 0.0f);
    sim->world = b2CreateWorld(&wd);

    CreateMazePhysics(sim);
    CreateMousePhysics(sim);

    ResetMousePhysics(sim);

    return sim;
}

void DestroySim(Sim *sim)
{
    b2DestroyWorld(sim->world);

    delete sim;
}

bool ResetSim(Sim *sim)
{
    if (!StartRun(sim))
        return false;

    ResetMousePhysics(sim);

    return true;
}

bool IsSimRunning(Sim *sim)
{
    if (sim->state.time >= RUN_TIME_MAX)
        return false;

    if (sim->state.run_number == 0)
        return false;

    return true;
}

void UpdateSim(Sim *sim, float dt)
{
    // Update mouse controller
    UpdateMouseController(sim);

    // Step physics
    b2World_Step(sim->world, dt, 8);

    // Update timers
    if (sim->state.run_number >= 1)
    {
        sim->state.time += dt;
        if (sim->state.time >= RUN_TIME_MAX)
            sim->state.time = RUN_TIME_MAX;
    }

    if (sim->state.run_state == RUNSTATE_RUNNING)
    {
        sim->state.run_time += dt;
        if (sim->state.run_time > RUN_TIME_MAX)
            sim->state.run_time = RUN_TIME_MAX;
    }

    // Update IMU readings (accelerometer and gyroscope)
    UpdateIMU(sim, dt);

    // Update mouse state
    UpdateMouseState(sim);

    Cell cell = PositionToCell(sim->mouse_position);

    // Check start position
    switch (sim->state.run_state)
    {
    case RUNSTATE_IDLE:
        if (!isStartCell(cell))
            sim->state.run_state = RUNSTATE_RUNNING;

        break;

    case RUNSTATE_RUNNING:
        if (IsGoalCell(cell))
        {
            sim->state.run_state = RUNSTATE_RETURNING;

            if (sim->state.run_time < sim->state.run_time_best || sim->state.run_time_best == 0.0f)
                sim->state.run_time_best = sim->state.run_time;
        }

        break;

    case RUNSTATE_RETURNING:
        if (isStartCell(cell))
            StartRun(sim);

        break;
    }
}

Vector2 GetMousePosition(Sim *sim)
{
    return sim->mouse_position;
}

float GetMouseRotation(Sim *sim)
{
    return sim->mouse_rotation;
}

const SimState *GetSimState(Sim *sim)
{
    return &sim->state;
}

void SetMouseSetpoint(Sim *sim, float distance, float rotation)
{
    // Set reference position and rotation for the mouse controller
    sim->reference_position = sim->mouse_position;
    sim->reference_rotation = sim->mouse_rotation;

    // Odometry error
    static std::mt19937 rng(std::random_device{}());
    static std::normal_distribution<float> noise(0.0f, 1.0f);
    sim->odometry_distance_error = 1.0f + ODOMETRY_DISTANCE_ERROR * noise(rng);
    sim->odometry_rotation_error = 1.0f + ODOMETRY_ROTATION_ERROR * noise(rng);

    // Set target distance and rotation for the mouse controller, applying odometry error.
    sim->target_distance = distance * sim->odometry_distance_error;
    sim->target_rotation = sim->mouse_rotation + rotation * sim->odometry_rotation_error;

    // Set remaining distance and rotation for mouse agent.
    sim->state.setpoint_distance = distance;
    sim->state.setpoint_rotation = rotation;
}
