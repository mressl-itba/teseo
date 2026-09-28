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
#include <cstring>
#include <random>
#include <vector>

#include <box2d/box2d.h>

#include "sim.h"

// Sensor angles

const float IR_SENSOR_ANGLES[IR_SENSOR_NUM] = {
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
    float floor_grip[GRID_SIZE][GRID_SIZE]; // Friction coefficient of each cell

    // Box2D world and bodies
    b2WorldId world;
    b2BodyId mouse_body;
    std::vector<float> reflectivity; // IR reflectivity of each wall and post shape (index = shape user data - 1)

    // Random numbers for all the errors, from the noise seed
    std::mt19937 rng;
    std::normal_distribution<float> normal;

    // Sim state
    Vector2 mouse_position;      // World position (meters, north/east coordinates)
    float mouse_rotation;        // Rotation (radians, CCW+, 0 = East)
    Vector2 mouse_velocity_last; // World-frame velocity, kept across steps for acceleration computation

    SimState state;

    // Velocity controller
    float target_velocity;           // Forward velocity set by the agent (m/s)
    float target_angular_velocity;   // Angular velocity set by the agent (rad/s)
    float velocity_integral;         // Integral of the forward velocity error (m)
    float angular_velocity_integral; // Integral of the angular velocity error (rad)
    float encoder_distance_last;     // Mean encoder reading at the last step, for measuring velocity

    // Wheel encoders
    float encoder_scale[ENCODER_NUM]; // Scale error of each wheel (fixed per mouse)
    float wheel_slip[ENCODER_NUM];    // Wheel surface speed minus ground speed (m/s), non-zero when slipping
    float wheel_speed[ENCODER_NUM];   // Wheel surface speed while slipping (m/s)
    bool wheel_slipping[ENCODER_NUM]; // True while the wheel slips on the floor
    Vector2 encoder_last_position;    // Mouse pose at the last encoder update
    float encoder_last_rotation;

    // Gyroscope drift
    float gyroscope_bias; // Current gyroscope bias (rad/s)

    // Agent's pose estimate (for display only)
    bool estimated_pose_valid;
    Vector2 estimated_position;
    float estimated_rotation;

    // Agent's status message
    char status_text[64];
};

// Random numbers

static float RandomGaussian(Sim *sim)
{
    return sim->normal(sim->rng);
}

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

    // Each shape remembers its IR reflectivity through its user data
    auto create_shape = [&](b2Polygon box, float reflectivity)
    {
        sim->reflectivity.push_back(reflectivity);
        shape_def.userData = (void *)(uintptr_t)sim->reflectivity.size();
        b2CreatePolygonShape(walls_body, &shape_def, &box);
    };

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
                create_shape(box, 1.0f + IR_WALL_REFLECTIVITY_NOISE * RandomGaussian(sim));
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
                create_shape(box, 1.0f + IR_WALL_REFLECTIVITY_NOISE * RandomGaussian(sim));
            }
        }
    }

    // Posts: a real maze has one at every cell corner, even where no wall touches it
    for (int32_t x = 0; x <= GRID_SIZE; x++)
    {
        for (int32_t y = 0; y <= GRID_SIZE; y++)
        {
            b2Polygon box = b2MakeOffsetBox(WALL_HALF_THICKNESS, WALL_HALF_THICKNESS,
                                            b2Vec2(x * CELL_SIZE, y * CELL_SIZE), b2MakeRot(0.0f));
            create_shape(box, IR_POST_REFLECTIVITY);
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

    // Compute acceleration in world frame (differentiating in the body frame would drop the centripetal term)
    Vector2 delta_v = Vector2Subtract(velocity, sim->mouse_velocity_last);
    sim->mouse_velocity_last = velocity;

    Vector2 acceleration = Vector2Scale(delta_v, 1.0f / dt);

    // Transform acceleration to body frame (y = forward, x = right)
    sim->state.accelerometer = Vector2Rotate(acceleration, TURN_CCW - sim->mouse_rotation);
}

static void UpdateMouseState(Sim *sim)
{
    b2Vec2 position = b2Body_GetPosition(sim->mouse_body);
    float rotation = b2Rot_GetAngle(b2Body_GetRotation(sim->mouse_body));
    float angular_velocity = b2Body_GetAngularVelocity(sim->mouse_body);

    sim->mouse_position = {position.x, position.y};
    sim->mouse_rotation = rotation;
    sim->state.gyroscope = angular_velocity + sim->gyroscope_bias;

    b2QueryFilter filter = b2DefaultQueryFilter();

    for (int i = 0; i < IR_SENSOR_NUM; i++)
    {
        float ray_angle = rotation + IR_SENSOR_ANGLES[i];
        Vector2 direction = Vector2FromAngle(ray_angle, IR_SENSOR_RANGE_MAX);

        b2Vec2 start = b2Vec2{position.x, position.y};
        b2Vec2 translation = b2Vec2{direction.x, direction.y};

        b2RayResult rayResult = b2World_CastRayClosest(sim->world, start, translation, filter);

        float reading = IR_SENSOR_RANGE_MAX;
        if (rayResult.hit)
        {
            // Less reflective surfaces look farther away (the light falls with the square of the distance)
            uintptr_t shape_index = (uintptr_t)b2Shape_GetUserData(rayResult.shapeId);
            float reflectivity = shape_index ? sim->reflectivity[shape_index - 1] : 1.0f;
            float distance = rayResult.fraction * IR_SENSOR_RANGE_MAX / sqrtf(reflectivity);

            // Noise grows with the square of the distance
            float ratio = distance / IR_SENSOR_NOISE_DISTANCE;
            reading = distance + IR_SENSOR_NOISE * ratio * ratio * RandomGaussian(sim);
        }

        sim->state.ir_sensors[i] = std::clamp(reading, 0.0f, IR_SENSOR_RANGE_MAX);
    }
}

static void ResetMouseController(Sim *sim)
{
    sim->target_velocity = 0.0f;
    sim->target_angular_velocity = 0.0f;
    sim->velocity_integral = 0.0f;
    sim->angular_velocity_integral = 0.0f;
    sim->encoder_distance_last = 0.0f;
}

static void UpdateGyroscopeDrift(Sim *sim, float dt)
{
    sim->gyroscope_bias += GYROSCOPE_BIAS_WALK * sqrtf(dt) * RandomGaussian(sim);
}

static float GetEncoderDistance(Sim *sim)
{
    // Distance traveled as the mouse measures it (mean of both wheel encoders).
    return 0.5f * (sim->state.encoders[ENCODER_LEFT] + sim->state.encoders[ENCODER_RIGHT]);
}

static void ResetEncoders(Sim *sim)
{
    sim->state.encoders[ENCODER_LEFT] = 0.0f;
    sim->state.encoders[ENCODER_RIGHT] = 0.0f;

    sim->encoder_last_position = sim->mouse_position;
    sim->encoder_last_rotation = sim->mouse_rotation;
}

static void UpdateEncoders(Sim *sim)
{
    // Body displacement since the last update, split into forward motion and rotation.
    // Sideways motion (e.g. sliding along a wall) is not seen by the wheels.
    Vector2 delta_position = Vector2Subtract(sim->mouse_position, sim->encoder_last_position);
    float delta_rotation = AngleDiff(sim->encoder_last_rotation, sim->mouse_rotation);
    Vector2 forward = Vector2FromAngle(sim->encoder_last_rotation + 0.5f * delta_rotation);
    float delta_distance = Vector2DotProduct(delta_position, forward);

    // Slipping wheels turn more than the mouse moves, and the encoders count it
    float wheel_delta[ENCODER_NUM] = {
        delta_distance - delta_rotation * MOUSE_WHEEL_HALF_TRACK + sim->wheel_slip[ENCODER_LEFT] * SIM_TIMESTEP,
        delta_distance + delta_rotation * MOUSE_WHEEL_HALF_TRACK + sim->wheel_slip[ENCODER_RIGHT] * SIM_TIMESTEP,
    };

    // Random slip: its variance grows with the distance traveled, whatever the time step
    float slip_noise = ENCODER_SLIP_NOISE / sqrtf(SIM_TIMESTEP);

    for (int i = 0; i < ENCODER_NUM; i++)
        sim->state.encoders[i] += wheel_delta[i] * (sim->encoder_scale[i] + slip_noise * RandomGaussian(sim));

    sim->encoder_last_position = sim->mouse_position;
    sim->encoder_last_rotation = sim->mouse_rotation;
}

static void ResetMousePhysics(Sim *sim)
{
    // Reset mouse state
    Vector2 position = {0.5f * CELL_SIZE, 0.5f * CELL_SIZE};
    float rotation = ROTATION_NORTH;

    // New residual gyroscope bias for this run (as after a gyroscope calibration at the start cell)
    sim->gyroscope_bias = GYROSCOPE_BIAS * RandomGaussian(sim);

    sim->estimated_pose_valid = false;
    sim->status_text[0] = '\0';

    b2Body_SetTransform(sim->mouse_body, b2Vec2(position.x, position.y), b2MakeRot(rotation));
    b2Body_SetLinearVelocity(sim->mouse_body, b2Vec2(0.0f, 0.0f));
    b2Body_SetAngularVelocity(sim->mouse_body, 0.0f);

    UpdateMouseState(sim);
    sim->mouse_velocity_last = {0.0f, 0.0f};
    sim->state.accelerometer = {0.0f, 0.0f};

    for (int i = 0; i < ENCODER_NUM; i++)
    {
        sim->wheel_slipping[i] = false;
        sim->wheel_speed[i] = 0.0f;
        sim->wheel_slip[i] = 0.0f;
    }

    ResetEncoders(sim);

    // Reset controller
    ResetMouseController(sim);
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

    // Tire grip: each wheel carries half the weight. While the motor force is within the grip, the
    // wheel rolls with the floor. Beyond it, the wheel slips: the floor pushes with the grip force
    // only, and the wheel speeds up or slows down with its own inertia until it rolls again.
    Cell cell = PositionToCell(sim->mouse_position);
    float grip = ValidateCell(cell) ? sim->floor_grip[cell.x][cell.y] : MOUSE_TIRE_GRIP;
    float wheel_grip = 0.5f * grip * MOUSE_MASS * GRAVITY;

    float wheel_target[ENCODER_NUM] = {left_wheel_target_velocity, right_wheel_target_velocity};
    float wheel_ground[ENCODER_NUM] = {left_wheel_current_velocity, right_wheel_current_velocity};
    float wheel_force[ENCODER_NUM];

    for (int i = 0; i < ENCODER_NUM; i++)
    {
        if (!sim->wheel_slipping[i])
        {
            // Rolling: the motor moves the wheel and the mouse together, so the floor transmits the
            // mouse's share of the motor force (each wheel moves half the mouse), unless it exceeds the grip
            const float mouse_share = 0.5f * MOUSE_MASS;
            float motor_force = MOUSE_MOTOR_K * (wheel_target[i] - wheel_ground[i]);
            wheel_force[i] = motor_force * mouse_share / (mouse_share + MOUSE_WHEEL_INERTIA);

            if (fabsf(wheel_force[i]) > wheel_grip)
            {
                sim->wheel_slipping[i] = true;
                sim->wheel_speed[i] = wheel_ground[i];
            }
        }

        if (sim->wheel_slipping[i])
        {
            // Slipping: kinetic friction against the relative motion, the wheel follows its own dynamics
            float motor_force = MOUSE_MOTOR_K * (wheel_target[i] - sim->wheel_speed[i]);
            float relative = sim->wheel_speed[i] - wheel_ground[i];
            float direction = relative != 0.0f ? copysignf(1.0f, relative) : copysignf(1.0f, motor_force);
            wheel_force[i] = direction * wheel_grip;

            sim->wheel_speed[i] += (motor_force - wheel_force[i]) / MOUSE_WHEEL_INERTIA * SIM_TIMESTEP;

            // Rolls again when the wheel catches up with the floor
            if ((sim->wheel_speed[i] - wheel_ground[i]) * direction <= 0.0f)
                sim->wheel_slipping[i] = false;
        }

        sim->wheel_slip[i] = sim->wheel_slipping[i] ? sim->wheel_speed[i] - wheel_ground[i] : 0.0f;
    }

    left_force = wheel_force[ENCODER_LEFT];
    right_force = wheel_force[ENCODER_RIGHT];

    // Apply forward force
    float force_magnitude = left_force + right_force;
    Vector2 forward_force = Vector2Scale(forward, force_magnitude);
    b2Body_ApplyForceToCenter(sim->mouse_body, b2Vec2(forward_force.x, forward_force.y), true);

    // Apply lateral friction: cancel velocity perpendicular to heading, up to the tire grip (skid).
    float lateral_impulse_max = grip * MOUSE_MASS * GRAVITY * SIM_TIMESTEP;
    float lateral_impulse_magnitude = std::clamp(-MOUSE_MASS * lateral_velocity, -lateral_impulse_max, lateral_impulse_max);
    Vector2 lateral_impulse = Vector2Scale(right, lateral_impulse_magnitude);
    b2Body_ApplyLinearImpulseToCenter(sim->mouse_body, b2Vec2(lateral_impulse.x, lateral_impulse.y), true);

    // Apply torque
    float tau = (right_force - left_force) * MOUSE_WHEEL_HALF_TRACK;
    b2Body_ApplyTorque(sim->mouse_body, tau, true);
}

static void UpdateMouseController(Sim *sim, float dt)
{
    // Measured velocities: the motor driver only knows what the encoders and the gyroscope tell it
    float encoder_distance = GetEncoderDistance(sim);
    float velocity = (encoder_distance - sim->encoder_distance_last) / dt;
    float angular_velocity = sim->state.gyroscope;
    sim->encoder_distance_last = encoder_distance;

    // PI control around a feedforward term: the motor model reaches the target by itself
    // with perfect sensors, the integral term makes it track what the sensors measure.
    float velocity_error = sim->target_velocity - velocity;
    float angular_velocity_error = sim->target_angular_velocity - angular_velocity;

    float velocity_command = sim->target_velocity +
                             MOUSE_VELOCITY_KP * velocity_error +
                             MOUSE_VELOCITY_KI * sim->velocity_integral;
    float angular_velocity_command = sim->target_angular_velocity +
                                     MOUSE_VELOCITY_KP * angular_velocity_error +
                                     MOUSE_VELOCITY_KI * sim->angular_velocity_integral;

    // Differential drive, limited by the maximum wheel velocity (motor voltage)
    float left_command = velocity_command - angular_velocity_command * MOUSE_WHEEL_HALF_TRACK;
    float right_command = velocity_command + angular_velocity_command * MOUSE_WHEEL_HALF_TRACK;

    bool saturated = fabsf(left_command) > MOUSE_WHEEL_VELOCITY_MAX ||
                     fabsf(right_command) > MOUSE_WHEEL_VELOCITY_MAX;

    left_command = std::clamp(left_command, -MOUSE_WHEEL_VELOCITY_MAX, MOUSE_WHEEL_VELOCITY_MAX);
    right_command = std::clamp(right_command, -MOUSE_WHEEL_VELOCITY_MAX, MOUSE_WHEEL_VELOCITY_MAX);

    // Integrate only while not saturated (anti-windup)
    if (!saturated)
    {
        sim->velocity_integral += velocity_error * dt;
        sim->angular_velocity_integral += angular_velocity_error * dt;
    }

    // A zero target means "stop": forget what was accumulated (e.g. while pushing against a wall),
    // or the mouse would keep moving after being told to stop
    if (sim->target_velocity == 0.0f)
        sim->velocity_integral = 0.0f;
    if (sim->target_angular_velocity == 0.0f)
        sim->angular_velocity_integral = 0.0f;

    ApplyDrive(sim, left_command, right_command);
}

// Public API

Sim *CreateSim(const Maze *maze, uint32_t noise_seed)
{
    Sim *sim = new Sim();
    sim->maze = maze;
    sim->rng.seed(noise_seed);

    ResetCellColors(sim);

    b2WorldDef wd = b2DefaultWorldDef();
    wd.gravity = b2Vec2(0.0f, 0.0f);
    sim->world = b2CreateWorld(&wd);

    CreateMazePhysics(sim);
    CreateMousePhysics(sim);

    // Floor grip, fixed for the whole simulation
    for (int x = 0; x < GRID_SIZE; x++)
        for (int y = 0; y < GRID_SIZE; y++)
            sim->floor_grip[x][y] = std::max(MOUSE_TIRE_GRIP + MOUSE_TIRE_GRIP_NOISE * RandomGaussian(sim), 0.5f);

    // Wheel scale errors are a property of the mouse, so they stay the same across runs:
    // a common error (wheel diameter) plus a smaller left/right mismatch.
    float encoder_scale = 1.0f + ENCODER_SCALE_ERROR * RandomGaussian(sim);
    for (int i = 0; i < ENCODER_NUM; i++)
        sim->encoder_scale[i] = encoder_scale * (1.0f + ENCODER_MISMATCH_ERROR * RandomGaussian(sim));

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

void UpdateSim(Sim *sim)
{
    const float dt = SIM_TIMESTEP;

    // Update mouse controller
    UpdateMouseController(sim, dt);

    // Step physics
    b2World_Step(sim->world, dt, 4);

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

    // Update gyroscope drift
    UpdateGyroscopeDrift(sim, dt);

    // Update mouse state (position, rotation, gyroscope and IR sensors)
    UpdateMouseState(sim);

    // Update wheel encoders (uses the pose just updated)
    UpdateEncoders(sim);

    // Update accelerometer (uses the rotation just updated)
    UpdateIMU(sim, dt);

    Cell cell = PositionToCell(sim->mouse_position);

    // Check start position
    switch (sim->state.run_state)
    {
    case RUNSTATE_IDLE:
        if (!IsStartCell(cell))
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
        if (IsStartCell(cell))
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

void SetMouseVelocity(Sim *sim, float linear, float angular)
{
    // Invalid values (NaN, infinity) would break the physics: treat them as "stop"
    sim->target_velocity = std::isfinite(linear) ? linear : 0.0f;
    sim->target_angular_velocity = std::isfinite(angular) ? angular : 0.0f;
}

void SetEstimatedPose(Sim *sim, Vector2 position, float rotation)
{
    sim->estimated_pose_valid = true;
    sim->estimated_position = position;
    sim->estimated_rotation = rotation;
}

bool GetEstimatedPose(Sim *sim, Vector2 *position, float *rotation)
{
    if (!sim->estimated_pose_valid)
        return false;

    *position = sim->estimated_position;
    *rotation = sim->estimated_rotation;

    return true;
}

void SetStatusText(Sim *sim, const char *text)
{
    snprintf(sim->status_text, sizeof(sim->status_text), "%s", text);
}

const char *GetStatusText(Sim *sim)
{
    return sim->status_text;
}
