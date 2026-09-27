/**
 * Teseo Micromouse Virtual Competition
 * Navigation layer
 *
 * @brief Moves the mouse from cell to cell and keeps track of where it is.
 *
 *        How it works:
 *        - Estimation: the pose is integrated from the wheel encoders (distance) and the
 *          gyroscope (rotation). Both drift, so while driving straight the IR sensors correct it:
 *          side walls give the distance to the center line and the heading, and a wall
 *          ahead gives the distance along the corridor.
 *        - Motion: the mouse turns in place at cell centers and drives straight lines with a
 *          trapezoidal speed profile, steering towards the center line.
 *        - Safety: the mouse compares the walls it sees with the ones it saw before.
 *          If they contradict each other, it is lost and stops.
 * @author Theseús the hero
 */

#include <algorithm>
#include <cmath>
#include <cstring>

#include "nav.h"

// Motion

#define NAV_ACCELERATION 2.0f      // m/s²
#define NAV_SPEED_MIN 0.05f        // m/s, final approach speed
#define NAV_DRIVE_DONE 0.002f      // m, distance to the cell center to finish a straight line
#define NAV_TURN_SPEED_MAX 6.0f    // rad/s
#define NAV_TURN_ACCELERATION 30.0f // rad/s²
#define NAV_TURN_DONE 0.01f        // rad, remaining rotation to finish a turn (≈0.6°)

// Steering towards the center line

#define NAV_STEER_LATERAL 10.0f  // rad of heading per m of lateral error
#define NAV_STEER_HEADING_MAX 0.3f // rad, maximum heading correction
#define NAV_STEER_GAIN 12.0f     // rad/s per rad of heading error

// Estimate corrections with the IR sensors

#define NAV_WALL_DETECT 0.12f                                  // m, closer readings are walls
#define NAV_WALL_DISTANCE (CELL_HALF_SIZE - WALL_HALF_THICKNESS) // m, from the cell center to a wall
#define NAV_ALIGNED 0.1f                                       // rad, max heading error to trust the IR sensors
#define NAV_FRONT_RANGE 0.3f                                   // m, max front reading used for corrections
#define NAV_LATERAL_OUTLIER 0.015f                             // m, larger lateral corrections are discarded
#define NAV_LATERAL_GAIN 0.05f                                 // Fraction of the lateral error corrected per step
#define NAV_FRONT_GAIN 0.05f                                   // Fraction of the longitudinal error corrected per step
#define NAV_HEADING_GAIN 0.2f                                  // Fraction of the heading error corrected per sample
#define NAV_HEADING_SAMPLE 0.02f                               // m traveled between heading samples

// Stall detection

#define NAV_STALL_TIME 0.3f // s without moving while trying to move

enum NavMode
{
    NAV_IDLE,
    NAV_TURNING,
    NAV_DRIVING,
    NAV_LOST,
};

static struct
{
    // Pose estimate
    Vector2 position;
    float rotation;
    float encoder_distance_last;
    float distance_traveled;

    // Belief
    Cell cell;
    Heading heading;
    uint8_t walls;

    // Path
    Heading path[NAV_PATH_MAX];
    int path_length;
    int path_index;

    // Current maneuver
    NavMode mode;
    Vector2 target; // Center of the destination cell, while driving
    float velocity;
    float angular_velocity;
    float max_speed;
    float stall_time;

    // Last side wall measurement, for the heading correction
    bool wall_sample_valid;
    int wall_sample_sides; // 1 = left wall, 2 = right wall, 3 = both
    float wall_sample_lateral;
    float wall_sample_distance;

    // Walls seen so far, to notice contradictions
    bool seen[GRID_SIZE][GRID_SIZE];
    uint8_t seen_walls[GRID_SIZE][GRID_SIZE];
} nav;

// Helpers

uint8_t HeadingToWall(Heading heading)
{
    return 1 << heading; // WALL_NORTH, WALL_EAST, WALL_SOUTH, WALL_WEST
}

Cell GetNeighborCell(Cell cell, Heading heading)
{
    const int32_t dx[] = {0, 1, 0, -1};
    const int32_t dy[] = {1, 0, -1, 0};

    return {cell.x + dx[heading], cell.y + dy[heading]};
}

static Heading RotateHeading(Heading heading, int quarter_turns_clockwise)
{
    return (Heading)(((heading + quarter_turns_clockwise) % 4 + 4) % 4);
}

static float HeadingToRotation(Heading heading)
{
    return ROTATION_NORTH + (float)heading * TURN_CW;
}

static Vector2 GetCellCenter(Cell cell)
{
    return {(cell.x + 0.5f) * CELL_SIZE, (cell.y + 0.5f) * CELL_SIZE};
}

static float GetEncoderDistance(const SimState *state)
{
    return 0.5f * (state->encoders[ENCODER_LEFT] + state->encoders[ENCODER_RIGHT]);
}

// Belief

static void SetLost(Sim *sim)
{
    nav.mode = NAV_LOST;
    nav.velocity = 0.0f;
    nav.angular_velocity = 0.0f;

    SetStatusText(sim, "LOST! Press [R]");
}

static uint8_t ReadWalls(const SimState *state)
{
    uint8_t walls = 0;

    if (state->ir_sensors[IR_SENSOR_FRONT] < NAV_WALL_DETECT)
        walls |= HeadingToWall(nav.heading);
    if (state->ir_sensors[IR_SENSOR_LEFT] < NAV_WALL_DETECT)
        walls |= HeadingToWall(RotateHeading(nav.heading, -1));
    if (state->ir_sensors[IR_SENSOR_RIGHT] < NAV_WALL_DETECT)
        walls |= HeadingToWall(RotateHeading(nav.heading, 1));

    // Behind is open (the mouse came from there), except at the maze border
    Heading back = RotateHeading(nav.heading, 2);
    if (!ValidateCell(GetNeighborCell(nav.cell, back)))
        walls |= HeadingToWall(back);

    return walls;
}

static void CheckWalls(Sim *sim)
{
    Cell cell = nav.cell;

    if (!ValidateCell(cell))
    {
        SetLost(sim);
        return;
    }

    for (int i = 0; i < 4; i++)
    {
        Heading heading = (Heading)i;
        uint8_t wall = HeadingToWall(heading);
        bool has_wall = nav.walls & wall;

        // The maze border always has walls
        Cell neighbor = GetNeighborCell(cell, heading);
        if (!ValidateCell(neighbor) && !has_wall)
        {
            SetLost(sim);
            return;
        }

        // The wall must match what was seen before, from this cell or from the neighbor
        if (nav.seen[cell.x][cell.y] && ((nav.seen_walls[cell.x][cell.y] & wall) != 0) != has_wall)
        {
            SetLost(sim);
            return;
        }

        uint8_t neighbor_wall = HeadingToWall(RotateHeading(heading, 2));
        if (ValidateCell(neighbor) && nav.seen[neighbor.x][neighbor.y] &&
            ((nav.seen_walls[neighbor.x][neighbor.y] & neighbor_wall) != 0) != has_wall)
        {
            SetLost(sim);
            return;
        }
    }

    nav.seen[cell.x][cell.y] = true;
    nav.seen_walls[cell.x][cell.y] = nav.walls;
}

static void Arrive(Sim *sim, const SimState *state)
{
    nav.mode = NAV_IDLE;
    nav.velocity = 0.0f;
    nav.angular_velocity = 0.0f;

    nav.cell = PositionToCell(nav.position);
    nav.walls = ReadWalls(state);

    CheckWalls(sim);
}

// Estimation

static void UpdateOdometry(const SimState *state)
{
    float encoder_distance = GetEncoderDistance(state);
    float distance = encoder_distance - nav.encoder_distance_last;
    nav.encoder_distance_last = encoder_distance;

    float rotation = state->gyroscope * SIM_TIMESTEP;

    nav.position = Vector2Add(nav.position, Vector2FromAngle(nav.rotation + 0.5f * rotation, distance));
    nav.rotation = AngleDiff(0.0f, nav.rotation + rotation);
    nav.distance_traveled += distance;
}

static void CorrectWithWalls(const SimState *state, Vector2 axis, Vector2 normal)
{
    float heading_error = AngleDiff(HeadingToRotation(nav.heading), nav.rotation); // Positive = rotated left
    if (fabsf(heading_error) > NAV_ALIGNED)
    {
        nav.wall_sample_valid = false;
        return;
    }

    float cos_error = cosf(heading_error);

    // Side walls: distance to the center line (positive = left of it)
    float left = state->ir_sensors[IR_SENSOR_LEFT] * cos_error;
    float right = state->ir_sensors[IR_SENSOR_RIGHT] * cos_error;
    bool left_wall = left < NAV_WALL_DETECT;
    bool right_wall = right < NAV_WALL_DETECT;

    float measured = 0.0f;
    if (left_wall && right_wall)
        measured = 0.5f * ((NAV_WALL_DISTANCE - left) + (right - NAV_WALL_DISTANCE));
    else if (left_wall)
        measured = NAV_WALL_DISTANCE - left;
    else if (right_wall)
        measured = right - NAV_WALL_DISTANCE;

    // Discard readings far from the estimate: near the end of a wall, the sensor grazes its edge
    float lateral = Vector2DotProduct(Vector2Subtract(nav.position, nav.target), normal);
    int sides = left_wall + 2 * right_wall;
    bool valid = sides && fabsf(measured - lateral) < NAV_LATERAL_OUTLIER;

    if (valid)
    {
        nav.position = Vector2Add(nav.position, Vector2Scale(normal, NAV_LATERAL_GAIN * (measured - lateral)));

        // Heading: how fast the distance to the wall changes along the way
        if (!nav.wall_sample_valid || nav.wall_sample_sides != sides)
        {
            nav.wall_sample_valid = true;
            nav.wall_sample_sides = sides;
            nav.wall_sample_lateral = measured;
            nav.wall_sample_distance = nav.distance_traveled;
        }
        else if (nav.distance_traveled - nav.wall_sample_distance >= NAV_HEADING_SAMPLE)
        {
            float slope = (measured - nav.wall_sample_lateral) / (nav.distance_traveled - nav.wall_sample_distance);
            float measured_error = atanf(slope);

            if (fabsf(measured_error) < NAV_ALIGNED)
                nav.rotation += NAV_HEADING_GAIN * (measured_error - heading_error);

            nav.wall_sample_lateral = measured;
            nav.wall_sample_distance = nav.distance_traveled;
        }
    }
    else
        nav.wall_sample_valid = false;

    // Wall ahead: distance along the corridor. Walls lie on cell boundaries.
    float front = state->ir_sensors[IR_SENSOR_FRONT] * cos_error;
    if (front < NAV_FRONT_RANGE)
    {
        float along = Vector2DotProduct(nav.position, axis);
        float boundary = roundf((along + front + WALL_HALF_THICKNESS) / CELL_SIZE) * CELL_SIZE;
        float measured_along = boundary - WALL_HALF_THICKNESS - front;

        if (fabsf(measured_along - along) < 0.25f * CELL_SIZE)
            nav.position = Vector2Add(nav.position, Vector2Scale(axis, NAV_FRONT_GAIN * (measured_along - along)));
    }
}

// Motion

static void StartNextMove()
{
    if (nav.path_index >= nav.path_length)
        return;

    Heading heading = nav.path[nav.path_index];

    // Turn in place first if needed
    if (heading != nav.heading)
    {
        nav.heading = heading;
        nav.mode = NAV_TURNING;
        nav.stall_time = 0.0f;
        return;
    }

    // Drive all consecutive moves in the same direction as one straight line
    Cell target = nav.cell;
    while (nav.path_index < nav.path_length && nav.path[nav.path_index] == heading)
    {
        target = GetNeighborCell(target, heading);
        nav.path_index++;
    }

    nav.target = GetCellCenter(target);
    nav.mode = NAV_DRIVING;
    nav.stall_time = 0.0f;
    nav.wall_sample_valid = false;
}

static void UpdateTurn(Sim *sim, const SimState *state)
{
    float error = AngleDiff(nav.rotation, HeadingToRotation(nav.heading));

    if (fabsf(error) < NAV_TURN_DONE && fabsf(state->gyroscope) < 0.2f)
    {
        nav.mode = NAV_IDLE;
        nav.angular_velocity = 0.0f;
        return;
    }

    // Fastest rotation that can still stop in time
    float speed = std::min(NAV_TURN_SPEED_MAX, sqrtf(2.0f * NAV_TURN_ACCELERATION * fabsf(error)));
    float target = copysignf(speed, error);
    float max_change = NAV_TURN_ACCELERATION * SIM_TIMESTEP;

    nav.velocity = 0.0f;
    nav.angular_velocity += std::clamp(target - nav.angular_velocity, -max_change, max_change);

    // Stalled: trying to turn, but the gyroscope says it does not
    if (fabsf(nav.angular_velocity) > 0.5f && fabsf(state->gyroscope) < 0.05f)
        nav.stall_time += SIM_TIMESTEP;
    else
        nav.stall_time = 0.0f;

    if (nav.stall_time > NAV_STALL_TIME)
        SetLost(sim);
}

static void UpdateDrive(Sim *sim, const SimState *state)
{
    Vector2 axis = Vector2FromAngle(HeadingToRotation(nav.heading));
    Vector2 normal = {-axis.y, axis.x}; // Points left

    CorrectWithWalls(state, axis, normal);

    Vector2 offset = Vector2Subtract(nav.position, nav.target);
    float remaining = -Vector2DotProduct(offset, axis);
    float lateral = Vector2DotProduct(offset, normal);
    float heading_error = AngleDiff(HeadingToRotation(nav.heading), nav.rotation);

    // A wall blocks the path: stop at the center of the cell before it
    float front = state->ir_sensors[IR_SENSOR_FRONT];
    if (fabsf(heading_error) < NAV_ALIGNED && front < NAV_FRONT_RANGE &&
        front < remaining + NAV_WALL_DISTANCE - 0.25f * CELL_SIZE)
    {
        Vector2 stop = Vector2Add(nav.position, Vector2Scale(axis, front - NAV_WALL_DISTANCE));
        nav.target = GetCellCenter(PositionToCell(stop));
        nav.path_index = nav.path_length;

        offset = Vector2Subtract(nav.position, nav.target);
        remaining = -Vector2DotProduct(offset, axis);
    }

    if (remaining < NAV_DRIVE_DONE)
    {
        Arrive(sim, state);
        return;
    }

    // Trapezoidal speed profile: accelerate up to the maximum speed, brake in time to stop
    float speed = std::min(nav.max_speed, sqrtf(2.0f * NAV_ACCELERATION * remaining));
    speed = std::max(speed, NAV_SPEED_MIN);
    nav.velocity = std::min(nav.velocity + NAV_ACCELERATION * SIM_TIMESTEP, speed);

    // Steer towards the center line
    float heading_target = -std::clamp(NAV_STEER_LATERAL * lateral, -NAV_STEER_HEADING_MAX, NAV_STEER_HEADING_MAX);
    nav.angular_velocity = NAV_STEER_GAIN * (heading_target - heading_error);

    // Stalled: trying to drive, but the encoders say it does not move
    float encoder_speed = fabsf(GetEncoderDistance(state) - nav.encoder_distance_last) / SIM_TIMESTEP;
    if (nav.velocity > 2.0f * NAV_SPEED_MIN && encoder_speed < 0.01f)
        nav.stall_time += SIM_TIMESTEP;
    else
        nav.stall_time = 0.0f;

    if (nav.stall_time > NAV_STALL_TIME)
        SetLost(sim);
}

// Public API

void NavReset(Sim *sim)
{
    const SimState *state = GetSimState(sim);

    nav.position = GetCellCenter({0, 0});
    nav.rotation = ROTATION_NORTH;
    nav.encoder_distance_last = GetEncoderDistance(state);
    nav.distance_traveled = 0.0f;

    nav.cell = {0, 0};
    nav.heading = HEADING_NORTH;

    nav.path_length = 0;
    nav.path_index = 0;

    nav.mode = NAV_IDLE;
    nav.velocity = 0.0f;
    nav.angular_velocity = 0.0f;
    nav.stall_time = 0.0f;
    nav.wall_sample_valid = false;

    if (nav.max_speed == 0.0f)
        nav.max_speed = NAV_SPEED_DEFAULT;

    memset(nav.seen, 0, sizeof(nav.seen));

    nav.walls = ReadWalls(state);
    CheckWalls(sim);

    SetMouseVelocity(sim, 0.0f, 0.0f);
}

void NavUpdate(Sim *sim)
{
    const SimState *state = GetSimState(sim);

    // The encoder distance of the previous step is needed by the stall detection, so
    // odometry is updated after the maneuver.
    switch (nav.mode)
    {
    case NAV_IDLE:
        StartNextMove();
        break;

    case NAV_TURNING:
        UpdateTurn(sim, state);
        break;

    case NAV_DRIVING:
        UpdateDrive(sim, state);
        break;

    case NAV_LOST:
        break;
    }

    UpdateOdometry(state);

    SetMouseVelocity(sim, nav.velocity, nav.angular_velocity);
    SetEstimatedPose(sim, nav.position, nav.rotation);
}

bool NavIsIdle()
{
    return nav.mode == NAV_IDLE && nav.path_index >= nav.path_length;
}

bool NavIsLost()
{
    return nav.mode == NAV_LOST;
}

Cell NavGetCell()
{
    return nav.cell;
}

Heading NavGetHeading()
{
    return nav.heading;
}

uint8_t NavGetWalls()
{
    return nav.walls;
}

void NavFollowPath(const Heading *path, int count)
{
    if (nav.mode == NAV_LOST)
        return;

    count = std::clamp(count, 0, NAV_PATH_MAX);

    memcpy(nav.path, path, count * sizeof(Heading));
    nav.path_length = count;
    nav.path_index = 0;
}

void NavSetSpeed(float max_speed)
{
    nav.max_speed = std::clamp(max_speed, NAV_SPEED_MIN, MOUSE_WHEEL_VELOCITY_MAX);
}
