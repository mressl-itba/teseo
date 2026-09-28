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
 *        - Recovery: if a turn gets stuck (a corner touches a wall because the mouse stopped
 *          off-center), the mouse turns back, moves forward a little and tries again.
 *          If a straight line gets stuck, the mouse backs up to the center of the cell behind it
 *          and the move ends.
 *        - Safety: the layer compares the walls it sees with the ones it saw before.
 *          If they contradict each other, it is lost and stops.
 * @author Theseús the hero
 */

#include <algorithm>
#include <cmath>

#include "nav.h"

// Motion

#define NAV_SPEED_MIN 0.05f        // m/s, final approach speed
#define NAV_BRAKE_MARGIN 0.7f      // Braking is planned with this fraction of the acceleration, to stop in time
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
#define NAV_FRONT_RANGE 0.2f                                   // m, max front reading used (farther ones are too noisy)
#define NAV_LATERAL_OUTLIER 0.015f                             // m, larger lateral corrections are discarded
#define NAV_LATERAL_RATE 25.0f                                 // 1/s, how fast the lateral error is corrected
#define NAV_FRONT_RATE 25.0f                                   // 1/s, how fast the longitudinal error is corrected
#define NAV_HEADING_GAIN 0.2f                                  // Fraction of the heading error corrected per sample
#define NAV_HEADING_SAMPLE 0.03f                               // m traveled between heading samples
#define NAV_HEADING_FILTER 0.01f                               // m, the side readings are averaged over this distance

// Stall detection

#define NAV_STALL_TIME 0.3f   // s without moving while trying to move
#define NAV_BLOCKED_TIME 0.01f // s a wall must be seen in the way before stopping (single readings are noisy)
#define NAV_NUDGE 0.015f     // m to move forward before retrying a stuck turn
#define NAV_RETRIES_MAX 3    // Stuck turns retried before giving up

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

    // Pending move (NavMove)
    bool move_pending;
    Heading move_heading;
    int move_cells;

    // Current maneuver
    NavMode mode;
    Vector2 target; // Center of the destination cell, while driving
    float velocity;
    float angular_velocity;
    float max_speed = NAV_SPEED_DEFAULT;
    float acceleration = NAV_ACCELERATION_DEFAULT;
    float stall_time;
    float blocked_time;

    // Recovery from stuck turns
    Heading turn_from; // Heading before the current turn
    bool nudge_pending; // Move forward a little after turning back
    int retries;

    // Last side wall measurement, for the heading correction
    bool wall_sample_valid;
    int wall_sample_sides; // 1 = left wall, 2 = right wall, 3 = both
    float wall_sample_lateral;
    float wall_sample_distance;
    float wall_lateral_filtered; // Side wall measurement averaged along the way (the readings are noisy)
    float wall_filter_distance;  // Distance traveled at the last filter update

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

Heading RotateHeading(Heading heading, int quarter_turns_clockwise)
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

static void Stop()
{
    nav.velocity = 0.0f;
    nav.angular_velocity = 0.0f;
}

static void SetLost()
{
    nav.mode = NAV_LOST;
    Stop();
}

// Measures how long the mouse has been stuck; true (once) when it is too long
static bool Stalled(bool stuck)
{
    nav.stall_time = stuck ? nav.stall_time + SIM_TIMESTEP : 0.0f;
    if (nav.stall_time <= NAV_STALL_TIME)
        return false;

    nav.stall_time = 0.0f;
    return true;
}

// Counts a retry after getting stuck; true (and lost) after too many
static bool GiveUp()
{
    if (++nav.retries <= NAV_RETRIES_MAX)
        return false;

    SetLost();
    return true;
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

    // Behind: as seen before if the mouse has been here (it may have turned without moving),
    // else open (the mouse came from there), except at the maze border
    Heading back = RotateHeading(nav.heading, 2);
    Cell cell = nav.cell;
    if (ValidateCell(cell) && nav.seen[cell.x][cell.y])
        walls |= nav.seen_walls[cell.x][cell.y] & HeadingToWall(back);
    else if (!ValidateCell(GetNeighborCell(cell, back)))
        walls |= HeadingToWall(back);

    return walls;
}

// True if the cell was seen before with a different wall in the given direction
static bool Contradicts(Cell cell, Heading heading, bool has_wall)
{
    return ValidateCell(cell) && nav.seen[cell.x][cell.y] &&
           ((nav.seen_walls[cell.x][cell.y] & HeadingToWall(heading)) != 0) != has_wall;
}

static void CheckWalls()
{
    Cell cell = nav.cell;

    if (!ValidateCell(cell))
    {
        SetLost();
        return;
    }

    for (int i = 0; i < 4; i++)
    {
        Heading heading = (Heading)i;
        bool has_wall = nav.walls & HeadingToWall(heading);
        Cell neighbor = GetNeighborCell(cell, heading);

        // The maze border always has walls, and each wall must match what was seen before,
        // from this cell or from the neighbor
        if ((!ValidateCell(neighbor) && !has_wall) || Contradicts(cell, heading, has_wall) ||
            Contradicts(neighbor, RotateHeading(heading, 2), has_wall))
        {
            SetLost();
            return;
        }
    }

    nav.seen[cell.x][cell.y] = true;
    nav.seen_walls[cell.x][cell.y] = nav.walls;
}

static void Arrive(const SimState *state)
{
    nav.mode = NAV_IDLE;
    Stop();

    // Retries count the times the mouse gets stuck without reaching a new cell
    Cell cell = PositionToCell(nav.position);
    if (cell.x != nav.cell.x || cell.y != nav.cell.y)
        nav.retries = 0;

    nav.cell = cell;
    nav.walls = ReadWalls(state);

    CheckWalls();
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
        nav.position = Vector2Add(nav.position, Vector2Scale(normal, NAV_LATERAL_RATE * SIM_TIMESTEP * (measured - lateral)));

        // Heading: how fast the (averaged) distance to the wall changes along the way
        if (!nav.wall_sample_valid || nav.wall_sample_sides != sides)
        {
            nav.wall_sample_valid = true;
            nav.wall_sample_sides = sides;
            nav.wall_lateral_filtered = measured;
            nav.wall_filter_distance = nav.distance_traveled;
            nav.wall_sample_lateral = measured;
            nav.wall_sample_distance = nav.distance_traveled;
        }
        else
        {
            float step = fabsf(nav.distance_traveled - nav.wall_filter_distance);
            nav.wall_lateral_filtered += std::min(step / NAV_HEADING_FILTER, 1.0f) * (measured - nav.wall_lateral_filtered);
            nav.wall_filter_distance = nav.distance_traveled;

            if (nav.distance_traveled - nav.wall_sample_distance >= NAV_HEADING_SAMPLE)
            {
                float slope = (nav.wall_lateral_filtered - nav.wall_sample_lateral) /
                              (nav.distance_traveled - nav.wall_sample_distance);
                float measured_error = atanf(slope);

                if (fabsf(measured_error) < NAV_ALIGNED)
                    nav.rotation += NAV_HEADING_GAIN * (measured_error - heading_error);

                nav.wall_sample_lateral = nav.wall_lateral_filtered;
                nav.wall_sample_distance = nav.distance_traveled;
            }
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
            nav.position = Vector2Add(nav.position, Vector2Scale(axis, NAV_FRONT_RATE * SIM_TIMESTEP * (measured_along - along)));
    }
}

// Motion

// Trapezoidal profile: the fastest velocity that can still stop within the remaining error,
// never changing faster than the acceleration
static float Ramp(float velocity, float error, float max_speed, float min_speed, float acceleration)
{
    float target = std::min(max_speed, sqrtf(2.0f * NAV_BRAKE_MARGIN * acceleration * fabsf(error)));
    target = copysignf(std::max(target, min_speed), error);
    float max_change = acceleration * SIM_TIMESTEP;
    return velocity + std::clamp(target - velocity, -max_change, max_change);
}

static void StartDrive(Vector2 target)
{
    nav.target = target;
    nav.mode = NAV_DRIVING;
    nav.stall_time = 0.0f;
    nav.blocked_time = 0.0f;
    nav.wall_sample_valid = false;
}

static void StartNextMove()
{
    if (!nav.move_pending)
        return;

    // Turn in place first if needed
    if (nav.move_heading != nav.heading)
    {
        nav.turn_from = nav.heading;
        nav.heading = nav.move_heading;
        nav.mode = NAV_TURNING;
        nav.stall_time = 0.0f;
        return;
    }

    nav.move_pending = false;
    if (nav.move_cells == 0)
        return;

    // Drive all the cells as one straight line
    Cell target = nav.cell;
    for (int i = 0; i < nav.move_cells; i++)
        target = GetNeighborCell(target, nav.heading);

    StartDrive(GetCellCenter(target));
}

static void UpdateTurn(const SimState *state)
{
    float error = AngleDiff(nav.rotation, HeadingToRotation(nav.heading));

    if (fabsf(error) < NAV_TURN_DONE && fabsf(state->gyroscope) < 0.2f)
    {
        nav.mode = NAV_IDLE;
        nav.angular_velocity = 0.0f;

        if (nav.nudge_pending)
        {
            // Turned back after getting stuck: move forward a little along the center line,
            // then the move continues and the turn is tried again
            Vector2 axis = Vector2FromAngle(HeadingToRotation(nav.heading));
            Vector2 center = GetCellCenter(nav.cell);
            float along = Vector2DotProduct(Vector2Subtract(nav.position, center), axis);

            nav.nudge_pending = false;
            StartDrive(Vector2Add(center, Vector2Scale(axis, along + NAV_NUDGE)));
        }
        else
            nav.retries = 0;

        return;
    }

    nav.velocity = 0.0f;
    nav.angular_velocity = Ramp(nav.angular_velocity, error, NAV_TURN_SPEED_MAX, 0.0f, NAV_TURN_ACCELERATION);

    // Stalled: trying to turn, but the gyroscope says it does not
    if (Stalled(fabsf(nav.angular_velocity) > 0.5f && fabsf(state->gyroscope) < 0.05f))
    {
        if (GiveUp())
            return;

        // Turn back to where the turn started, then move forward a little and retry
        nav.heading = nav.turn_from;
        nav.nudge_pending = true;
        nav.angular_velocity = 0.0f;
    }
}

static void UpdateDrive(const SimState *state)
{
    Vector2 axis = Vector2FromAngle(HeadingToRotation(nav.heading));
    Vector2 normal = {-axis.y, axis.x}; // Points left

    CorrectWithWalls(state, axis, normal);

    Vector2 offset = Vector2Subtract(nav.position, nav.target);
    float remaining = -Vector2DotProduct(offset, axis);
    float lateral = Vector2DotProduct(offset, normal);
    float heading_error = AngleDiff(HeadingToRotation(nav.heading), nav.rotation);

    // A wall blocks the way: stop at the center of the cell before it
    float front = state->ir_sensors[IR_SENSOR_FRONT];
    if (fabsf(heading_error) < NAV_ALIGNED && front < NAV_FRONT_RANGE &&
        front < remaining + NAV_WALL_DISTANCE - 0.25f * CELL_SIZE)
        nav.blocked_time += SIM_TIMESTEP;
    else
        nav.blocked_time = 0.0f;

    if (nav.blocked_time >= NAV_BLOCKED_TIME)
    {
        Vector2 stop = Vector2Add(nav.position, Vector2Scale(axis, front - NAV_WALL_DISTANCE));
        nav.target = GetCellCenter(PositionToCell(stop));
        nav.move_pending = false;

        offset = Vector2Subtract(nav.position, nav.target);
        remaining = -Vector2DotProduct(offset, axis);
    }

    if (fabsf(remaining) < NAV_DRIVE_DONE && fabsf(nav.velocity) <= NAV_SPEED_MIN)
    {
        Arrive(state);
        return;
    }

    // The speed never changes faster than the acceleration, even when the estimate jumps:
    // braking harder would make the wheels slip. If the mouse overshoots, it backs up.
    nav.velocity = Ramp(nav.velocity, remaining, nav.max_speed, NAV_SPEED_MIN, nav.acceleration);

    // Steer towards the center line (reversed when backing up)
    float heading_target = -std::clamp(NAV_STEER_LATERAL * lateral, -NAV_STEER_HEADING_MAX, NAV_STEER_HEADING_MAX);
    if (nav.velocity < 0.0f)
        heading_target = -heading_target;
    nav.angular_velocity = NAV_STEER_GAIN * (heading_target - heading_error);

    // Stalled: trying to drive, but the encoders say it does not move
    // Stuck: end the move and back up to the center of the cell behind the mouse
    // (walls can only be read from a cell center). The planner then decides what to do.
    float encoder_speed = fabsf(GetEncoderDistance(state) - nav.encoder_distance_last) / SIM_TIMESTEP;
    if (Stalled(fabsf(nav.velocity) > 2.0f * NAV_SPEED_MIN && encoder_speed < 0.01f))
    {
        if (GiveUp())
            return;

        Cell cell = PositionToCell(nav.position);
        Vector2 center = GetCellCenter(cell);
        if (Vector2DotProduct(Vector2Subtract(nav.position, center), axis) < 0.0f)
            center = Vector2Subtract(center, Vector2Scale(axis, CELL_SIZE));

        nav.target = center;
        nav.move_pending = false;
        nav.blocked_time = 0.0f;
    }
}

// Public API

void NavReset(Sim *sim)
{
    const SimState *state = GetSimState(sim);

    // Start from scratch, except the speed (it persists across resets)
    float max_speed = nav.max_speed;
    float acceleration = nav.acceleration;
    nav = {};
    nav.max_speed = max_speed;
    nav.acceleration = acceleration;

    nav.position = GetCellCenter({0, 0});
    nav.rotation = ROTATION_NORTH;
    nav.encoder_distance_last = GetEncoderDistance(state);
    nav.cell = {0, 0};
    nav.heading = HEADING_NORTH;
    nav.mode = NAV_IDLE;

    nav.walls = ReadWalls(state);
    CheckWalls();

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
        UpdateTurn(state);
        break;

    case NAV_DRIVING:
        UpdateDrive(state);
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
    return nav.mode == NAV_IDLE && !nav.move_pending;
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

void NavMove(Heading heading, int cells)
{
    if (!NavIsIdle())
        return;

    nav.move_pending = true;
    nav.move_heading = heading;
    nav.move_cells = std::clamp(cells, 0, GRID_SIZE - 1);
}

void NavSetSpeed(float max_speed, float acceleration)
{
    nav.max_speed = std::clamp(max_speed, NAV_SPEED_MIN, MOUSE_WHEEL_VELOCITY_MAX);
    nav.acceleration = std::max(acceleration, 0.1f);
}
