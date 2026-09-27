/**
 * Teseo Micromouse Virtual Competition
 * Physics simulation module
 *
 * @brief Implements the physics simulation using Box2D.
 * @author Theseús the hero
 */

#ifndef SIM_H
#define SIM_H

#include <cstdint>

#include <raymath.h>

#include "maze.h"

// Rotations

#define PI 3.14159265358979323846f

#define ROTATION_EAST (0.0f * PI)
#define ROTATION_NORTH (0.5f * PI)
#define ROTATION_WEST (1.0f * PI)
#define ROTATION_SOUTH (1.5f * PI)

#define TURN_CCW (0.5f * PI) // +90° turn in radians (left)
#define TURN_CW (-0.5f * PI) // -90° turn in radians (right)
#define TURN_REVERSE PI      // +180° turn in radians

// Maze geometry

#define CELL_SIZE 0.18f       // m
#define WALL_THICKNESS 0.012f // m

#define MAZE_SIZE (GRID_SIZE * CELL_SIZE) // m

#define CELL_HALF_SIZE (CELL_SIZE / 2.0f)
#define WALL_HALF_THICKNESS (WALL_THICKNESS / 2.0f)
#define WALL_HALF_WIDTH (CELL_SIZE / 2.0f + WALL_HALF_THICKNESS)
#define WALL_HALF_HEIGHT (CELL_SIZE / 2.0f + WALL_HALF_THICKNESS)

// Mouse geometry and physics

#define MOUSE_WIDTH 0.080f       // m
#define MOUSE_LENGTH 0.100f      // m
#define MOUSE_MASS 0.100f        // kg
#define MOUSE_WHEEL_TRACK 0.070f // m (axle width)

#define MOUSE_HALF_WIDTH (MOUSE_WIDTH / 2.0f)
#define MOUSE_HALF_LENGTH (MOUSE_LENGTH / 2.0f)
#define MOUSE_DENSITY (MOUSE_MASS / (MOUSE_WIDTH * MOUSE_LENGTH))
#define MOUSE_WHEEL_HALF_TRACK (MOUSE_WHEEL_TRACK / 2.0f)

// Wheel drive

#define MOUSE_WHEEL_VELOCITY_MAX 1.5f                                    // m/s
#define MOUSE_WHEEL_FORCE_MAX 0.5f                                       // N
#define MOUSE_MOTOR_K (MOUSE_WHEEL_FORCE_MAX / MOUSE_WHEEL_VELOCITY_MAX) // N·s/m (k_t·k_e / R·r²)

// Odometry error
// The mouse controller measures distance with the wheel encoders and heading with the
// integrated gyroscope, so their errors make the mouse drift away from the commanded path.

#define ENCODER_SCALE_ERROR 0.007f                  // Stddev of the wheels' common scale error, fixed per mouse (≈7 mm/m)
#define ENCODER_MISMATCH_ERROR 0.001f               // Stddev of each wheel's additional scale error, fixed per mouse
#define ENCODER_SLIP_NOISE 0.01f                    // Relative stddev of each step's wheel displacement (wheel slip)
#define ODOMETRY_ROTATION_ERROR 0.01f               // Relative stddev of each commanded rotation (≈1° per 90°)
#define GYROSCOPE_BIAS (0.05f * PI / 180.0f)        // Stddev of residual gyroscope bias, drawn each run (rad/s)
#define GYROSCOPE_BIAS_WALK (0.005f * PI / 180.0f)  // Gyroscope bias random walk (rad/s per √s)

// Wheel controller

#define MOUSE_KP_DISTANCE 8.0f  // m/s per m (distance proportional gain)
#define MOUSE_KD_DISTANCE 1.5f  // m/s per m/s (velocity damping)
#define MOUSE_KP_ROTATION 1.0f  // m/s per rad (rotation proportional gain)
#define MOUSE_KD_ROTATION 0.09f // m/s per rad/s (angular velocity damping)

// Mouse sensors (5 IR sensors: left, front-left, front, front-right, right)

#define IR_SENSOR_NUM 5
#define IR_SENSOR_RANGE_MIN MOUSE_HALF_WIDTH // m
#define IR_SENSOR_RANGE_MAX 1.0f             // m

enum
{
    IR_SENSOR_LEFT,
    IR_SENSOR_FRONT_LEFT,
    IR_SENSOR_FRONT,
    IR_SENSOR_FRONT_RIGHT,
    IR_SENSOR_RIGHT,
};

extern const float IR_SENSOR_ANGLES[IR_SENSOR_NUM];

// Wheel encoders

#define ENCODER_NUM 2

enum
{
    ENCODER_LEFT,
    ENCODER_RIGHT,
};

// Sim state

#define RUN_TOTAL 5         // Total number of runs to execute
#define RUN_TIME_MAX 300.0f // Seconds

enum RunState
{
    RUNSTATE_IDLE,
    RUNSTATE_RUNNING,
    RUNSTATE_RETURNING,
};

/**
 * @brief The state of the simulation at a given moment, used for rendering and by the mouse agent.
 */
struct SimState
{
    float time;          // Elapsed time since simulation start (seconds).
    int run_number;      // Run counter (starts at 1, increments on reset).
    RunState run_state;  // Current run state (idle, running, returning).
    float run_time;      // Elapsed time since start of run (seconds).
    float run_time_best; // Best time achieved so far (seconds).

    Vector2 accelerometer; // Linear acceleration in body frame (m/s²): y=forward, x=right
    float gyroscope;       // Angular velocity (rad/s, CCW+), includes a slowly drifting bias

    float encoders[ENCODER_NUM]; // Distance traveled by each wheel since the last reset (meters, positive = forward)

    float setpoint_distance; // Distance remaining to reach setpoint (meters, positive = forward)
    float setpoint_rotation; // Rotation remaining to reach setpoint (radians, CCW+)

    float ir_sensors[IR_SENSOR_NUM]; // Distance readings from the 5 IR sensors (meters)
};

// Opaque handle

struct Sim;

// -----------------------------------------------------------------------------
// WARNING: Only the following functions are available to mouse agents.
// Do not call any other functions from the simulation API.
// Doing so will disqualify yout team from the competition.

/**
 * @brief Creates a vector from an angle in radians (CCW+).
 *
 * @param angle The angle in radians (CCW+).
 * @param length Optional length of the resulting vector (default is 1.0f).
 *
 * @return A vector pointing in the direction of the angle.
 */
Vector2 Vector2FromAngle(float angle, float length = 1.0f);

/**
 * @brief Computes the difference between two angles, returning the smallest signed angle between them.
 *
 * @param source The source angle (radians).
 * @param target The target angle (radians).
 *
 * @return The angle difference (radians, in the range [-π, π], positive = CCW).
 */
float AngleDiff(float source, float target);

/**
 * @brief Converts a world position (meters) to a maze cell coordinate.
 *
 * @param position The world position to convert (meters, north/east coordinates).
 *
 * @return The corresponding maze cell coordinate.
 */
Cell PositionToCell(Vector2 position);

/**
 * @brief Paints the given maze cell with the specified color. Used for debugging and visualization.
 *
 * @param sim The simulation instance.
 * @param cell The cell coordinate to paint.
 * @param color The color to use for painting.
 */
void PaintCell(Sim *sim, Cell cell, uint32_t color);

/**
 * @brief Resets the colors of all maze cells to the default color. Used for debugging and visualization.
 *
 * @param sim The simulation instance.
 */
void ResetCellColors(Sim *sim);

/**
 * @brief Gets the color associated with the given maze cell, based on its state (e.g. discovered, goal).
 *
 * @param sim The simulation instance.
 * @param cell The cell coordinate to query.
 *
 * @return The color associated with the maze cell.
 */
uint32_t GetCellColor(Sim *sim, Cell cell);

/**
 * @brief Returns the current state of the simulation.
 *
 * @param sim The simulation instance to query.
 *
 * @return A pointer to the current simulation state.
 */
const SimState *GetSimState(Sim *sim);

/**
 * @brief Sets a movement setpoint for the mouse.
 *        The mouse will attempt to rotate to the target rotation and drive to the target distance.
 *
 * @param sim The simulation instance.
 * @param distance The distance setpoint in meters (positive = forward).
 * @param rotation The rotation setpoint in radians (CCW+).
 */
void SetMouseSetpoint(Sim *sim, float distance, float rotation);

/**
 * @brief Reports the agent's estimate of the mouse pose. The UI draws it as an outline
 *        next to the real mouse, and shows the estimation error. Used for debugging.
 *
 * @param sim The simulation instance.
 * @param position The estimated position (meters, north/east coordinates).
 * @param rotation The estimated rotation (radians, CCW+, 0 = East).
 */
void SetEstimatedPose(Sim *sim, Vector2 position, float rotation);

// This is the end of the agent API.
// The following functions are used by the UI and must not be called by mouse agents.
// -----------------------------------------------------------------------------

// Simulation API

/**
 * @brief Creates a new Teseo simulation instance with the given maze.
 *
 * @param maze The maze layout to use for the simulation.
 *
 * @return A pointer to the created simulation instance.
 */
Sim *CreateSim(const Maze *maze);

/**
 * @brief Frees all resources associated with the simulation instance.
 *
 * @param sim The simulation instance to destroy.
 */
void DestroySim(Sim *sim);

// Simulation

/**
 * @brief Checks if the simulation is currently running (i.e. a run is ongoing and not yet completed).
 *
 * @return true if the simulation is running, false otherwise.
 */
bool IsSimRunning(Sim *sim);

/**
 * @brief Gets the current position of the mouse (meters, north/east).
 *
 * @param sim The simulation instance to query.
 *
 * @return The current position of the mouse in world coordinates.
 */
Vector2 GetMousePosition(Sim *sim);

/**
 * @brief Gets the current rotation of the mouse in radians (CCW+).
 *
 * @param sim The simulation instance to query.
 *
 * @return The current rotation of the mouse in radians.
 */
float GetMouseRotation(Sim *sim);

/**
 * @brief Gets the pose estimate last reported by the agent with SetEstimatedPose().
 *
 * @param sim The simulation instance to query.
 * @param position Output: the estimated position.
 * @param rotation Output: the estimated rotation.
 *
 * @return true if the agent reported an estimate since the last reset, false otherwise.
 */
bool GetEstimatedPose(Sim *sim, Vector2 *position, float *rotation);

/**
 * @brief Starts a new run and resets the mouse.
 *
 * @param sim The simulation instance to reset.
 *
 * @return true if the reset was successful and a new run has started,
 *         false if the maximum number of runs has been reached.
 */
bool ResetSim(Sim *sim);

/**
 * @brief Steps the physics simulation forward by dt seconds.
 *
 * @param sim The simulation instance to step.
 * @param dt Time step in seconds.
 */
void UpdateSim(Sim *sim, float dt);

#endif // SIM_H
