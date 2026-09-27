/**
 * Teseo Micromouse Virtual Competition
 * Navigation layer
 *
 * @brief Moves the mouse from cell to cell and keeps track of where it is.
 *
 *        The mouse estimates its own position with its sensors (wheel encoders,
 *        gyroscope and IR sensors) and corrects its errors by itself.
 *        NavGetCell() tells where the mouse BELIEVES it is: at the default speed
 *        it is always right, but at higher speeds it may get lost.
 *
 *        You can use this layer as is, or modify it to make the mouse faster.
 * @author Theseús the hero
 */

#ifndef NAV_H
#define NAV_H

#include <cstdint>

#include "sim/sim.h"

/**
 * @brief Absolute directions, in clockwise order.
 */
enum Heading
{
    HEADING_NORTH,
    HEADING_EAST,
    HEADING_SOUTH,
    HEADING_WEST,
};

#define NAV_PATH_MAX (GRID_SIZE * GRID_SIZE) // Maximum number of moves in a path

#define NAV_SPEED_DEFAULT 0.5f        // m/s
#define NAV_ACCELERATION_DEFAULT 2.0f // m/s²

/**
 * @brief Returns the wall bit (WALL_NORTH, WALL_EAST, ...) in the given direction.
 */
uint8_t HeadingToWall(Heading heading);

/**
 * @brief Returns the neighboring cell in the given direction.
 */
Cell GetNeighborCell(Cell cell, Heading heading);

/**
 * @brief Resets the navigation layer: the mouse is at the start cell, facing north.
 *        Call it from ResetMouse().
 *
 * @param sim The simulation instance.
 */
void NavReset(Sim *sim);

/**
 * @brief Updates the position estimate and drives the motors. Call it at the start of UpdateMouse().
 *
 * @param sim The simulation instance.
 */
void NavUpdate(Sim *sim);

/**
 * @brief Returns true when the mouse has finished the path and is stopped at the center of a cell.
 */
bool NavIsIdle();

/**
 * @brief Returns true if the mouse noticed that its belief is wrong (e.g. it sees walls where it
 *        saw none before). The mouse stops: press [R] to restart the run from the start cell.
 */
bool NavIsLost();

/**
 * @brief Returns the cell where the mouse believes it is.
 */
Cell NavGetCell();

/**
 * @brief Returns the direction the mouse believes it is facing.
 */
Heading NavGetHeading();

/**
 * @brief Returns the walls of the current cell (WALL_NORTH | WALL_EAST | ...), as seen by the
 *        IR sensors when the mouse stopped there. Only valid while NavIsIdle() is true.
 */
uint8_t NavGetWalls();

/**
 * @brief Moves the mouse along a path: each element is the direction of the next cell.
 *        Consecutive moves in the same direction are driven as one fast straight line.
 *        If a wall blocks the path, the mouse stops at the last reachable cell.
 *
 * @param path The directions of the moves.
 * @param count The number of moves (at most NAV_PATH_MAX).
 */
void NavFollowPath(const Heading *path, int count);

/**
 * @brief Sets the maximum speed and acceleration on straight lines. Faster is riskier:
 *        with too much acceleration the wheels slip and the mouse misjudges its position.
 *
 * @param max_speed The maximum speed (m/s), NAV_SPEED_DEFAULT by default.
 * @param acceleration The acceleration and braking (m/s²), NAV_ACCELERATION_DEFAULT by default.
 */
void NavSetSpeed(float max_speed, float acceleration);

#endif // NAV_H
