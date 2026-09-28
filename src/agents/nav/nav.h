/**
 * Teseo Micromouse Virtual Competition
 * Navigation layer
 *
 * @brief Moves the mouse from cell to cell and keeps track of where it is.
 *
 *        This layer does the "magic" for you: it estimates the mouse's position with the
 *        sensors (wheel encoders, gyroscope and IR sensors) and corrects its errors by
 *        itself. You only work with cells, headings and walls.
 *
 *        But everything it tells you is what the layer BELIEVES, not the truth.
 *        At the default speed the belief is very reliable; at higher speeds the wheels
 *        may slip and the mouse may get lost (NavIsLost()).
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
 * @brief Returns true when the mouse has finished the move and is stopped at the center of a cell.
 */
bool NavIsIdle();

/**
 * @brief Returns true if the layer noticed that its belief is wrong (e.g. it sees walls where it
 *        saw none before). The mouse stops: press [R] to restart the run from the start cell.
 */
bool NavIsLost();

/**
 * @brief Returns the cell where the layer believes the mouse is. While moving, it is the cell
 *        where the mouse last stopped: it is updated when the mouse stops again.
 */
Cell NavGetCell();

/**
 * @brief Returns the direction the layer believes the mouse is facing (updated like NavGetCell()).
 */
Heading NavGetHeading();

/**
 * @brief Returns the walls of the current cell (WALL_NORTH | WALL_EAST | ...), as seen by the
 *        IR sensors when the mouse stopped there. Only valid while NavIsIdle() is true.
 *        Walls are only read where the mouse stops: the cells it drives through without
 *        stopping are not read.
 */
uint8_t NavGetWalls();

/**
 * @brief Turns in place to face the given direction (if needed), then drives the given number
 *        of cells straight ahead, as one fast straight line. With 0 cells, it only turns.
 *        If a wall blocks the way (or the mouse gets stuck), the move ends early: compare
 *        NavGetCell() with the destination to find out. Ignored unless NavIsIdle() is true:
 *        wait until the mouse stops before giving a new move.
 *
 * @param heading The direction to move in.
 * @param cells The number of cells to drive.
 */
void NavMove(Heading heading, int cells);

/**
 * @brief Sets the maximum speed and acceleration on straight lines. Faster is riskier:
 *        with too much acceleration the wheels slip and the mouse misjudges its position.
 *
 * @param max_speed The maximum speed (m/s), NAV_SPEED_DEFAULT by default.
 * @param acceleration The acceleration and braking (m/s²), NAV_ACCELERATION_DEFAULT by default.
 */
void NavSetSpeed(float max_speed, float acceleration);

#endif // NAV_H
