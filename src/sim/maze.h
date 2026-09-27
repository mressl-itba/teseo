/**
 * Teseo Micromouse Virtual Competition
 * Maze generation module
 *
 * @details Generates and loads maze layouts for the Teseo simulation.
 * @author Theseús the hero
 */

#ifndef MAZE_H
#define MAZE_H

#include <cstdint>

// Maze geometry

#define GRID_SIZE 16

/**
 * @brief Cell coordinate (x = column, y = row), (0, 0) is southwest corner.
 */
struct Cell
{
    int32_t x;
    int32_t y;
};

// Wall bitmask flags for each cell

#define WALL_NORTH 0x1
#define WALL_EAST 0x2
#define WALL_SOUTH 0x4
#define WALL_WEST 0x8

// Colors

#define COLOR_CELL_DEFAULT 0x161a26
#define COLOR_CELL_VISITED 0x2c344c
#define COLOR_CELL_RED 0x7a1d24
#define COLOR_CELL_GREEN 0x3a7a1d
#define COLOR_CELL_BLUE 0x1d377a

/**
 * @brief Maze data structure: each cell has a 4-bit wall bitmask (WALL_NORTH, EAST, SOUTH, WEST)
 */
struct Maze
{
    uint8_t walls[GRID_SIZE][GRID_SIZE];
};

/**
 * @brief Generates a random perfect maze using recursive backtracker.
 *
 * @param seed Random seed (same seed = same maze).
 *
 * @return A pointer to the generated Maze.
 */
Maze *GenerateMaze(uint32_t seed);

/**
 * @brief Loads a maze from a file.
 * 
 * @param filename Path to the maze file.
 * 
 * @return A pointer to the loaded Maze, or nullptr if the file cannot be read or is malformed.
 */
Maze *LoadMaze(const char *filename);

/**
 * @brief Destroys a maze instance and frees its memory.
 *
 * @param maze Pointer to the Maze instance to destroy.
 */
void DestroyMaze(Maze *maze);

/**
 * @brief Returns true if the given cell coordinate is valid within the maze grid.
 * 
 * @param cell The cell coordinate to validate.
 * 
 * @return true if the cell is valid, false otherwise.
 */
bool ValidateCell(Cell cell);

/**
 * @brief Returns true if the given wall is present on cell (x, y).
 *        Out-of-bounds cells always report all walls present.
 *
 * @param maze The maze instance.
 * @param cell The cell coordinate.
 * @param wall_bit One of WALL_NORTH, WALL_EAST, WALL_SOUTH, WALL_WEST.
 * 
 * @return true if the wall is present, false if it's open.
 */
bool HasWall(const Maze *maze, Cell cell, uint8_t wall_bit);

/**
 * @brief Returns true if the given cell is the starting cell (0, 0).
 * 
 * @param cell The cell coordinate.
 * 
 * @return true if the cell is the start cell, false otherwise.
 */
bool IsStartCell(Cell cell);

/**
 * @brief Returns true if the given cell is part of the goal area.
 * 
 * @param cell The cell coordinate.
 * 
 * @return true if the cell is a goal cell, false otherwise.
 */
bool IsGoalCell(Cell cell);

#endif // MAZE_H
