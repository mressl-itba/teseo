/**
 * Teseo Micromouse Virtual Competition
 * Maze generation module
 *
 * @details Generates and loads maze layouts for the Teseo simulation.
 * @author Theseús the hero
 */

#include <cstdio>
#include <cstring>
#include <random>
#include <stack>

#include "maze.h"

// Maze creation and destruction

Maze *GenerateMaze(uint32_t seed)
{
    Maze *maze = new Maze();

    // All walls closed
    for (int32_t x = 0; x < GRID_SIZE; x++)
        for (int32_t y = 0; y < GRID_SIZE; y++)
            maze->walls[x][y] = WALL_NORTH | WALL_EAST | WALL_SOUTH | WALL_WEST;

    // Recursive backtracker (iterative via explicit stack)
    std::mt19937 rng(seed);
    bool visited[GRID_SIZE][GRID_SIZE] = {};

    std::stack<Cell> stack;

    stack.push({0, 0});
    visited[0][0] = true;

    // Direction: North, East, South, West
    const int32_t dx[] = {0, 1, 0, -1};
    const int32_t dy[] = {1, 0, -1, 0};
    const uint8_t wall_fwd[] = {WALL_NORTH, WALL_EAST, WALL_SOUTH, WALL_WEST};
    const uint8_t wall_bwd[] = {WALL_SOUTH, WALL_WEST, WALL_NORTH, WALL_EAST};

    while (!stack.empty())
    {
        Cell cur = stack.top();

        int32_t neighbors[4] = {};
        int32_t ncount = 0;
        for (int32_t d = 0; d < 4; d++)
        {
            int32_t nx = cur.x + dx[d], ny = cur.y + dy[d];
            if (nx >= 0 && nx < GRID_SIZE && ny >= 0 && ny < GRID_SIZE && !visited[nx][ny])
                neighbors[ncount++] = d;
        }

        if (ncount == 0)
        {
            stack.pop();

            continue;
        }

        int32_t d = neighbors[std::uniform_int_distribution<>(0, ncount - 1)(rng)];
        int32_t nx = cur.x + dx[d], ny = cur.y + dy[d];

        // Carve passage
        maze->walls[cur.x][cur.y] &= ~wall_fwd[d];
        maze->walls[nx][ny] &= ~wall_bwd[d];

        visited[nx][ny] = true;
        stack.push({nx, ny});
    }

    // Open the interior of the goal 2x2 area so the mouse can traverse freely
    int32_t gx = GRID_SIZE / 2 - 1;
    int32_t gy = GRID_SIZE / 2 - 1;

    maze->walls[gx][gy] &= ~(WALL_NORTH | WALL_EAST);
    maze->walls[gx + 1][gy] &= ~(WALL_NORTH | WALL_WEST);
    maze->walls[gx][gy + 1] &= ~(WALL_SOUTH | WALL_EAST);
    maze->walls[gx + 1][gy + 1] &= ~(WALL_SOUTH | WALL_WEST);

    return maze;
}

Maze *LoadMaze(const char *filename)
{
    FILE *f = fopen(filename, "r");
    if (!f)
        return nullptr;

    Maze *maze = new Maze();
    for (int x = 0; x < GRID_SIZE; x++)
        for (int y = 0; y < GRID_SIZE; y++)
            maze->walls[x][y] = 0;

    char line[128];
    int file_row = 0;
    int cell_width = 0;

    while (fgets(line, sizeof(line), f) && file_row <= 2 * GRID_SIZE)
    {
        int len = (int)strlen(line);
        while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r'))
            line[--len] = '\0';

        // Detect cell width from the first line: 2 (compact) or 4 (wide).
        if (file_row == 0)
        {
            cell_width = (len - 1) / GRID_SIZE;
            if (cell_width != 2 && cell_width != 4)
                break;
        }

        if (file_row % 2 == 0)
        {
            // Post row: check middle character between each pair of posts for horizontal wall.
            int p = file_row / 2;
            for (int x = 0; x < GRID_SIZE; x++)
            {
                int pos = cell_width * x + cell_width / 2;
                if (pos < len && line[pos] != ' ')
                {
                    if (p < GRID_SIZE)
                        maze->walls[x][GRID_SIZE - 1 - p] |= WALL_NORTH;
                    if (p > 0)
                        maze->walls[x][GRID_SIZE - p] |= WALL_SOUTH;
                }
            }
        }
        else
        {
            // Cell row: check boundary characters for vertical walls.
            int y = GRID_SIZE - 1 - (file_row / 2);
            for (int x = 0; x <= GRID_SIZE; x++)
            {
                int pos = cell_width * x;
                if (pos < len && line[pos] != ' ')
                {
                    if (x < GRID_SIZE)
                        maze->walls[x][y] |= WALL_WEST;
                    if (x > 0)
                        maze->walls[x - 1][y] |= WALL_EAST;
                }
            }
        }

        file_row++;
    }

    fclose(f);

    // Reject malformed files: unknown format or missing rows.
    if (file_row != 2 * GRID_SIZE + 1)
    {
        delete maze;

        return nullptr;
    }

    // The maze border is always closed, even if the file forgets a wall
    for (int i = 0; i < GRID_SIZE; i++)
    {
        maze->walls[i][0] |= WALL_SOUTH;
        maze->walls[i][GRID_SIZE - 1] |= WALL_NORTH;
        maze->walls[0][i] |= WALL_WEST;
        maze->walls[GRID_SIZE - 1][i] |= WALL_EAST;
    }

    return maze;
}

void DestroyMaze(Maze *maze)
{
    delete maze;
}

// Maze query functions

bool ValidateCell(Cell cell)
{
    return cell.x >= 0 && cell.x < GRID_SIZE && cell.y >= 0 && cell.y < GRID_SIZE;
}

bool HasWall(const Maze *maze, Cell cell, uint8_t wall_bit)
{
    if (cell.x < 0 || cell.x >= GRID_SIZE || cell.y < 0 || cell.y >= GRID_SIZE)
        return true;

    return (maze->walls[cell.x][cell.y] & wall_bit) != 0;
}

bool IsStartCell(Cell cell)
{
    return cell.x == 0 && cell.y == 0;
}

bool IsGoalCell(Cell cell)
{
    int32_t gx = GRID_SIZE / 2 - 1;
    int32_t gy = GRID_SIZE / 2 - 1;

    return (cell.x == gx || cell.x == gx + 1) && (cell.y == gy || cell.y == gy + 1);
}
