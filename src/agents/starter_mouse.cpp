/**
 * Teseo Micromouse Virtual Competition
 * Starter mouse
 *
 * @details Right-hand wall-follower: at each cell, go right if possible, else straight,
 *          else left, else back. Simple, but slow and it can loop forever in some mazes.
 * @author Theseús the hero
 */

// IMPORTANT: COPY THIS FILE WITH A NEW NAME TO START YOUR OWN MOUSE

#include "nav/nav.h"
#include "sim/mouse.h"

const char *GetMouseName()
{
    return "Starter Mouse";
}

void ResetMouse(Sim *sim)
{
    NavReset(sim);
    ResetCellColors(sim);
}

void UpdateMouse(Sim *sim)
{
    NavUpdate(sim);

    // Wait until the mouse stops at a cell
    if (!NavIsIdle() || NavIsLost())
        return;

    Cell cell = NavGetCell();
    Heading heading = NavGetHeading();
    uint8_t walls = NavGetWalls();

    PaintCell(sim, cell, COLOR_CELL_VISITED);

    // Headings are in clockwise order: +1 is right, +3 is left, +2 is back
    Heading right = (Heading)((heading + 1) % 4);
    Heading left = (Heading)((heading + 3) % 4);
    Heading back = (Heading)((heading + 2) % 4);

    Heading next;
    if (!(walls & HeadingToWall(right)))
        next = right;
    else if (!(walls & HeadingToWall(heading)))
        next = heading;
    else if (!(walls & HeadingToWall(left)))
        next = left;
    else
        next = back;

    NavMove(next, 1);
}
