/**
 * Teseo Micromouse Virtual Competition
 * Mouse interface
 *
 * @brief Functions that every mouse must define. Each .cpp file in src/agents/
 *        builds its own executable, so there is exactly one mouse per program.
 * @author Theseús the hero
 */

#ifndef MOUSE_H
#define MOUSE_H

#include "sim.h"

/**
 * @brief Returns the name of the mouse, shown in the UI.
 */
const char *GetMouseName();

/**
 * @brief Called when a run is started with [R], with the mouse back at the start cell.
 *        Keep what the mouse learned (e.g. the maze map) in global variables: they survive resets.
 *
 * @param sim The simulation instance.
 */
void ResetMouse(Sim *sim);

/**
 * @brief Called every simulation step (every SIM_TIMESTEP seconds) while the simulation is running.
 *
 * @param sim The simulation instance.
 */
void UpdateMouse(Sim *sim);

#endif // MOUSE_H
