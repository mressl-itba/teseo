/**
 * Teseo Micromouse Virtual Competition
 * Agent interface
 *
 * @details Defines the agent interface for mouse implementations.
 * @author Theseús the hero
 */

#ifndef AGENT_H
#define AGENT_H

#include <cstdint>

#include "sim.h"

/**
 * @brief Function pointer type for creating a mouse agent.
 *
 * @return A pointer to the mouse agent's instance.
 */
typedef void *(*MouseCreateCallback)();

/**
 * @brief Function pointer type for destroying a mouse agent.
 *
 * @param info A pointer to the mouse agent's instance that was returned by the create function.
 */
typedef void (*MouseDestroyCallback)(void *userdata);

/**
 * @brief Function pointer type for updating a mouse agent.
 *        This function will be called by the simulation engine each time step.
 *
 * @param userdata Pointer returned by CreateMouse.
 * @param info Current sensor readings.
 */
typedef void (*MouseUpdateCallback)(void *userdata, Sim *sim);

/**
 * @brief Function pointer type for resetting a mouse agent.
 *        This function will be called by the simulation engine
 *        when the mouse is reset manually to the initial position.
 * 
 * @param userdata Pointer returned by CreateMouse.
 * @param sim The simulation instance.
 */
typedef void (*MouseResetCallback)(void *userdata, Sim *sim);

/**
 * @brief Describes a mouse implementation. Define one of these as a global
 *        in your .cpp file and register it in main.cpp.
 */
struct MouseDescriptor
{
    const char *name;
    MouseCreateCallback create;
    MouseDestroyCallback destroy;
    MouseUpdateCallback update;
    MouseResetCallback reset;
};

/**
 * @brief Mouse agent instance, created from a MouseDescriptor.
 */
struct Mouse
{
    MouseDescriptor *descriptor;
    void *userdata;
};

/**
 * @brief Creates a new mouse agent instance using the provided descriptor.
 * 
 * @param descriptor The MouseDescriptor defining the agent's behavior and lifecycle functions.
 * 
 * @return A pointer to the created Mouse instance.
 */
Mouse *CreateMouse(MouseDescriptor &descriptor);

/**
 * @brief Destroys a mouse agent instance, calling its destructor and freeing resources.
 * 
 * @param instance The mouse instance to destroy.
 */
void DestroyMouse(Mouse *mouse);

#endif // AGENT_H
