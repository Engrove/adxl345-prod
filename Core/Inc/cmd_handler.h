/* filename: Core/Inc/cmd_handler.h */
#ifndef COMMAND_HANDLER_H
#define COMMAND_HANDLER_H

#include "app_context.h"

/**
 * @brief Initializes the command handler module.
 * @param ctx Pointer to the application context.
 */
void CmdHandler_Init(AppContext_t* ctx);

/**
 * @brief Processes the incoming command queue with a time and line budget.
 *        This function should be called repeatedly in the main application loop.
 * @param ctx Pointer to the application context.
 */
void CmdHandler_ProcessInput(AppContext_t* ctx);

/**
 * @brief Handles the global stop flag logic, ensuring a clean shutdown of active processes.
 * @param ctx Pointer to the application context.
 */
void CmdHandler_HandleStop(AppContext_t* ctx);

#endif // COMMAND_HANDLER_H