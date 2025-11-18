/* filename: Core/Inc/dev_diagnostics.h */
#ifndef DEV_DIAGNOSTICS_H
#define DEV_DIAGNOSTICS_H

#include "main.h"
#include "app_context.h"
#include "comm.h" // För COMM_SendfLine

// --- Makro för att skicka diagnostikresultat till konsolen ---
// FIX: Korrigerad syntax för att undvika stray '\' i program
#define DIAG_SEND_RESULT(name, desc, value, pass) \
    COMM_SendfLine("DIAG_RES,test=%s,desc=\"%s\",val=%s,pass=%s", name, desc, value, pass ? "PASS" : "FAIL")

/**
 * @brief Huvudfunktion för att köra diagnostiktesterna.
 * @note Denna funktion är avsedd att anropas av kommandohanteraren.
 * @param ctx Pekare till applikationskontexten.
 */
void DevDiag_RunAllTests(AppContext_t* ctx);

#endif // DEV_DIAGNOSTICS_H