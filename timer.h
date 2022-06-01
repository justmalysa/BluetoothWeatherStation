#ifndef TIMER_H
#define TIMER_H

#include <stdbool.h>

typedef void (*timer_callback_t)(void);

/* Function prototypes ------------------------------------------------------- */

/** Initializes the TIM interface.
 *  @return true if the initialization was successfull and the port is operational,
 *          false otherwise
 */
bool TIM_Init(timer_callback_t callback);

#endif /* TIMER_H */
