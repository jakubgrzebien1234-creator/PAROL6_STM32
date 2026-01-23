#ifndef HOMING_H
#define HOMING_H

#include "main.h"
#include "tmc5160.h"

typedef enum {
    HOMING_IDLE = 0,
    HOMING_RESET,
    HOMING_MOVE_J2_J3,
    HOMING_WAIT_J2_J3,
    HOMING_SAFE_POS,
    HOMING_MOVE_J1,
    HOMING_WAIT_J1,
    HOMING_RETURN_ZERO,
    HOMING_MOVE_J4_J6,
    HOMING_WAIT_J4_J6,
    HOMING_J6_OFFSET,
    HOMING_J6_OFFSET_WAIT,
    HOMING_MOVE_J5,
    HOMING_WAIT_J5,
    HOMING_FINAL_MOVE_ZERO,
    HOMING_FINAL_WAIT_ZERO,
    HOMING_DONE
} HomingState_t;

void HomeAll_Start(void);
void HomeAll_Handler(void);

#endif
