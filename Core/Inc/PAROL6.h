/*
 * PAROL6.h
 *
 *  Created on: Nov 23, 2025
 *      Author: jakub
 */

#include <stdint.h>

#ifndef INC_PAROL6_H_
#define INC_PAROL6_H_


void PAROL6_INIT(void);
void PAROL6_RAMP_MODE(int8_t XACTUAL);
void PAROL6_XACTUAL(int8_t XACTUAL);

void PAROL6_TERMINAL_RAMP(int RAMP_ID, int A1, int V1, int AMAX, int VMAX, int D1);
void PAROL6_TERMINAL_CURRENT(int CURRENT_ID, int IHOLD, int IRUN, int IHOLDDELAY);
void PAROL6_StallGuard(int motor_id, int new_sensitivity);
void PAROL6_TERMINAL_HOMING(int HOMING_ID, int VMAX_H, int AMAX_H, int OFFSET);

void PAROL6_RAMP_ALL();
void PAROL6_CURRENT_ALL();
void PAROL6_HOMING_ALL();
void PAROL6_POSITION(uint8_t Driver);
void Update_Motor_Status(void);
void PAROL6_HardStopALL();

void PAROL6_RAMP(int id);
void PAROL6_CURRENT(int id);
void PAROL6_AFTER_HOMING();
void PAROL6_TOOL_CHANGE();
void PAROL6_LIMIT_SWITCH_STATUS();

void PAROL6_EGRIP_INIT(void);
void PAROL6_EGRIP();
void Check_StallGuard(void);
void PAROL6_Stop(void* driver_ptr);
void PAROL6_EMERGENCY_STOP();
void TMC_Poll_Sensors(void);
void PAROL6_EGRIP_PROCESS(void);
void PAROL6_RESUME(void);
void PAROL6_MOTOR_CONNECTED();

// Definicje stanów dla czytelności kodu
typedef enum {
    STATE_NO_TOOL,         // Brak chwytaka (Impuls 3s)
    STATE_TOOL_CONNECTED,  // Chwytak jest (Włączone, max 3s safety)
    STATE_DISCONNECT_WAIT, // Oczekiwanie 1s po odłączeniu
    STATE_OFF_LOCKED       // Wyłączone na stałe (do zmiany stanu wejść)
} ToolState_t;

#endif /* INC_PAROL6_H_ */
