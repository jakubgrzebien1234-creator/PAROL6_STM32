/*
 * OT_Variables.h
 *
 *  Created on: Nov 25, 2025
 *      Author: jakub
 */

#ifndef INC_VARIABLES_OT_H_
#define INC_VARIABLES_OT_H_

#include <stdint.h> // Jeśli używałbyś uint32_t, warto to dodać



extern int RAMP_ID;
extern int CURRENT_ID;
extern int STALL_ID;
extern int HOMING_ID;
extern int VGRIP_ID;
extern int SGRIP_ID;

// Rampy
extern int A1[9];
extern int V1[9];
extern int AMAX[9];
extern int VMAX[9];
extern int D1[9];

// Prąd
extern int IHOLD[9];
extern int IRUN[9];
extern int IHOLDDELAY[9];

// Bazowanie (Homing)
extern int VMAX_H[9];
extern int AMAX_H[9];
extern float OFFSET[9];


extern int PUMP_ON_PRESSURE;	// Ciśnienie załączenia pompy
extern int PUMP_OFF_PRESSURE;	// Ciśnienie wyłączenia pompy
extern int VALVE_DELAY;		// Opóźnienie załączenia zaworu
extern int SOLNEOID_ON_TIME;

extern int EGRIP_IHOLD;		// Prąd trzymania EGRIP
extern int EGRIP_IRUN;			// Prąd pracy EGRIP
extern int EGRIP_SPEED;		// Prędkość EGRIP
extern int EGRIP_FORCE;		// Siła EGRIP
extern int EGRIP_SGT_THRS;

extern int EMM[9];

#endif /* INC_VARIABLES_OT_H_ */
