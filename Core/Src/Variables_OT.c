/*
 * OT_Variables.c
 *
 *  Created on: Nov 25, 2025
 *      Author: jakub
 */

#include "Variables_OT.h"

int RAMP_ID;	// Identyfikator rampy
int CURRENT_ID; // Identyfikator prądu
int HOMING_ID;	// Identyfikator bazowania

int A1[9];		// Przyspieszenie A1
int V1[9];		// Prędkość V1
int AMAX[9];	// Przyspieszenie AMAX
int VMAX[9];	// Prędkość VMAX
int D1[9];		// Hamowanie D1

int IHOLD[9];	// Prąd trzymania
int IRUN[9];	// Prąd pracy
int IHOLDDELAY[9];	// Opóźnienie przełączenia

int VMAX_H[9];	// Prędkość VMAX bazowania
int AMAX_H[9];	// Przyspieszenie AMAX bazowania
float OFFSET[9];// Offset bazowania

int PUMP_ON_PRESSURE = 0;	// Ciśnienie załączenia pompy
int PUMP_OFF_PRESSURE = 0;	// Ciśnienie wyłączenia pompy
int SOLNEOID_ON_TIME = 0;

int EGRIP_IHOLD = 0;		// Prąd trzymania EGRIP
int EGRIP_IRUN = 0;			// Prąd pracy EGRIP
int EGRIP_SPEED = 0;		// Prędkość EGRIP
int EGRIP_FORCE = 0;		// Siła EGRIP
int EGRIP_SGT_THRS = 0;		// Próg zadziałania STALL

int EMM[9];			// Sygnał niepodpiętego silnika
