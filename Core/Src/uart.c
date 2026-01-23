/* uart.c */
#include "main.h"
#include "uart.h"
#include "Variables_OT.h"
#include "stdio.h"
#include <string.h>
#include "Variables_GLOBAL.h"

// --- 1. WAŻNE DEKLARACJE (Tego brakowało) ---
extern TIM_HandleTypeDef htim1;
extern uint32_t makeIHOLD_IRUN();
extern void PAROL6_Reset_Collision_State();
extern void UART_Direct_Send(const uint8_t* data);
extern void PAROL6_TOOL_CHANGE_ELECTRIC();
extern void PAROL6_EGRIP();
extern void PAROL6_EGRIP_INIT();

void Parse_And_Execute_Command(uint8_t* data)
{
    int id;
    int val1, val2, val3, val4, val5;
    // 1. Zapis konfiguracji (Ignorujemy w trybie Live, ale nie usuwamy żeby nie psuć protokołu)
    if (strstr((char*)data, "SAVE_CONFIG")) return;

    // 2. Reset błędów
    if (strstr((char*)data, "COLLISION_OK")) {
        UART_Direct_Send((uint8_t*)"SYSTEM_RESET_OK\r\n");
        return;
    }

    // >>> NOWA KOMENDA: ODBLOKOWANIE SYSTEMU <<<
    if (strstr((char*)data, "CONFIG_DONE"))
    {
        system_configured = 1; // Odblokuj pętlę while w main.c
        UART_Direct_Send((uint8_t*)"SYSTEM_ARMED_OK\r\n");
        return;
    }

        // ZAMYKANIE (Kręcenie w jedną stronę - np. RAMPMODE 1)
        if (strstr((char*)data, "EGRIP_OPEN")) {
            // RAMPMODE = 1 (Velocity Mode Positive - obrót w prawo)
            TMC5160_WriteRegister(&tmc7, 0x20, 1);
            TMC5160_WriteRegister(&tmc7, 0x27, EGRIP_SPEED);
            // Upewnij się, że AMAX jest ustawione (przyspieszenie)
            TMC5160_WriteRegister(&tmc7, 0x26, 5000);
            return;
        }

        // OTWIERANIE (Kręcenie w drugą stronę - np. RAMPMODE 2)
        if (strstr((char*)data, "EGRIP_CLOSE")) {
            // RAMPMODE = 2 (Velocity Mode Negative - obrót w lewo)
            TMC5160_WriteRegister(&tmc7, 0x20, 2);
            TMC5160_WriteRegister(&tmc7, 0x27, EGRIP_SPEED);
            // Upewnij się, że AMAX jest ustawione
            TMC5160_WriteRegister(&tmc7, 0x26, 5000);
            return;
        }

        // OPCJONALNIE: Komenda STOP (jeśli będziesz chciał zatrzymać ręcznie)
        if (strstr((char*)data, "EGRIP_STOP")) {
            // VMAX = 0 zatrzymuje silnik w trybie Velocity
            TMC5160_WriteRegister(&tmc7, 0x27, 0);
            return;
        }

        // 3. Obsługa RUCHU (JOG / GROUP MOVE)
        // Oczekiwany format: "J_j1,j2,j3,j4,j5,j6" np. "J_90.0,0.0,10.5,-20.0,50.0,30.0"
        if (data[0] == 'J' && data[1] == '_')
        {
            float v1, v2, v3, v4, v5, v6;

            // Parsujemy 6 wartości oddzielonych przecinkami
            // Zwraca liczbę pomyślnie odczytanych zmiennych (powinno być 6)
            if (sscanf((char*)data, "J_%f,%f,%f,%f,%f,%f", &v1, &v2, &v3, &v4, &v5, &v6) == 6)
            {
                // 1. Ustawienie trybu pozycji (RAMP_MODE = 0) dla wszystkich osi
                // (Ważne, żeby nie zostały w trybie prędkości po homingu)
                TMC5160_WriteRegister(&tmc1, 0x20, 0);
                TMC5160_WriteRegister(&tmc2, 0x20, 0);
                TMC5160_WriteRegister(&tmc3, 0x20, 0);
                TMC5160_WriteRegister(&tmc4, 0x20, 0);
                TMC5160_WriteRegister(&tmc5, 0x20, 0);
                TMC5160_WriteRegister(&tmc8, 0x20, 0); // Pamiętam, że J6 to tmc8

                // 2. Zadanie pozycji (z zachowaniem Twoich przełożeń i odwróconego znaku)
                // J1: Ratio 6.4
                SetRotationAngle(&tmc1, 6.4, -v1);

                // J2: Ratio 20.0
                SetRotationAngle(&tmc2, 20.0, -v2);

                // J3: Ratio 18.095...
                SetRotationAngle(&tmc3, 18.0952381, -v3);

                // J4: Ratio 4.0
                SetRotationAngle(&tmc4, 4.0, -v4);

                // J5: Ratio 4.0
                SetRotationAngle(&tmc5, 4.0, -v5);

                // J6: Ratio 10.0 (tmc8)
                SetRotationAngle(&tmc8, 10.0, -v6);

                return; // Sukces, wychodzimy
            }
        }


    // 4. Konfiguracja RAMP (Ruch)
    if (sscanf((char*)data, "OT,ramp,J%d,%d,%d,%d,%d,%d", &id, &val1, &val2, &val3, &val4, &val5) == 6) {
        if (id >= 1 && id <= 6) {
            A1[id] = val1; V1[id] = val2; AMAX[id] = val3; VMAX[id] = val4; D1[id] = val5;

            TMC5160_Driver* drv = (TMC5160_Driver*)motors[id-1];
            TMC5160_WriteRegister(drv, 0x24, A1[id]);
            TMC5160_WriteRegister(drv, 0x25, V1[id]);
            TMC5160_WriteRegister(drv, 0x26, AMAX[id]);
            TMC5160_WriteRegister(drv, 0x27, VMAX[id]);
            TMC5160_WriteRegister(drv, 0x28, D1[id]);
            TMC5160_WriteRegister(drv, 0x2A, D1[id]);

            TMC5160_WriteRegister(drv, 0x23, 1);
            TMC5160_WriteRegister(drv, 0x2B, 10);
        }
        return;
    }

    // 5. Konfiguracja CURRENT (Prąd)
    if (sscanf((char*)data, "OT,current,J%d,%d,%d,%d", &id, &val1, &val2, &val3) == 4) {
        if (id >= 1 && id <= 6) {
            IHOLD[id] = val1; IRUN[id] = val2; IHOLDDELAY[id] = val3;

            uint32_t CURRENT = makeIHOLD_IRUN(IHOLD[id], IRUN[id], IHOLDDELAY[id]);
            TMC5160_Driver* drv = (TMC5160_Driver*)motors[id-1];
            TMC5160_WriteRegister(drv, 0x10, CURRENT);
        }
        return;
    }

    // 6. Reszta komend
    if (sscanf((char*)data, "OT,homing,J%d,%d,%d,%d", &id, &val1, &val2, &val3) == 4) {
        if (id >= 1 && id <= 6) { VMAX_H[id] = val1; AMAX_H[id] = val2; OFFSET[id] = val3; }
        return;
    }


    if (sscanf((char*)data, "OT,global,%d", &val1) == 1) {
           	SOLNEOID_ON_TIME = val1;
            return;
        }


    // Obsługa chwytaków (VGrip / SGrip) - Ważne, żeby też były
    if (sscanf((char*)data, "OT,VGrip,%d,%d,%d", &val1, &val2, &val3) == 3) {
        PUMP_ON_PRESSURE = val1; PUMP_OFF_PRESSURE = val2; VALVE_DELAY = val3;
        return;
    }
    if (sscanf((char*)data, "OT,SGrip,%d,%d,%d,%d,%d", &val1, &val2, &val3, &val4,&val5) == 5) {
    	EGRIP_IHOLD = val1; EGRIP_IRUN = val2; EGRIP_SPEED = val3; EGRIP_FORCE = val4; EGRIP_SGT_THRS = val5;
    	PAROL6_EGRIP_INIT();
    	    uint32_t current_val = TMC5160_ReadRegister(&tmc7, 0x6D);
    	    current_val &= ~(0x7F << 16);
    	    current_val |= ((EGRIP_FORCE & 0x7F) << 16);
    	    current_val |= (1UL << 24);
    	    TMC5160_WriteRegister(&tmc7, 0x6D, current_val);
        return;
    }


    if (strstr((char*)data, "TOOL_CHANGE")) {tool_changer_active = 1; }
    if (strstr((char*)data, "HOME")) { homing_command = 1; }
    if (strstr((char*)data, "VAC_ON"))   { HAL_GPIO_WritePin(PUMP_GPIO_Port, PUMP_Pin, GPIO_PIN_SET); }
    if (strstr((char*)data, "VAC_OFF"))  { HAL_GPIO_WritePin(PUMP_GPIO_Port, PUMP_Pin, GPIO_PIN_RESET); }
    if (strstr((char*)data, "VALVEON"))  { __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 500); }
    if (strstr((char*)data, "VALVEOFF")) { __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0); }
    if (strstr((char*)data, "VGripON"))  { VGripStatus = 1; }
    if (strstr((char*)data, "VGripOFF")) { VGripStatus = 0; }
}
