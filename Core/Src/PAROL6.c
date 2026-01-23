/*
 * PAROL6.c
 *
 * Description:
 * Główna logika sterowania robotem. Obsługa inicjalizacji silników,
 * sterowania ruchem, chwytakiem, wymianą narzędzi oraz procedurami bezpieczeństwa.
 */

#include "PAROL6.h"
#include "main.h"
#include "tmc5160.h"
#include "Variables_GLOBAL.h"
#include "Variables_OT.h" // Upewnij się, że tu są tablice VMAX[], AMAX[], IRUN[] itp.
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

// Deklaracja zewnętrzna (jeśli nie ma w nagłówkach)
extern void UART_Direct_Send(const uint8_t* data);

// Definicje lokalne (jeśli nie ma ich w .h)
#define TMC5160_DRV_STATUS_ADDR 0x6F
#define TMC5160_COOLCONF_ADDR   0x6D
#define TMC5160_RAMP_STATUS_ADDR 0x35

#ifndef SOLNEOID_ON_TIME
#define SOLNEOID_ON_TIME 5000 // Domyślny czas jeśli brak define
#endif

// Zmienne lokalne dla tego pliku (static)
static volatile int32_t EGRIP_VACTUAL = 0;

/* ============================================================================== */
/* 1. INICJALIZACJA SYSTEMU                                                     */
/* ============================================================================== */

void PAROL6_INIT(void) {
    for(int i = 0; i < 6; i++) {
        if(motors[i] == NULL) continue;

        // 1. ODBLOKOWANIE PRĄDU (Global Scaler)
        TMC5160_WriteRegister(motors[i], 0x0B, 200);

        // 2. CHOPCONF (Chopper Configuration)
        // 0x10410153 = MRES 0 (Native 256 microsteps)
        TMC5160_WriteRegister(motors[i], 0x6C, 0x10410153);

        // 4. TPOWERDOWN (Opóźnienie przejścia w stan IHOLD)
        TMC5160_WriteRegister(motors[i], 0x11, 0x0000000A);

        // 5. TPWMTHRS
        TMC5160_WriteRegister(motors[i], 0x13, 500);

        // 6. PWMCONF
        TMC5160_WriteRegister(motors[i], 0x70, 0xC40C001E);

        // 7. GCONF - MODYFIKACJA DLA SILNIKA NR 2 (i==1)
        // (Tu warto sprawdzić czy to na pewno ma być dla każdego, czy tylko dla i=1?)
        // W oryginale było w pętli dla każdego silnika:
        TMC5160_WriteRegister(motors[i], 0x00, 0x00000004);

        // 8. TCOOLTHRS
        TMC5160_WriteRegister(motors[i], 0x14, 0);

        // 9. Tryb pozycyjny (RAMPMODE = 0)
        TMC5160_WriteRegister(motors[i], 0x20, 0);
    }
}

void PAROL6_EGRIP_INIT(void) {
    // Używamy &tmc7 zdefiniowanego w Variables_GLOBAL
    TMC5160_Driver* grip_drv = &tmc7;

    // 1. Reset błędów
    TMC5160_WriteRegister(grip_drv, 0x01, 0x00000007); // GSTAT
    TMC5160_WriteRegister(grip_drv, 0x0B, 200);

    // 2. SpreadCycle (Wymagane dla StallGuard2)
    TMC5160_WriteRegister(grip_drv, 0x00, 0x00000000); // GCONF

    // 3. Prądy
    TMC5160_WriteRegister(grip_drv, 0x0B, 200);        // GLOBALSCALER
    // Zakładam, że EGRIP_IHOLD/IRUN są w Variables_OT.h lub main.h
    TMC5160_WriteRegister(grip_drv, 0x10, makeIHOLD_IRUN(EGRIP_IHOLD, EGRIP_IRUN, 6));
    TMC5160_WriteRegister(grip_drv, 0x11, 10);         // TPOWERDOWN

    // 4. Chopper
    TMC5160_WriteRegister(grip_drv, 0x6C, 0x10410153); // CHOPCONF

    // 6. Próg prędkości (TCOOLTHRS) - StallGuard działa powyżej tej prędkości
    TMC5160_WriteRegister(grip_drv, 0x14, 2000);

    // 7. Wyłączenie StealthChop (TPWMTHRS)
    TMC5160_WriteRegister(grip_drv, 0x13, 500);

    // 8. Ramp generator
    TMC5160_WriteRegister(grip_drv, 0x20, 1);     // RAMPMODE = Velocity/Positioning initialization
    TMC5160_WriteRegister(grip_drv, 0x23, 10000); // VSTART
    TMC5160_WriteRegister(grip_drv, 0x24, 50000); // A1
    TMC5160_WriteRegister(grip_drv, 0x25, 10000); // V1
    TMC5160_WriteRegister(grip_drv, 0x26, 10000); // AMAX
    TMC5160_WriteRegister(grip_drv, 0x28, 10000); // DMAX
    TMC5160_WriteRegister(grip_drv, 0x2B, 10);    // VSTOP
}

/* ============================================================================== */
/* 2. STEROWANIE RUCHEM (RAMP & POSITION)                                       */
/* ============================================================================== */

void PAROL6_XACTUAL(int8_t XACTUAL) {
    // Nadpisanie pozycji dla wszystkich osi (używane przy bazowaniu)
    for(int i = 0; i < 6; i++) {
        if(motors[i]) TMC5160_WriteRegister(motors[i], 0x21, XACTUAL);
    }
}

void PAROL6_RAMP(int id) {
    if (id < 1 || id > 6) return; // Zabezpieczenie indeksu
    TMC5160_Driver* drv = (TMC5160_Driver*)motors[id-1];

    // 1. Zabezpieczenie przed zerami (domyślne wartości)
    if(VMAX[id] == 0) VMAX[id] = 50000;
    if(AMAX[id] == 0) AMAX[id] = 5000;
    if(A1[id] == 0)   A1[id]   = 1000;
    if(D1[id] == 0)   D1[id]   = 5000;

    // 2. Wymuszenie trybu pozycjonowania
    TMC5160_WriteRegister(drv, 0x20, 0); // RAMPMODE = Position

    // 3. Wysłanie parametrów
    TMC5160_WriteRegister(drv, 0x24, A1[id]);   // A1
    TMC5160_WriteRegister(drv, 0x25, V1[id]);   // V1
    TMC5160_WriteRegister(drv, 0x26, AMAX[id]); // AMAX
    TMC5160_WriteRegister(drv, 0x27, VMAX[id]); // VMAX
    TMC5160_WriteRegister(drv, 0x28, D1[id]);   // DMAX
    TMC5160_WriteRegister(drv, 0x2A, D1[id]);   // D1

    TMC5160_WriteRegister(drv, 0x23, 1);  // VSTART
    TMC5160_WriteRegister(drv, 0x2B, 10); // VSTOP
}

void PAROL6_RAMP_ALL() {
    for(int i = 0; i < 6; i++) {
        // Indeksowanie tablic VMAX/AMAX w Twoim kodzie zaczyna się od 1 dla silnika 1?
        // UWAGA: Tablice w C są od 0. Jeśli VMAX[0] to silnik 1, to ok.
        // Jeśli VMAX[1] to silnik 1, to poniżej trzeba użyć [i] lub [i+1] zależnie od definicji.
        // Zakładam spójność z PAROL6_RAMP(id), gdzie używasz VMAX[id].

        // Tutaj zakładam, że A1, V1, etc. są indeksowane 0..5 w pętli.
        if (A1[i] <= 10000) TMC5160_WriteRegister(motors[i], 0x24, A1[i]);
        if (V1[i] <= 20000) TMC5160_WriteRegister(motors[i], 0x25, V1[i]);

        TMC5160_WriteRegister(motors[i], 0x26, AMAX[i]);
        TMC5160_WriteRegister(motors[i], 0x27, VMAX[i]);
        TMC5160_WriteRegister(motors[i], 0x28, D1[i]);
        TMC5160_WriteRegister(motors[i], 0x2A, D1[i]);
    }
}

void PAROL6_CURRENT(int id) {
    if (id < 1 || id > 6) return;
    // id jest 1-based, tablice IRUN/IHOLD zakładam że też (wg Twojego starego kodu)
    uint32_t CURRENT = makeIHOLD_IRUN(IHOLD[id], IRUN[id], IHOLDDELAY[id]);
    TMC5160_WriteRegister(motors[id-1], 0x10, CURRENT);
}

void PAROL6_CURRENT_ALL() {
    // Poprawiono zakres pętli na i < 6
    for(int i = 0; i < 6; i++) {
        // Zakładam indeksowanie tablicy 0..5
        uint32_t CURRENT = makeIHOLD_IRUN(IHOLD[i], IRUN[i], IHOLDDELAY[i]);
        TMC5160_WriteRegister(motors[i], 0x10, CURRENT);
    }
}

void PAROL6_HOMING_ALL() {
    // Poprawiono zakres pętli na i < 6
    for(int i = 0; i < 6; i++) {
        TMC5160_WriteRegister(motors[i], 0x27, VMAX_H[i]);
        TMC5160_WriteRegister(motors[i], 0x26, AMAX_H[i]);
    }
}

void PAROL6_AFTER_HOMING() {
    for(int i = 1; i <= 6; i++) {
        TMC5160_Driver* drv = (TMC5160_Driver*)motors[i-1];

        // 1. SYNCHRONIZACJA CELU
        int32_t current_pos = TMC5160_ReadRegister(drv, 0x21); // XACTUAL
        TMC5160_WriteRegister(drv, 0x2D, current_pos);         // XTARGET

        // 2. Przywrócenie parametrów ruchu
        PAROL6_RAMP(i);

        // 3. Przywrócenie prądów
        if(IRUN[i] == 0) IRUN[i] = 10;
        if(IHOLD[i] == 0) IHOLD[i] = 5;
        PAROL6_CURRENT(i);
    }
}

void PAROL6_POSITION(uint8_t Driver) {
    // Driver: 0..5
    if (Driver >= 6) return;

    int i = Driver;
    position[i] = TMC5160_ReadRegister(motors[i], 0x21);
    realAngle[i] = GetRotationAngle(motors[i], ratios[i]);

    // Format: "A1_45.25"
    sprintf(msg, "A%d_%.2f\r\n", i + 1, realAngle[i]);
    UART_Direct_Send((uint8_t*)msg);
}

/* ============================================================================== */
/* 3. BEZPIECZEŃSTWO I AWARYJNE ZATRZYMANIE                                     */
/* ============================================================================== */

void PAROL6_Stop(void* driver_ptr) {
    if (driver_ptr == NULL) return;
    TMC5160_Driver* drv = (TMC5160_Driver*)driver_ptr;

    // 1. VMAX = 0 (Hamowanie z rampą)
    TMC5160_WriteRegister(drv, 0x27, 0);

    // 2. Synchronizacja celu, aby nie kontynuował po przywróceniu prędkości
    uint32_t current_pos = TMC5160_ReadRegister(drv, 0x21);
    TMC5160_WriteRegister(drv, 0x2D, current_pos);
}

void PAROL6_HardStopALL(void) {
    for(int i = 0; i < 6; i++) {
        TMC5160_Driver* drv = (TMC5160_Driver*)motors[i];
        if (!drv) continue;

        // Tryb prędkości -> VMAX=0 -> XTARGET=XACTUAL -> Tryb pozycji
        TMC5160_WriteRegister(drv, 0x20, 1); // RAMPMODE = Velocity
        TMC5160_WriteRegister(drv, 0x27, 0); // VMAX = 0
        int32_t current_pos = TMC5160_ReadRegister(drv, 0x21);
        TMC5160_WriteRegister(drv, 0x2D, current_pos);
        TMC5160_WriteRegister(drv, 0x20, 0); // RAMPMODE = Position
    }
}

void PAROL6_EMERGENCY_STOP(void) {
    for(int i = 0; i < 6; i++) {
        TMC5160_Driver* drv = (TMC5160_Driver*)motors[i];
        if (drv == NULL) continue;

        // >>> HARD STOP SEQUENCE <<<
        TMC5160_WriteRegister(drv, 0x20, 1); // RAMPMODE = 1 (Velocity)
        TMC5160_WriteRegister(drv, 0x27, 0); // VMAX = 0
        TMC5160_WriteRegister(drv, 0x22, 0); // VACTUAL = 0 (Stop natychmiastowy)

        int32_t current_pos = TMC5160_ReadRegister(drv, 0x21);
        TMC5160_WriteRegister(drv, 0x2D, current_pos);

        TMC5160_WriteRegister(drv, 0x20, 0); // RAMPMODE = 0 (Position)
    }
}

void PAROL6_RESUME(void) {
    for(int i = 0; i < 6; i++) {
        TMC5160_Driver* drv = (TMC5160_Driver*)motors[i];
        if (drv == NULL) continue;

        // 1. Przywróć tryb pozycyjny
        TMC5160_WriteRegister(drv, 0x20, 0);

        // 2. Przywróć bezpieczne parametry ruchu
        // Używamy i+1 jeśli tablice AMAX/VMAX są 1-based (sprawdź Variables_OT)
        TMC5160_WriteRegister(drv, 0x26, AMAX[i+1]);
        TMC5160_WriteRegister(drv, 0x27, VMAX[i+1]);
        TMC5160_WriteRegister(drv, 0x2A, D1[i+1]);
        TMC5160_WriteRegister(drv, 0x25, V1[i+1]);
        TMC5160_WriteRegister(drv, 0x24, A1[i+1]);
    }
}

void PAROL6_MOTOR_CONNECTED() {
    static uint8_t error_count[6] = {0};

    for(int i = 0; i < 6; i++) {
        uint32_t drv_stat = TMC5160_ReadRegister(motors[i], 0x6F);
        // EMM[] zdefiniowane w Variables_OT?
        EMM[i] = drv_stat & 0x60000000; // Sprawdzenie flag błędów sterownika

        if (EMM[i] != 0) {
            if (error_count[i] < 5) {
                error_count[i]++;
            }
            if (error_count[i] == 5) {
                char loc_msg[32];
                sprintf(loc_msg, "EMM%d", i + 1); // Wyślij nr silnika
                UART_Direct_Send((uint8_t*)loc_msg);
                error_count[i]++; // Blokada powtarzania
            }
        } else {
            error_count[i] = 0;
        }
    }
}

void PAROL6_LIMIT_SWITCH_STATUS() {
    static int prev_states[6] = {-1, -1, -1, -1, -1, -1};

    // Sprawdzenie czy cokolwiek się zmieniło
    if (s_j1 != prev_states[0] || s_j2 != prev_states[1] || s_j3 != prev_states[2] ||
        s_j4 != prev_states[3] || s_j5 != prev_states[4] || s_j6 != prev_states[5])
    {
        char loc_msg[64];
        sprintf(loc_msg, "LIMITSWITCH_%d,%d,%d,%d,%d,%d\r\n", s_j1, s_j2, s_j3, s_j4, s_j5, s_j6);
        UART_Direct_Send((uint8_t*)loc_msg);

        // Aktualizacja historii
        prev_states[0] = s_j1;
        prev_states[1] = s_j2;
        prev_states[2] = s_j3;
        prev_states[3] = s_j4;
        prev_states[4] = s_j5;
        prev_states[5] = s_j6;
    }
}

/* ============================================================================== */
/* 4. OBSŁUGA NARZĘDZI (CHWYTAK, WYMIANA)                                       */
/* ============================================================================== */

void PAROL6_TOOL_CHANGE() {
    static uint32_t timerStart = 0;
    static uint8_t isRunning = 0;

    // Start sekwencji
    if (isRunning == 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
        timerStart = HAL_GetTick();
        isRunning = 1;
    }

    // Odliczanie czasu
    if (isRunning == 1) {
        if (HAL_GetTick() - timerStart > SOLNEOID_ON_TIME) {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
            isRunning = 0;
            tool_changer_active = 0; // Zakończ proces
        }
    }
}

void PAROL6_EGRIP_PROCESS(void) {
    // --- Konfiguracja i Zmienne Statyczne ---
    static uint32_t prev_sg_result = 0xFFFFFFFF;
    static int16_t stall_error_count = 0;
    static uint32_t speed_stable_timer = 0;
    static float sg_baseline = 0.0f;
    static uint8_t baseline_init = 0;

    const uint32_t STABILIZATION_DELAY = 500;
    const int16_t MAX_ERROR_COUNT = 10;

    if(HAL_GetTick() - EGRIP_PROCESS_TIMER > 10) {
        EGRIP_PROCESS_TIMER = HAL_GetTick();

        // 1. Odczyt rejestrów chwytaka
        uint32_t drv_status = TMC5160_ReadRegister(&tmc7, 0x6F);
        uint32_t current_sg = drv_status & 0x3FF; // Wyciągnięcie wartości SG
        EGRIP_SG_RESULT = current_sg;

        int32_t vactual = (int32_t)TMC5160_ReadRegister(&tmc7, 0x22);

        // 2. Raportowanie UART przy zmianie
        if (current_sg != prev_sg_result) {
            prev_sg_result = current_sg;
            // Opcjonalnie można to wykomentować, żeby nie spamować UART
            // char loc_msg[32];
            // sprintf(loc_msg, "EGRIP_SR_%lu", current_sg);
            // UART_Direct_Send((uint8_t*)loc_msg);
        }

        // 3. Logika Utyku (Stall Detection)
        // Sprawdzamy tylko gdy silnik jedzie szybko (blisko zadanej prędkości)
        // Zakładam, że EGRIP_SPEED i EGRIP_SGT_THRS są w Variables_OT.h
        if(abs(vactual) > 500 && abs(vactual) >= (EGRIP_SPEED - 100)) {

            // Czekamy na ustabilizowanie prędkości
            if(HAL_GetTick() - speed_stable_timer > STABILIZATION_DELAY) {

                if(baseline_init == 0) {
                    sg_baseline = (float)current_sg;
                    baseline_init = 1;
                    stall_error_count = 0;
                }

                // Obliczenie progu
                uint32_t safe_thrs = (EGRIP_SGT_THRS > 0) ? EGRIP_SGT_THRS : 30;
                float cutoff_limit = sg_baseline - (float)safe_thrs;

                // Porównanie
                if((float)current_sg < cutoff_limit) {
                    stall_error_count++;
                } else {
                    if(stall_error_count > 0) stall_error_count--;
                    // Adaptacja średniej (BaseLine)
                    sg_baseline = (sg_baseline * 0.95f) + ((float)current_sg * 0.05f);
                }

                // Reakcja na błąd (STOP)
                if(stall_error_count >= MAX_ERROR_COUNT) {
                    TMC5160_WriteRegister(&tmc7, 0x27, 0); // VMAX=0 -> STOP
                    stall_error_count = 0;
                }
            }
        } else {
            // Reset gdy silnik stoi lub zwalnia
            speed_stable_timer = HAL_GetTick();
            stall_error_count = 0;
            baseline_init = 0;
            sg_baseline = 0.0f;
        }
    }
}
