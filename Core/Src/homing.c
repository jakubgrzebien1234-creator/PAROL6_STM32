#include "homing.h"
#include "Variables_GLOBAL.h"
#include "Variables_OT.h"
#include <stdlib.h> // abs()

extern void PAROL6_AFTER_HOMING();


// >>> DODANO: Dostęp do flagi ESTOP z main.c <<<
extern volatile uint8_t ESTOP_TRIGGER;

// Konfiguracja
#define MICROSTEPS 51200.0
float RATIOS[6] = {6.4, 20.0, 18.0952381, 4.0, 4.0, 10.0};

// Stan maszyny
volatile HomingState_t h_state = HOMING_IDLE;

// Flagi lokalne
static uint8_t loc_homed2 = 0;
static uint8_t loc_homed3 = 0;
static uint8_t loc_homed4 = 0;
static uint8_t loc_homed6 = 0;
static int32_t target_j6_pos = 0;
extern void UART_Direct_Send();

// Funkcja pomocnicza (bez zmian)
int32_t Calculate_Home_Value(int32_t base_steps, int motor_idx)
{
    float angle_correction = OFFSET[motor_idx+1];
    float ratio = RATIOS[motor_idx];
    int32_t steps_correction = (int32_t)((MICROSTEPS * ratio * angle_correction) / 360.0);
    return base_steps + steps_correction;
}

// Funkcja HardStop bez delay (wykonuje się natychmiast)
void TMC5160_Stop_Now(TMC5160_Driver* driver)
{
    TMC5160_WriteRegister(driver, 0x20, 1); // RAMP_MODE = Velocity
    TMC5160_WriteRegister(driver, 0x22, 0); // VACTUAL = 0
    TMC5160_WriteRegister(driver, 0x27, 0); // VMAX = 0
    // Aktualizacja pozycji (XTARGET = XACTUAL)
    int32_t pos = TMC5160_ReadRegister(driver, 0x21);
    TMC5160_WriteRegister(driver, 0x2D, pos);
    // Powrót do Position Mode
    TMC5160_WriteRegister(driver, 0x20, 0);
}

// --- API ---

void HomeAll_Start(void)
{
    h_state = HOMING_RESET;
}

void HomeAll_Handler(void)
{
    // >>> ZMIANA: NATYCHMIASTOWE PRZERWANIE JEŚLI ESTOP AKTYWNY <<<
    if (ESTOP_TRIGGER == 1)
    {
        // 1. Zresetuj maszynę stanów do stanu początkowego (IDLE)
        h_state = HOMING_IDLE;

        // 2. Opcjonalnie: Zresetuj flagi lokalne, żeby po odblokowaniu
        // procedura musiała być uruchomiona od nowa, a nie kontynuowała z błędem
        loc_homed2 = 0; loc_homed3 = 0;
        loc_homed4 = 0; loc_homed6 = 0;

        // 3. Wyjdź z funkcji - nie wysyłaj żadnych komend ruchu!
        return;
    }
    // >>> KONIEC ZMIANY <<<


    // Zmienne pomocnicze do sprawdzania pozycji w finalnym etapie
    int32_t pos1, pos2, pos3, pos4, pos5, pos6;

    switch (h_state)
    {
    case HOMING_IDLE:
        break;

    case HOMING_RESET:
        loc_homed2 = 0; loc_homed3 = 0;
        loc_homed4 = 0; loc_homed6 = 0;
        homed_j1g = 0; homed_j2g = 0; homed_j3g = 0;
        homed_j4g = 0; homed_j5g = 0; homed_j6g = 0;
        TMC5160_WriteRegister(&tmc5, 0x28, 5000);   // DMAX
        TMC5160_WriteRegister(&tmc5, 0x2A, 5000);   // D1
        h_state = HOMING_MOVE_J2_J3;
        break;

    // --- ETAP 1: J2 i J3 ---
    case HOMING_MOVE_J2_J3:
        TMC5160_WriteRegister(&tmc2, 0x20, 0); TMC5160_WriteRegister(&tmc2, 0x27, 30000); TMC5160_WriteRegister(&tmc2, 0x26, 5000); TMC5160_WriteRegister(&tmc2, 0x2D, 1000000);
        TMC5160_WriteRegister(&tmc3, 0x20, 0); TMC5160_WriteRegister(&tmc3, 0x27, 30000); TMC5160_WriteRegister(&tmc3, 0x26, 5000); TMC5160_WriteRegister(&tmc3, 0x2D, -1000000);
        h_state = HOMING_WAIT_J2_J3;
        break;

    case HOMING_WAIT_J2_J3:
        if (!loc_homed2 && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_RESET) {
            TMC5160_Stop_Now(&tmc2);
            int32_t val = Calculate_Home_Value(162133, 1);
            TMC5160_WriteRegister(&tmc2, 0x21, val); TMC5160_WriteRegister(&tmc2, 0x2D, val);
            UART_Direct_Send(msg_h2);
            loc_homed2 = 1; homed_j2g = 1;
        }
        if (!loc_homed3 && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == GPIO_PIN_RESET) {
            TMC5160_Stop_Now(&tmc3);
            int32_t val = Calculate_Home_Value(-187335, 2);
            TMC5160_WriteRegister(&tmc3, 0x21, val); TMC5160_WriteRegister(&tmc3, 0x2D, val);
            UART_Direct_Send(msg_h3);
            loc_homed3 = 1; homed_j3g = 1;
        }
        if (loc_homed2 && loc_homed3) {
            // Bezpieczny powrót J2/J3 przed ruchem J1
            SetRotationAngle(&tmc2, 20, 0);
            SetRotationAngle(&tmc3, 18.0952381, 0);
            h_state = HOMING_MOVE_J1;
        }
        break;

    // --- ETAP 2: J1 ---
    case HOMING_MOVE_J1:
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_RESET) {
            int32_t val = Calculate_Home_Value(24772, 0);
            TMC5160_WriteRegister(&tmc1, 0x21, val); TMC5160_WriteRegister(&tmc1, 0x2D, val);
            homed_j1g = 1;
            h_state = HOMING_RETURN_ZERO;
        } else {
            TMC5160_WriteRegister(&tmc1, 0x20, 0); TMC5160_WriteRegister(&tmc1, 0x27, 30000); TMC5160_WriteRegister(&tmc1, 0x26, 5000); TMC5160_WriteRegister(&tmc1, 0x2D, 1000000);
            h_state = HOMING_WAIT_J1;
        }
        break;

    case HOMING_WAIT_J1:
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_RESET) {
            TMC5160_Stop_Now(&tmc1);
            int32_t val = Calculate_Home_Value(24772, 0);
            TMC5160_WriteRegister(&tmc1, 0x21, val); TMC5160_WriteRegister(&tmc1, 0x2D, val);
            UART_Direct_Send(msg_h1);
            homed_j1g = 1;
            h_state = HOMING_RETURN_ZERO;
        }
        break;

    // --- ETAP 3: Pośredni powrót J1-J3 do zera ---
    case HOMING_RETURN_ZERO:
        TMC5160_WriteRegister(&tmc1, 0x27, 30000); TMC5160_WriteRegister(&tmc1, 0x2D, 0);
        TMC5160_WriteRegister(&tmc2, 0x27, 200000); TMC5160_WriteRegister(&tmc2, 0x2D, 0);
        TMC5160_WriteRegister(&tmc3, 0x27, 200000); TMC5160_WriteRegister(&tmc3, 0x2D, 0);
        UART_Direct_Send(msg_r2);
        UART_Direct_Send(msg_r3);
        UART_Direct_Send(msg_r1);
        h_state = HOMING_MOVE_J4_J6;
        break;

    // --- ETAP 4: J4 i J6 ---
    case HOMING_MOVE_J4_J6:
        TMC5160_WriteRegister(&tmc4, 0x20, 0); TMC5160_WriteRegister(&tmc4, 0x27, 30000); TMC5160_WriteRegister(&tmc4, 0x26, 5000); TMC5160_WriteRegister(&tmc4, 0x2D, 1000000);
        TMC5160_WriteRegister(&tmc8, 0x20, 0); TMC5160_WriteRegister(&tmc8, 0x27, 30000); TMC5160_WriteRegister(&tmc8, 0x26, 5000); TMC5160_WriteRegister(&tmc8, 0x2D, -1000000);
        h_state = HOMING_WAIT_J4_J6;
        break;

    case HOMING_WAIT_J4_J6:
        if (!loc_homed4 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_RESET) {
            TMC5160_Stop_Now(&tmc4);
            int32_t val = Calculate_Home_Value(31802, 3);
            TMC5160_WriteRegister(&tmc4, 0x21, val); TMC5160_WriteRegister(&tmc4, 0x2D, val);
            UART_Direct_Send(msg_h4);
            loc_homed4 = 1; homed_j4g = 1;
        }
        if (!loc_homed6 && HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5) == GPIO_PIN_RESET) {
            TMC5160_Stop_Now(&tmc8);
            int32_t val = Calculate_Home_Value(-120888, 5);
            TMC5160_WriteRegister(&tmc8, 0x21, val); TMC5160_WriteRegister(&tmc8, 0x2D, val);
            UART_Direct_Send(msg_h6);
            loc_homed6 = 1; homed_j6g = 1;
        }
        if (loc_homed4 && loc_homed6) h_state = HOMING_J6_OFFSET;
        break;

    case HOMING_J6_OFFSET:
        TMC5160_WriteRegister(&tmc8, 0x20, 0); TMC5160_WriteRegister(&tmc8, 0x27, 80000); TMC5160_WriteRegister(&tmc8, 0x26, 15000);
        target_j6_pos = (int32_t)(MICROSTEPS * RATIOS[5] * 70.0 / 360.0);
        TMC5160_WriteRegister(&tmc8, 0x2D, target_j6_pos);
        h_state = HOMING_J6_OFFSET_WAIT;
        break;

    case HOMING_J6_OFFSET_WAIT:
        if (abs((int32_t)TMC5160_ReadRegister(&tmc8, 0x21) - target_j6_pos) <= 50) h_state = HOMING_MOVE_J5;
        break;

    // --- ETAP 5: J5 ---
    case HOMING_MOVE_J5:
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == GPIO_PIN_RESET) {
             int32_t val = Calculate_Home_Value(68267, 4);
             TMC5160_WriteRegister(&tmc5, 0x21, val); TMC5160_WriteRegister(&tmc5, 0x2D, val);
             UART_Direct_Send(msg_h5);
             homed_j5g = 1;
             h_state = HOMING_FINAL_MOVE_ZERO;
        } else {
            TMC5160_WriteRegister(&tmc5, 0x20, 0); TMC5160_WriteRegister(&tmc5, 0x27, 10000); TMC5160_WriteRegister(&tmc5, 0x26, 4000); TMC5160_WriteRegister(&tmc5, 0x2D, 1000000);
            h_state = HOMING_WAIT_J5;
        }
        break;

    case HOMING_WAIT_J5:
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4) == GPIO_PIN_RESET) {
            TMC5160_Stop_Now(&tmc5);
            int32_t val = Calculate_Home_Value(68267, 4);
            TMC5160_WriteRegister(&tmc5, 0x21, val); TMC5160_WriteRegister(&tmc5, 0x2D, val);
            UART_Direct_Send(msg_h5);
            homed_j5g = 1;
            h_state = HOMING_FINAL_MOVE_ZERO;
        }
        break;

    // --- ETAP 6: FINALNE USTAWIENIE WSZYSTKIEGO NA 0 ---
    case HOMING_FINAL_MOVE_ZERO:
    	UART_Direct_Send(msg_r4);
    	UART_Direct_Send(msg_r5);
    	UART_Direct_Send(msg_r6);
        TMC5160_WriteRegister(&tmc1, 0x20, 0); TMC5160_WriteRegister(&tmc1, 0x27, 30000);  TMC5160_WriteRegister(&tmc1, 0x2D, 0);
        TMC5160_WriteRegister(&tmc2, 0x20, 0); TMC5160_WriteRegister(&tmc2, 0x27, 100000); TMC5160_WriteRegister(&tmc2, 0x2D, 0);
        TMC5160_WriteRegister(&tmc3, 0x20, 0); TMC5160_WriteRegister(&tmc3, 0x27, 100000); TMC5160_WriteRegister(&tmc3, 0x2D, 0);
        TMC5160_WriteRegister(&tmc4, 0x20, 0); TMC5160_WriteRegister(&tmc4, 0x27, 50000);  TMC5160_WriteRegister(&tmc4, 0x2D, 0);
        TMC5160_WriteRegister(&tmc5, 0x20, 0); TMC5160_WriteRegister(&tmc5, 0x27, 30000);  TMC5160_WriteRegister(&tmc5, 0x2D, 0);
        TMC5160_WriteRegister(&tmc8, 0x20, 0); TMC5160_WriteRegister(&tmc8, 0x27, 50000);  TMC5160_WriteRegister(&tmc8, 0x2D, 0);
        h_state = HOMING_FINAL_WAIT_ZERO;
        break;

    case HOMING_FINAL_WAIT_ZERO:
        pos1 = (int32_t)TMC5160_ReadRegister(&tmc1, 0x21);
        pos2 = (int32_t)TMC5160_ReadRegister(&tmc2, 0x21);
        pos3 = (int32_t)TMC5160_ReadRegister(&tmc3, 0x21);
        pos4 = (int32_t)TMC5160_ReadRegister(&tmc4, 0x21);
        pos5 = (int32_t)TMC5160_ReadRegister(&tmc5, 0x21);
        pos6 = (int32_t)TMC5160_ReadRegister(&tmc8, 0x21);

        if (abs(pos1) < 100 && abs(pos2) < 100 && abs(pos3) < 100 &&
            abs(pos4) < 100 && abs(pos5) < 100 && abs(pos6) < 100)
        {
            h_state = HOMING_DONE;
        }
        break;

    case HOMING_DONE:
        UART_Direct_Send((uint8_t*)"HOMING_COMPLETE_OK\r\n");
        PAROL6_AFTER_HOMING();
        h_state = HOMING_IDLE;
        break;

    default:
        h_state = HOMING_IDLE;
        break;
    }
}
