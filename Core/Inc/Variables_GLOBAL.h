/*
 * Variables_GLOBAL.h
 */

#ifndef INC_VARIABLES_GLOBAL_H_
#define INC_VARIABLES_GLOBAL_H_

// --- 1. NAPRAWA BŁĘDÓW TYPÓW
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "main.h"
#include "tmc5160.h"

// ==============================================================================
// 2. STAŁE
// ==============================================================================
#define DEBOUNCE_TIME_MS 50
#define TEMP_THRESHOLD 0.5f
#define MIN_ABSOLUTE_SPEED 15000
#define SG_FILTER_SAMPLES 6
#define SG_ACTIVATION_THRESHOLD 0.6f
#define RX_BUFFER_SIZE 256
#define TX_BUFFER_SIZE 256
#define ADC_SAMPLES_COUNT 20
#define LIMIT_SWITCH_DEBOUNCE_TIME 5
#define ESTOP_FILTER_MS 300
#define POS_REPORT_INTERVAL 100

// ==============================================================================
// 3. STRUKTURY
// ==============================================================================
typedef struct __attribute__((packed)) {
    uint8_t powergood_3v3;
    uint8_t powergood_5v;
    uint8_t power_ok;
    uint8_t power_stat;
    float temp1;
    float temp2;
    float temp3;
    float temp4;
} SystemStatus_t;

#define I2C_RX_SIZE sizeof(SystemStatus_t)

// ==============================================================================
// 4. DEKLARACJE EXTERN
// ==============================================================================

// Sprzęt
extern TMC5160_Driver tmc1, tmc2, tmc3, tmc4, tmc5, tmc6, tmc7, tmc8;
extern void* motors[6];
extern GPIO_TypeDef* limit_switch_ports[6];
extern const uint16_t limit_switch_pins[6];

// Zmienne procesowe
extern uint32_t position[9];
extern double ratios[6];
extern double realAngle[9];
extern float prev_angles[6];
extern uint32_t positionTimer;

// Diagnostyka
extern uint16_t SG_RESULT[9];
extern uint8_t STALL_GUARD[9];
extern uint8_t s_j1, s_j2, s_j3, s_j4, s_j5, s_j6;
extern uint8_t system_configured;
extern uint8_t reported_errors[6];


// Status i Bezpieczeństwo
extern uint8_t homing_command;
extern volatile uint8_t ESTOP_TRIGGER;
extern volatile uint8_t ESTOP_need_check;
extern uint8_t ESTOP_prev_physical;
extern uint32_t limitTimer;
extern uint32_t protTimer;
extern volatile uint8_t limit_switches_need_check;
extern uint32_t last_limit_check_time;

// Flagi Bazowania
extern uint8_t homed_j1g;
extern uint8_t homed_j2g;
extern uint8_t homed_j3g;
extern uint8_t homed_j4g;
extern uint8_t homed_j5g;
extern uint8_t homed_j6g;

// Komunikacja SPI
extern volatile uint8_t spi_ready;
extern uint8_t spi_tx_buf[5];
extern uint8_t spi_rx_buf[5];

// UART
extern uint8_t rx_buffer[RX_BUFFER_SIZE];
extern uint8_t received_data[RX_BUFFER_SIZE];
extern volatile uint8_t new_message_flag;
extern volatile uint8_t tx_buffer[TX_BUFFER_SIZE];
extern volatile uint16_t tx_head;
extern volatile uint16_t tx_tail;

// UART5
extern uint8_t rx_buffer5[RX_BUFFER_SIZE];
extern uint8_t received_data5[RX_BUFFER_SIZE];
extern volatile uint8_t new_message_flag5;
extern volatile uint8_t tx_buffer5[TX_BUFFER_SIZE];
extern volatile uint16_t tx_head5;
extern volatile uint16_t tx_tail5;

// I2C
extern uint8_t I2CrxBuffer[I2C_RX_SIZE];
extern SystemStatus_t currentStatus;
extern uint32_t last_pos_report_time;
extern uint8_t current_report_motor_id;
extern uint8_t last_sent_state[6];
extern char msg[128];

// Chwytak
extern volatile double cisnienie;
extern uint8_t cisnienie_stan;
extern uint32_t adc_timer;
extern uint8_t tool_changer_active;
extern uint8_t VGripStatus;

// --- TU TEŻ BYŁY KONFLIKTY ---
extern uint32_t EGRIP_PROCESS_TIMER;
extern uint16_t EGRIP_SG_RESULT;
extern float EGRIP_VALUE;
extern int EGRIP_IHOLD;
extern int EGRIP_IRUN;

// Rampy
extern uint32_t AMAX_S[9];
extern uint32_t VMAX_S[9];

// Wiadomości stałe
extern const uint8_t msg_h1[]; extern const uint8_t msg_r1[];
extern const uint8_t msg_h2[]; extern const uint8_t msg_r2[];
extern const uint8_t msg_h3[]; extern const uint8_t msg_r3[];
extern const uint8_t msg_h4[]; extern const uint8_t msg_r4[];
extern const uint8_t msg_h5[]; extern const uint8_t msg_r5[];
extern const uint8_t msg_h6[]; extern const uint8_t msg_r6[];

#endif /* INC_VARIABLES_GLOBAL_H_ */
