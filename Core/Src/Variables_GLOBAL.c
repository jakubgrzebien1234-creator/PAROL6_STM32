/*
 * Variables_GLOBAL.c
 */

#include "Variables_GLOBAL.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

// --- Definicje zmiennych ---

// Sprzęt
TMC5160_Driver tmc1 = { GPIOG, GPIO_PIN_6 };
TMC5160_Driver tmc2 = { GPIOG, GPIO_PIN_7 };
TMC5160_Driver tmc3 = { GPIOG, GPIO_PIN_8 };
TMC5160_Driver tmc4 = { GPIOC, GPIO_PIN_6 };
TMC5160_Driver tmc5 = { GPIOA, GPIO_PIN_9 };
TMC5160_Driver tmc6 = { GPIOA, GPIO_PIN_10 };
TMC5160_Driver tmc7 = { GPIOA, GPIO_PIN_11 };
TMC5160_Driver tmc8 = { GPIOA, GPIO_PIN_12 };

void* motors[6] = {&tmc1, &tmc2, &tmc3, &tmc4, &tmc5, &tmc8};

GPIO_TypeDef* limit_switch_ports[6] = {GPIOB, GPIOA, GPIOA, GPIOB, GPIOC, GPIOC};
const uint16_t limit_switch_pins[6] = {GPIO_PIN_1, GPIO_PIN_6, GPIO_PIN_7, GPIO_PIN_0, GPIO_PIN_4, GPIO_PIN_5};

// Zmienne procesowe
uint32_t position[9];
double ratios[6] = {6.4, 20.0, 18.0952381, 4.0, 4.0, 10.0};
double realAngle[9];
float prev_angles[6] = {0.0f};
uint32_t positionTimer = 0;

// Diagnostyka
uint8_t s_j1 = 0, s_j2 = 0, s_j3 = 0, s_j4 = 0, s_j5 = 0, s_j6 = 0;
uint8_t system_configured = 0;
uint8_t reported_errors[6] = {0};


// Status i Bezpieczeństwo
uint8_t homing_command = 0;
volatile uint8_t ESTOP_TRIGGER = 0;
volatile uint8_t ESTOP_need_check = 0;
uint8_t ESTOP_prev_physical = 0;
uint32_t limitTimer = 0;
uint32_t protTimer = 0;
volatile uint8_t limit_switches_need_check = 0;
uint32_t last_limit_check_time = 0;

// Flagi Bazowania
uint8_t homed_j1g = 0;
uint8_t homed_j2g = 0;
uint8_t homed_j3g = 0;
uint8_t homed_j4g = 0;
uint8_t homed_j5g = 0;
uint8_t homed_j6g = 0;

// SPI
volatile uint8_t spi_ready = 1;
uint8_t spi_tx_buf[5];
uint8_t spi_rx_buf[5];

// UART
uint8_t rx_buffer[RX_BUFFER_SIZE];
uint8_t received_data[RX_BUFFER_SIZE];
volatile uint8_t new_message_flag = 0;
volatile uint8_t tx_buffer[TX_BUFFER_SIZE];
volatile uint16_t tx_head = 0;
volatile uint16_t tx_tail = 0;

// UART5
uint8_t rx_buffer5[RX_BUFFER_SIZE];
uint8_t received_data5[RX_BUFFER_SIZE];
volatile uint8_t new_message_flag5 = 0;
volatile uint8_t tx_buffer5[TX_BUFFER_SIZE];
volatile uint16_t tx_head5 = 0;
volatile uint16_t tx_tail5 = 0;

// I2C
uint8_t I2CrxBuffer[I2C_RX_SIZE] __attribute__((aligned(32)));
SystemStatus_t currentStatus;
uint32_t last_pos_report_time = 0;
uint8_t current_report_motor_id = 0;
uint8_t last_sent_state[6] = {0};
char msg[128];

// Chwytak
volatile double cisnienie = 0;
uint8_t cisnienie_stan = 0;
uint32_t adc_timer = 0;
uint8_t tool_changer_active = 0;
uint8_t VGripStatus = 0;
uint32_t EGRIP_PROCESS_TIMER = 0;
uint16_t EGRIP_SG_RESULT = 0;



// Wiadomości stałe
const uint8_t msg_h1[] = "$H1\n"; const uint8_t msg_r1[] = "$R1\n";
const uint8_t msg_h2[] = "$H2\n"; const uint8_t msg_r2[] = "$R2\n";
const uint8_t msg_h3[] = "$H3\n"; const uint8_t msg_r3[] = "$R3\n";
const uint8_t msg_h4[] = "$H4\n"; const uint8_t msg_r4[] = "$R4\n";
const uint8_t msg_h5[] = "$H5\n"; const uint8_t msg_r5[] = "$R5\n";
const uint8_t msg_h6[] = "$H6\n"; const uint8_t msg_r6[] = "$R6\n";
