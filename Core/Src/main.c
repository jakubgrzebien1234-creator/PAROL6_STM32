/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tmc5160.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "homing.h"
#include "PAROL6.h"
#include "uart.h"
#include "startup.h"
#include "Variables_OT.h"
#include "Variables_GLOBAL.h"
#include "math.h"
#include "CONFIG.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// Prototypy wewnętrzne (pomocnicze)
void UART8_Send_Internal(const uint8_t* data);
void UART5_Send_Internal(const uint8_t* data);

extern void UART_Direct_Send(const uint8_t* data);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
void Parse_And_Execute_Command(uint8_t* data);
void SPI_WaitReady(void);
uint8_t send_message_dma(UART_HandleTypeDef* huart, const uint8_t* msg, uint16_t len);
void UART_Direct_Send(const uint8_t* data);
void Handle_Limit_Switches_Loop(void);
void Handle_ESTOP(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void CheckLimitSwitchesOnStartup(void)
{
    uint8_t* states[6] = {&s_j1, &s_j2, &s_j3, &s_j4, &s_j5, &s_j6};
    const uint8_t* msg_H[6] = {msg_h1, msg_h2, msg_h3, msg_h4, msg_h5, msg_h6};
    const uint8_t* msg_R[6] = {msg_r1, msg_r2, msg_r3, msg_r4, msg_r5, msg_r6};

    for (int i = 0; i < 6; i++)
    {
        GPIO_PinState pinState = HAL_GPIO_ReadPin(limit_switch_ports[i], limit_switch_pins[i]);

        if (pinState == GPIO_PIN_RESET)
        {
            *states[i] = 1;
            UART_Direct_Send(msg_H[i]);
        }
        else
        {
            *states[i] = 0;
            UART_Direct_Send(msg_R[i]);
        }
        HAL_Delay(10);
    }
}

void Handle_Position_Reporting(void)
{
    if (HAL_GetTick() - last_pos_report_time < 20) return;

    last_pos_report_time = HAL_GetTick();

    PAROL6_POSITION(current_report_motor_id);

    current_report_motor_id++;
    if (current_report_motor_id >= 6) {
        current_report_motor_id = 0;
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI3_Init();
  MX_UART8_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_I2C2_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
  ESTOP_prev_physical = (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_15) == GPIO_PIN_RESET) ? 1 : 0;
  ESTOP_TRIGGER = ESTOP_prev_physical;


  if(HAL_I2C_Slave_Receive_IT(&hi2c2, I2CrxBuffer, I2C_RX_SIZE) != HAL_OK)
  {
      Error_Handler();
  }


  HAL_ADC_Start(&hadc1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    HAL_Delay(500);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_SET);
    CheckLimitSwitchesOnStartup();

    if (HAL_UARTEx_ReceiveToIdle_IT(&huart8, rx_buffer, RX_BUFFER_SIZE) != HAL_OK)
      {
          Error_Handler();
      }


      if (HAL_UARTEx_ReceiveToIdle_IT(&huart5, rx_buffer5, RX_BUFFER_SIZE) != HAL_OK)
      {
          Error_Handler();
      }


    PAROL6_INIT();
    PAROL6_XACTUAL(0);
    PAROL6_EGRIP_INIT();

    uint8_t ESTOP_INITIAL = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_15);
    if(ESTOP_INITIAL == 0){
    	ESTOP_TRIGGER = 1;
    }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
      while (1)
      {
    	  if(ESTOP_Present){
          Handle_ESTOP();
    	  }
          if (new_message_flag)
          {
              __disable_irq();
              char temp[RX_BUFFER_SIZE];
              memset(temp, 0, RX_BUFFER_SIZE);
              memcpy(temp, received_data, RX_BUFFER_SIZE);
              temp[RX_BUFFER_SIZE - 1] = 0;
              new_message_flag = 0;
              __enable_irq();

              if (ESTOP_TRIGGER == 0 || ESTOP_Present == false)
              {
                  Parse_And_Execute_Command((uint8_t*)temp);
              }
          }
              if (new_message_flag5)
              {
                __disable_irq();
                char temp5[RX_BUFFER_SIZE];
                memset(temp5, 0, RX_BUFFER_SIZE);
                memcpy(temp5, received_data5, RX_BUFFER_SIZE);
                temp5[RX_BUFFER_SIZE - 1] = 0;
                new_message_flag5 = 0;
                __enable_irq();

                if (ESTOP_TRIGGER == 0 || ESTOP_Present == false)
                {
                	Parse_And_Execute_Command((uint8_t*)temp5);
                }


          }

          if (ESTOP_TRIGGER == 0 || ESTOP_Present == false)
          {
              HomeAll_Handler();


              if(system_configured == 0 || System_Configured_Command_Present == false)
              {
                  Handle_Limit_Switches_Loop();
                  if(Electric_Gripper_Present){
                  PAROL6_EGRIP_PROCESS();
                  }

                  if(HAL_GetTick() - limitTimer > 50){
                      limitTimer = HAL_GetTick();
                      PAROL6_LIMIT_SWITCH_STATUS();
                  }

                  if (homing_command)
                  {
                      HomeAll_Start();
                      homing_command = 0;
                  }

                  if(Vacuum_Sensor_Present){
                  if (cisnienie < 0.0f)
                  {
                      if (HAL_GetTick() - adc_timer >= 50)
                      {
                          adc_timer = HAL_GetTick();
                          HAL_ADC_Start_IT(&hadc1);

                          sprintf(msg, "P:%.2f\r\n", cisnienie);
                          UART_Direct_Send((uint8_t*)msg);
                      }
                      cisnienie_stan = 1;
                  }

                  else
                  {

                      if (cisnienie_stan == 1)
                      {
                          sprintf(msg, "P:0.0\r\n");
                          UART_Direct_Send((uint8_t*)msg);
                          cisnienie_stan = 0;
                      }

                      if (HAL_GetTick() - adc_timer >= 50)
                      {
                           adc_timer = HAL_GetTick();
                           HAL_ADC_Start_IT(&hadc1);
                      }
                  }

              }

                  if (tool_changer_active && Tool_Change_Present) {
                      PAROL6_TOOL_CHANGE();
                  }

                  if(VGripStatus && Vacuum_Gripper_Present)
                  {
                       __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 500);
                       if(cisnienie > PUMP_ON_PRESSURE)      HAL_GPIO_WritePin(PUMP_GPIO_Port, PUMP_Pin, GPIO_PIN_SET);
                       else if(cisnienie < PUMP_OFF_PRESSURE) HAL_GPIO_WritePin(PUMP_GPIO_Port, PUMP_Pin, GPIO_PIN_RESET);
                  }
                  else
                  {
                       HAL_GPIO_WritePin(PUMP_GPIO_Port, PUMP_Pin, GPIO_PIN_RESET);
                       __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
                  }



                 if(STM_Protect_Present){
                  if(HAL_GetTick() - protTimer > 100)
                  {
                      protTimer = HAL_GetTick();
                      static SystemStatus_t lastSentStatus = {0};

                      SystemStatus_t statusSnapshot;

                      __disable_irq();
                      memcpy(&statusSnapshot, &currentStatus, sizeof(SystemStatus_t));
                      __enable_irq();

                      uint8_t shouldSend = 0;

                      if (statusSnapshot.powergood_3v3 != lastSentStatus.powergood_3v3 ||
                          statusSnapshot.powergood_5v  != lastSentStatus.powergood_5v  ||
                          statusSnapshot.power_ok      != lastSentStatus.power_ok      ||
                          statusSnapshot.power_stat    != lastSentStatus.power_stat)
                      {
                          shouldSend = 1;
                      }

                      if (!shouldSend)
                      {
                          if (fabsf(statusSnapshot.temp1 - lastSentStatus.temp1) > TEMP_THRESHOLD ||
                              fabsf(statusSnapshot.temp2 - lastSentStatus.temp2) > TEMP_THRESHOLD ||
                              fabsf(statusSnapshot.temp3 - lastSentStatus.temp3) > TEMP_THRESHOLD ||
                              fabsf(statusSnapshot.temp4 - lastSentStatus.temp4) > TEMP_THRESHOLD)
                          {
                              shouldSend = 1;
                          }
                      }

                      if (shouldSend)
                      {
                          char msg[128];
                          sprintf(msg, "PROT_%d,%d,%d,%d,%.2f,%.2f,%.2f,%.2f\r\n",
                                  statusSnapshot.powergood_3v3,
                                  statusSnapshot.powergood_5v,
                                  statusSnapshot.power_ok,
                                  statusSnapshot.power_stat,
                                  statusSnapshot.temp1,
                                  statusSnapshot.temp2,
                                  statusSnapshot.temp3,
                                  statusSnapshot.temp4
                                 );
                          UART_Direct_Send((uint8_t*)msg);


                          memcpy(&lastSentStatus, &statusSnapshot, sizeof(SystemStatus_t));
                      }

                      PAROL6_MOTOR_CONNECTED();

                  }
                  }
                  if(HAL_GetTick() - positionTimer >= 50) {
                      positionTimer = HAL_GetTick();
                      float angles[6];
                      uint8_t data_changed = 0;
                      for(int i=0; i<6; i++){
                          int32_t raw_pos = TMC5160_ReadRegister((TMC5160_Driver*)motors[i], 0x21);
                          angles[i] = GetRotationAngle((TMC5160_Driver*)motors[i], ratios[i]);
                          if(fabs(angles[i] - prev_angles[i]) > 0.01f) {
                              data_changed = 1;
                          }
                      }
                      if(data_changed) {
                          char big_msg[128];
                          sprintf(big_msg, "A_%.2f_%.2f_%.2f_%.2f_%.2f_%.2f\r\n",
                                  angles[0], angles[1], angles[2],
                                  angles[3], angles[4], angles[5]);
                          UART_Direct_Send((uint8_t*)big_msg);
                          for(int i=0; i<6; i++) prev_angles[i] = angles[i];
                      }
                  }
              }
          }

          else
          {
              HAL_Delay(10);
          }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void UART_Direct_Send(const uint8_t* data)
{
    UART8_Send_Internal(data);
    UART5_Send_Internal(data);
}

void UART_TX_Empty_Handler(void)
{
    if (tx_head != tx_tail)
    {
        UART8->TDR = tx_buffer[tx_tail];
        tx_tail = (tx_tail + 1) % TX_BUFFER_SIZE;
    }
    else
    {
        __HAL_UART_DISABLE_IT(&huart8, UART_IT_TXE);
    }
}

void UART5_TX_Empty_Handler(void)
{
    if (tx_head5 != tx_tail5)
    {
        UART5->TDR = tx_buffer5[tx_tail5];
        tx_tail5 = (tx_tail5 + 1) % TX_BUFFER_SIZE;
    }
    else
    {
        __HAL_UART_DISABLE_IT(&huart5, UART_IT_TXE);
    }
}

void Process_Pin_State(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, const uint8_t* msg_high, const uint8_t* msg_low, uint8_t* state_variable)
{
    if (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_SET)
    {
        UART_Direct_Send(msg_low);
        *state_variable = 0;
    }
    else
    {
        UART_Direct_Send(msg_high);
        *state_variable = 1;
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_15)
    {
        ESTOP_need_check = 1;
    }
    else
    {
        limit_switches_need_check = 1;
    }
}


void Handle_Limit_Switches_Loop(void)
{
    if (!limit_switches_need_check && (HAL_GetTick() - last_limit_check_time < 5)) return;

    last_limit_check_time = HAL_GetTick();
    limit_switches_need_check = 0;

    GPIO_TypeDef* ports[6] = {GPIOB, GPIOA, GPIOA, GPIOB, GPIOC, GPIOC};
    uint16_t pins[6] = {GPIO_PIN_1, GPIO_PIN_6, GPIO_PIN_7, GPIO_PIN_0, GPIO_PIN_4, GPIO_PIN_5};
    uint8_t* states[6] = {&s_j1, &s_j2, &s_j3, &s_j4, &s_j5, &s_j6};
    const uint8_t* msg_H[6] = {msg_h1, msg_h2, msg_h3, msg_h4, msg_h5, msg_h6};
    const uint8_t* msg_R[6] = {msg_r1, msg_r2, msg_r3, msg_r4, msg_r5, msg_r6};

    static uint32_t last_debounce_time[6] = {0};
    static uint8_t  last_raw_state[6] = {0};

    for(int i=0; i<6; i++)
    {
        uint8_t current_reading = (HAL_GPIO_ReadPin(ports[i], pins[i]) == GPIO_PIN_RESET) ? 1 : 0;

        if (current_reading != last_raw_state[i])
        {
            last_debounce_time[i] = HAL_GetTick();
        }
        last_raw_state[i] = current_reading;

        if ((HAL_GetTick() - last_debounce_time[i]) > LIMIT_SWITCH_DEBOUNCE_TIME)
        {
            if (current_reading != *states[i])
            {
                *states[i] = current_reading;
                if (current_reading == 1) UART_Direct_Send(msg_H[i]);
                else                      UART_Direct_Send(msg_R[i]);
                last_sent_state[i] = current_reading;
            }
        }
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == UART8)
    {
        if (Size > 0)
        {
            if (Size > RX_BUFFER_SIZE - 1) Size = RX_BUFFER_SIZE - 1;
            memcpy(received_data, rx_buffer, Size);
            received_data[Size] = '\0';
            new_message_flag = 1;
        }
        memset(rx_buffer, 0, RX_BUFFER_SIZE);
        HAL_UARTEx_ReceiveToIdle_IT(&huart8, rx_buffer, RX_BUFFER_SIZE);
    }
    else if (huart->Instance == UART5)
    {
        if (Size > 0)
        {
            if (Size > RX_BUFFER_SIZE - 1) Size = RX_BUFFER_SIZE - 1;
            memcpy(received_data5, rx_buffer5, Size);
            received_data5[Size] = '\0';
            new_message_flag5 = 1;
        }
        memset(rx_buffer5, 0, RX_BUFFER_SIZE);
        HAL_UARTEx_ReceiveToIdle_IT(&huart5, rx_buffer5, RX_BUFFER_SIZE);
    }
}


void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART8)
    {
        HAL_UART_Abort(huart);
        HAL_UART_AbortReceive(huart);
        HAL_UART_AbortTransmit(huart);
        HAL_UART_DeInit(huart);
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        MX_UART8_Init();
        memset(rx_buffer, 0, RX_BUFFER_SIZE);
        HAL_UARTEx_ReceiveToIdle_IT(&huart8, rx_buffer, RX_BUFFER_SIZE);
    }
    else if (huart->Instance == UART5)
    {
        HAL_UART_Abort(huart);
        HAL_UART_AbortReceive(huart);
        HAL_UART_AbortTransmit(huart);
        HAL_UART_DeInit(huart);
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        MX_UART5_Init();
        memset(rx_buffer5, 0, RX_BUFFER_SIZE);
        HAL_UARTEx_ReceiveToIdle_IT(&huart5, rx_buffer5, RX_BUFFER_SIZE);
    }
}


void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == SPI3) spi_ready = 1;
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == SPI3) spi_ready = 1;
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        static uint32_t adc_accumulator = 0;
        static uint8_t adc_sample_idx = 0;

        adc_accumulator += HAL_ADC_GetValue(hadc);
        adc_sample_idx++;

        if (adc_sample_idx >= ADC_SAMPLES_COUNT)
        {
            uint32_t adc_avg = adc_accumulator / ADC_SAMPLES_COUNT;
            const float VS = 5.0f;
            float vout = (float)adc_avg * VS / 65535.0f;
            float raw_pressure = (vout / VS - 0.92f) / 0.007652f;

            if (raw_pressure > -0.03f)
            {
                cisnienie = 0.0f;
            }
            else
            {
                cisnienie = raw_pressure;
            }

            adc_accumulator = 0;
            adc_sample_idx = 0;
        }
    }
}

void SPI_WaitReady(void)
{
    while (!spi_ready) {}
}


void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C2)
    {
        memcpy(&currentStatus, I2CrxBuffer, I2C_RX_SIZE);
        HAL_I2C_Slave_Receive_IT(&hi2c2, I2CrxBuffer, I2C_RX_SIZE);
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C2)
    {
        __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_AF | I2C_FLAG_BERR | I2C_FLAG_OVR | I2C_FLAG_ARLO);
        HAL_I2C_Slave_Receive_IT(&hi2c2, I2CrxBuffer, I2C_RX_SIZE);
    }
}



void Handle_ESTOP(void)
{
    static uint32_t last_stable_time = 0;
    static uint8_t  last_raw_state = 0;
    static uint8_t  current_stable_state = 255;

    uint8_t raw_state = (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_15) == GPIO_PIN_RESET) ? 1 : 0;

    if (current_stable_state == 255) {
        current_stable_state = raw_state;
        ESTOP_prev_physical = raw_state;
        last_raw_state = raw_state;
        last_stable_time = HAL_GetTick();
        return;
    }

    if (raw_state != last_raw_state) {
        last_stable_time = HAL_GetTick();
    }
    last_raw_state = raw_state;

    if ((HAL_GetTick() - last_stable_time) > ESTOP_FILTER_MS)
    {
        if (raw_state != current_stable_state)
        {
            current_stable_state = raw_state;

            if (current_stable_state == 1)
            {
                ESTOP_TRIGGER = 1;
                UART_Direct_Send((uint8_t*)"ESTOP_TRIGGER\r\n");
                PAROL6_EMERGENCY_STOP();
            }
            else
            {
                ESTOP_TRIGGER = 0;
                UART_Direct_Send((uint8_t*)"ESTOP_RELEASE\r\n");
                PAROL6_RESUME();
            }
            ESTOP_prev_physical = current_stable_state;
        }
    }
    ESTOP_need_check = 0;
}

void UART8_Send_Internal(const uint8_t* data)
{
    while (*data)
    {
        uint16_t next_head = (tx_head + 1) % TX_BUFFER_SIZE;
        if (next_head != tx_tail)
        {
            tx_buffer[tx_head] = *data;
            tx_head = next_head;
        }
        data++;
    }
    __HAL_UART_ENABLE_IT(&huart8, UART_IT_TXE);
}

void UART5_Send_Internal(const uint8_t* data)
{
    while (*data)
    {
        uint16_t next_head = (tx_head5 + 1) % TX_BUFFER_SIZE;
        if (next_head != tx_tail5)
        {
            tx_buffer5[tx_head5] = *data;
            tx_head5 = next_head;
        }
        data++;
    }
    __HAL_UART_ENABLE_IT(&huart5, UART_IT_TXE);
}


/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  * where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* Możesz dodać printf tutaj do debugowania */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
