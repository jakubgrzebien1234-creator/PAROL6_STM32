#ifndef TMC5160_H_
#define TMC5160_H_

#include "main.h"


typedef struct {
    GPIO_TypeDef* cs_port;
    uint16_t cs_pin;
} TMC5160_Driver;

extern TMC5160_Driver tmc1;
extern TMC5160_Driver tmc2;
extern TMC5160_Driver tmc3;
extern TMC5160_Driver tmc4;
extern TMC5160_Driver tmc5;
extern TMC5160_Driver tmc6;
extern TMC5160_Driver tmc7;
extern TMC5160_Driver tmc8;

extern SPI_HandleTypeDef hspi3;

void TMC5160_WriteRegister(TMC5160_Driver* driver, uint8_t address, int32_t value);
uint32_t TMC5160_ReadRegister(TMC5160_Driver* driver, uint8_t address);

void SetRotationAngle(TMC5160_Driver* driver, uint8_t ratio, double angle_deg);
void TMC5160_Velocity_Right(TMC5160_Driver* driver);
void TMC5160_Velocity_Left(TMC5160_Driver* driver);
uint32_t makeIHOLD_IRUN(int ihold, int irun, int iholddelay);
void TMC5160_Init(TMC5160_Driver* driver,
                  uint8_t ihold, uint8_t irun, uint8_t iholddelay,
                  uint8_t stall, uint16_t A1, uint32_t V1,
                  uint32_t AMAX, uint32_t VMAX,
                  uint16_t DMAX, uint16_t D1);


void TMC5160_SetVelocity(TMC5160_Driver* driver, int32_t velocity);
void TMC5160_SetAcceleration(TMC5160_Driver* driver, int32_t acceleration);
void TMC5160_SendPositions(void);
double GetRotationAngle(TMC5160_Driver* driver, double ratio);
extern void TMC5160_Set_StallGuard_With_Filter(TMC5160_Driver* drv, int8_t sgt_value);
#endif /* TMC5160_H_ */
