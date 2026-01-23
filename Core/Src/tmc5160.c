#include "main.h"
#include "tmc5160.h"
#include <stdlib.h>
#include <stdint.h> // dla int32_t/int64_t itp.
#include "string.h"
#include "stdio.h"
#include "Variables_GLOBAL.h"
#include "Variables_OT.h"
extern UART_HandleTypeDef huart8;


#define MAX_VELOCITY 1000   // w jednostkach TMC5160 (np. krok/s)
#define MAX_ACCEL    5000   // w jednostkach TMC5160 (np. krok/s^2)

extern SPI_HandleTypeDef hspi3;


uint32_t makeIHOLD_IRUN(int ihold, int irun, int iholddelay);

void TMC5160_Select(TMC5160_Driver* driver) {
    if (driver == NULL || driver->cs_port == NULL) return; // ZABEZPIECZENIE
    HAL_GPIO_WritePin(driver->cs_port, driver->cs_pin, GPIO_PIN_RESET);
    for(volatile int i = 0; i < 500; i++) { __NOP(); }
}

void TMC5160_Deselect(TMC5160_Driver* driver) {
    HAL_GPIO_WritePin(driver->cs_port, driver->cs_pin, GPIO_PIN_SET);
}

void TMC5160_WriteRegister(TMC5160_Driver* driver, uint8_t address, int32_t value)
{
    // Przygotuj dane
    spi_tx_buf[0] = address | 0x80;
    spi_tx_buf[1] = (value >> 24) & 0xFF;
    spi_tx_buf[2] = (value >> 16) & 0xFF;
    spi_tx_buf[3] = (value >> 8) & 0xFF;
    spi_tx_buf[4] = value & 0xFF;

    SPI_WaitReady();
    spi_ready = 0;
    TMC5160_Select(driver);
    HAL_SPI_Transmit_DMA(&hspi3, spi_tx_buf, 5);
    SPI_WaitReady();
    TMC5160_Deselect(driver);
}

uint32_t TMC5160_ReadRegister(TMC5160_Driver* driver, uint8_t address)
{
    uint32_t value = 0;
    uint8_t tx_buf[5] = {0};
    uint8_t rx_buf[5] = {0};

    // Adres odczytu (MSB musi być 0 dla odczytu w TMC5160, ale adresy w .h są zazwyczaj poprawne)
    // Z manuala: "For a read access, the address bit 7 must be 0"
    tx_buf[0] = address & 0x7F;

    // --- KROK 1: Wyślij adres rejestru ---
    TMC5160_Select(driver); // CS LOW

    // Używamy zwykłego TransmitReceive z timeoutem 10ms (blokująco)
    HAL_SPI_TransmitReceive(&hspi3, tx_buf, rx_buf, 5, 10);

    TMC5160_Deselect(driver); // CS HIGH

    // Krótkie opóźnienie (czasami potrzebne dla TMC, żeby przygotował dane)
    // Wystarczy kilka cykli zegara, HAL_GPIO... zajmuje wystarczająco czasu.
    // Ewentualnie: for(volatile int i=0; i<10; i++);

    // --- KROK 2: Odczytaj dane (Wysyłając dummy bytes) ---
    // Zerujemy bufor nadawczy (same 0x00)
    memset(tx_buf, 0, 5);

    TMC5160_Select(driver); // CS LOW

    // Teraz zegar wypycha dane z TMC do STM
    HAL_SPI_TransmitReceive(&hspi3, tx_buf, rx_buf, 5, 10);

    TMC5160_Deselect(driver); // CS HIGH

    // --- Składanie wyniku ---
    // TMC zwraca: [Status] [Byte3] [Byte2] [Byte1] [Byte0]
    // Więc dane są w indeksach 1, 2, 3, 4
    value = ((uint32_t)rx_buf[1] << 24) |
            ((uint32_t)rx_buf[2] << 16) |
            ((uint32_t)rx_buf[3] << 8) |
            (uint32_t)rx_buf[4];

    return value;
}

// =======================================================
//  Inicjalizacja TMC5160 - ustawienia wykonywane raz lub rzadko
// =======================================================
void TMC5160_Init(TMC5160_Driver* driver,uint8_t ihold, uint8_t irun, uint8_t iholddelay, uint8_t stall, uint16_t A1, uint32_t V1,uint32_t AMAX, uint32_t VMAX,uint16_t DMAX, uint16_t D1)
{
    // CHOPCONF - konfiguracja choppera (microstepping, blank time, itd.)
    TMC5160_WriteRegister(driver, 0x6C, 0x000100C3);

    // Ustawienia prądu silnika
    TMC5160_WriteRegister(driver, 0x10, makeIHOLD_IRUN(ihold, irun, iholddelay));

    // PWM MODE
    TMC5160_WriteRegister(driver, 0x00, 0x00000004);

    // TPWMTHRS - próg przejścia na sterowanie silentStep
    TMC5160_WriteRegister(driver, 0x13, 500);

    // STALLGUARD (opcjonalne)
    // TMC5160_WriteRegister(driver, 0x6D, ((uint32_t)(stall & 0x7F)) << 16);

    // Ustawienia ramp
    TMC5160_WriteRegister(driver, 0x24, (uint32_t)A1);   // A1: przyspieszenie początkowe
    TMC5160_WriteRegister(driver, 0x25, V1);             // V1: prędkość startowa
    TMC5160_WriteRegister(driver, 0x26, AMAX);           // AMAX: max przyspieszenie
    TMC5160_WriteRegister(driver, 0x27, VMAX);           // VMAX: max prędkość
    TMC5160_WriteRegister(driver, 0x28, (uint32_t)DMAX); // DMAX: max hamowanie
    TMC5160_WriteRegister(driver, 0x2A, (uint32_t)D1);   // D1: hamowanie początkowe
    TMC5160_WriteRegister(driver, 0x2B, 10);             // VSTOP: minimalna prędkość końcowa

    // RAMP MODE – pozycjonowanie (mode = 0)
    TMC5160_WriteRegister(driver, 0x20, 0);
}

// =======================================================
//  Ustawienie kąta obrotu w stopniach
// =======================================================
void SetRotationAngle(TMC5160_Driver* driver, uint8_t ratio, double angle_deg)
{
    int64_t tmp = (int64_t)(51200.0 * angle_deg * ratio);
    int32_t target = (int32_t)(tmp / 360.0);
    TMC5160_WriteRegister(driver, 0x2D, (uint32_t)target);
}

double GetRotationAngle(TMC5160_Driver* driver, double ratio)
{
    // 1. Odczyt rejestru XACTUAL (0x21)
    // Rzutujemy na int32_t, ponieważ pozycja może być ujemna (np. -500 kroków)
    int32_t xactual = (int32_t)TMC5160_ReadRegister(driver, 0x21);

    // 2. Zabezpieczenie przed dzieleniem przez zero (gdyby ratio było 0)
    if (ratio == 0.0) return 0.0;

    // 3. Obliczenie kąta
    // Wzór: Angle = (Steps * 360) / (MicrostepsPerRev * Ratio)
    double angle = ((double)xactual * 360.0) / (51200.0 * ratio);

    return angle;
}


uint32_t makeIHOLD_IRUN(int ihold, int irun, int iholddelay)
{
    uint32_t reg = 0;
    reg |= (uint32_t)(ihold & 0x1F);              // bits 0-4
    reg |= (uint32_t)(irun & 0x1F) << 8;         // bits 8-12
    reg |= (uint32_t)(iholddelay & 0x0F) << 16;  // bits 16-19
    return reg;
}


void TMC5160_SetVelocity(TMC5160_Driver* driver, int32_t velocity){
	TMC5160_WriteRegister(driver, 0x27, velocity);
}

void TMC5160_SetAcceleration(TMC5160_Driver* driver, int32_t acceleration){
	TMC5160_WriteRegister(driver, 0x26, acceleration);
}



void TMC5160_SendPositions(void)
{
    // Użyj 'static', aby nie alokować bufora na stosie co 100ms
    static char msg[128];

    // Odczyt aktualnych pozycji
    int32_t pos1 = TMC5160_ReadRegister(&tmc1, 0x21); // XACTUAL
    int32_t pos2 = TMC5160_ReadRegister(&tmc2, 0x21);
    int32_t pos3 = TMC5160_ReadRegister(&tmc3, 0x21);
    int32_t pos4 = TMC5160_ReadRegister(&tmc4, 0x21);
    int32_t pos5 = TMC5160_ReadRegister(&tmc5, 0x21);
    int32_t pos6 = TMC5160_ReadRegister(&tmc8, 0x21);


    snprintf(msg, sizeof(msg),
             "POS j1=%ld j2=%ld j3=%ld j4=%ld j5=%ld j6=%ld\r\n",
             pos1, pos2, pos3, pos4, pos5, pos6);

    // <-- KLUCZOWA ZMIANA: Użyj tej samej bezpiecznej funkcji, co przerwania
    send_message_blocking(&huart8, (uint8_t*)msg, strlen(msg));
}


void TMC5160_Set_StallGuard_With_Filter(TMC5160_Driver* drv, int8_t sgt_value)
{
    // 1. ODCZYTAJ GCONF (Global Configuration)
    uint32_t gconf = TMC5160_ReadRegister(drv, 0x00);

    // 2. WYMUŚ SPREADCYCLE
    // StallGuard2 działa TYLKO w SpreadCycle.
    // Jeśli bit 2 (en_pwm_mode) jest 1 (StealthChop), StallGuard zwraca 0.
    // Zerujemy ten bit.
    if (gconf & (1UL << 2))
    {
        gconf &= ~(1UL << 2);
        TMC5160_WriteRegister(drv, 0x00, gconf);
    }

    // 3. USTAW TCOOLTHRS (Prędkość minimalna dla SG)
    // Poniżej tej prędkości (ok. 2000 w zależności od mikrokroków) sterownik
    // wyłącza diagnostykę, żeby nie wykrywać fałszywych utyków przy ruszaniu.
    // Jeśli testujesz bardzo wolno, zmniejsz to na 1000.
    TMC5160_WriteRegister(drv, 0x14, 2000);

    // 4. KONFIGURACJA COOLCONF (SGT + FILTR)
    uint32_t coolconf_reg = 0;

    // Ustawienie wartości SGT (bity 16..22)
    // SGT to liczba ze znakiem 7-bitowa.
    coolconf_reg |= (((uint32_t)sgt_value) << 16) & 0x007F0000;

    // Zapis do sterownika
    TMC5160_WriteRegister(drv, 0x6D, coolconf_reg);
}

