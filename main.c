#include "stm32f4xx_hal.h"
#include "bme280.h"
#include "usart.h"
#include "timer.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static bool send_measurements = false;

static void Send_Measurements(void)
{
    int32_t t_fine; /* temperature in special format needed in humidity and pressure compensation */
    char buffer[256];
    size_t bytes = 0;
    bytes = 0;

    bme280_measure();

    int32_t temp_C = bme280_temp_get(&t_fine);
    bytes += snprintf(&buffer[bytes], sizeof(buffer) - bytes, "Temperature: %d.%02d deg C\n", temp_C/100, abs(temp_C)%100);

    uint32_t press_hPa = bme280_press_get(&t_fine);
    bytes += snprintf(&buffer[bytes], sizeof(buffer) - bytes, "Absolute pressure: %d.%02d hPa \n", press_hPa/100, press_hPa%100);

    int32_t hum_RH = bme280_hum_get(&t_fine);
    bytes += snprintf(&buffer[bytes], sizeof(buffer) - bytes, "Humidity: %d %% RH \n", hum_RH);

    USART_WriteData(buffer, bytes);
}

void Timer_Callback(void)
{
    send_measurements = true;
}

void SysTick_Handler(void)
{
    HAL_IncTick();
}


int main(void)
{
    HAL_Init();
    USART_Init();
    bme280_init();
    TIM_Init(Timer_Callback);

    while (1)
    {
        if (send_measurements)
        {
            Send_Measurements();
            send_measurements = false;
        }
    }
} /* main */
