#include "stm32f4xx_hal.h"
#include "bme280.h"
#include <stdio.h>
#include <string.h>


int main(void)
{
    HAL_Init();
    bme280_init();

    while (1)
    {

    }
} /* main */
