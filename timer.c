#include "timer.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_rcc.h"

/* Private definitions -------------------------------------------------------*/

/* Definition for TIMx clock resources */
#define TIMx                             TIM6
#define TIMx_CLK_ENABLE()                __TIM6_CLK_ENABLE()

#define TIMx_FORCE_RESET()               __TIM6_FORCE_RESET()
#define TIMx_RELEASE_RESET()             __TIM6_RELEASE_RESET()

/* Definition for TIMx's NVIC */
#define TIMx_IRQn                        TIM6_DAC_IRQn
#define TIMx_IRQHandler                  TIM6_DAC_IRQHandler

/* Private variables ---------------------------------------------------------*/

/* TIM descriptor */
static TIM_HandleTypeDef TimHandle;
static timer_callback_t m_callback;


void TIMx_IRQHandler(void)
{
    __HAL_TIM_SET_AUTORELOAD(&TimHandle, 0x2710);
    if (__HAL_TIM_GET_FLAG(&TimHandle, TIM_FLAG_UPDATE))
    {
        if (__HAL_TIM_GET_IT_SOURCE(&TimHandle, TIM_IT_UPDATE))
        {
            m_callback();
        }
        __HAL_TIM_CLEAR_IT(&TimHandle, TIM_IT_UPDATE);
    }
}

/**
  * This function configures the hardware resources used in this example: 
  *  - Peripheral's clock enable
  *  - NVIC configuration for TIM interrupt request enable
  */
void HAL_TIM_MspInit(TIM_HandleTypeDef *htim)
{
    /* Enable TIM6 clock */
    TIMx_CLK_ENABLE(); 

    /* Configure the NVIC for TIM */
    /* NVIC for TIM6 */
    HAL_NVIC_SetPriority(TIMx_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(TIMx_IRQn);
}

/**
  * This function frees the hardware resources used in this example:
  *   - Disable the Peripheral's clock
  *   - Revert NVIC configuration to their default state
  */
void HAL_TIM_MspDeInit(TIM_HandleTypeDef *htim)
{
    /* Reset peripherals */
    TIMx_FORCE_RESET();
    TIMx_RELEASE_RESET();

    /* Disable the NVIC for USART */
    HAL_NVIC_DisableIRQ(TIMx_IRQn);
}

bool TIM_Init(timer_callback_t callback)
{
    /* configure TIM */
    TimHandle.Instance                 = TIMx;

    TimHandle.Init.AutoReloadPreload   = TIM_AUTORELOAD_PRELOAD_ENABLE;
    TimHandle.Init.Period              = 0x2710;
    TimHandle.Init.Prescaler           = 0x3E80;
    TimHandle.Channel                  = HAL_TIM_ACTIVE_CHANNEL_1;

    HAL_TIM_MspInit(&TimHandle);

    if(HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
    {
        return false;
    }

    m_callback = callback;
    HAL_TIM_Base_Start_IT(&TimHandle); 

    return true;
}
