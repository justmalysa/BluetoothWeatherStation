#include "i2c.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_rcc.h"

/* Private definitions -------------------------------------------------------*/

/* Definition for I2Cx clock resources */
#define I2Cx                            I2C1
#define I2Cx_CLK_ENABLE()               __I2C1_CLK_ENABLE();
#define I2Cx_SDA_GPIO_CLK_ENABLE()      __GPIOB_CLK_ENABLE()
#define I2Cx_SCL_GPIO_CLK_ENABLE()      __GPIOB_CLK_ENABLE() 

#define I2Cx_FORCE_RESET()              __I2C1_FORCE_RESET()
#define I2Cx_RELEASE_RESET()            __I2C1_RELEASE_RESET()

/* Definition for I2Cx Pins */
#define I2Cx_SDA_PIN                    GPIO_PIN_9
#define I2Cx_SDA_GPIO_PORT              GPIOB
#define I2Cx_SDA_AF                     GPIO_AF4_I2C1
#define I2Cx_SCL_PIN                    GPIO_PIN_8
#define I2Cx_SCL_GPIO_PORT              GPIOB
#define I2Cx_SCL_AF                     GPIO_AF4_I2C1

/* Definition for I2Cx's NVIC */
#define I2Cx_IRQn                       I2C1_EV_IRQn
#define I2Cx_IRQHandler                 I2C1_EV_IRQHandler

// I2C descriptor
static I2C_HandleTypeDef I2CHandle;
// I2C DMA descriptors
static DMA_HandleTypeDef DMAHandle_TX;
static DMA_HandleTypeDef DMAHandle_RX;

static I2C_Event_Callback m_callback;

/**
  * This function configures the hardware resources used in this example: 
  *  - Peripheral's clock enable
  *	 - Peripheral's GPIO Configuration  
  *  - NVIC configuration for I2C interrupt request enable
  */
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
    GPIO_InitTypeDef  GPIO_InitStruct;

    /*##-1- Enable peripherals and GPIO Clocks #################################*/
    /* Enable GPIO SDA/SCL clock */
    I2Cx_SDA_GPIO_CLK_ENABLE();
    I2Cx_SCL_GPIO_CLK_ENABLE();
    /* Enable I2C1 clock */
    I2Cx_CLK_ENABLE(); 

    /*##-2- Configure peripheral GPIO ##########################################*/  
    /* I2C SDA GPIO pin configuration  */
    GPIO_InitStruct.Pin       = I2Cx_SDA_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
    GPIO_InitStruct.Alternate = I2Cx_SCL_AF;

    HAL_GPIO_Init(I2Cx_SDA_GPIO_PORT, &GPIO_InitStruct);

    /* I2C SCL GPIO pin configuration  */
    GPIO_InitStruct.Pin = I2Cx_SCL_PIN;
    GPIO_InitStruct.Alternate = I2Cx_SCL_AF;

    HAL_GPIO_Init(I2Cx_SCL_GPIO_PORT, &GPIO_InitStruct);

    /*##-3- Configure the NVIC for I2C ########################################*/
    /* NVIC for I2C1 */
    HAL_NVIC_SetPriority(I2Cx_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(I2Cx_IRQn);
}

/**
  * This function frees the hardware resources used in this example:
  *   - Disable the Peripheral's clock
  *   - Revert GPIO and NVIC configuration to their default state  
  */
void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c)
{
    /*##-1- Reset peripherals ##################################################*/
    I2Cx_FORCE_RESET();
    I2Cx_RELEASE_RESET();

    /*##-2- Disable peripherals and GPIO Clocks ################################*/
    /* Configure I2C SDA as alternate function */
    HAL_GPIO_DeInit(I2Cx_SDA_GPIO_PORT, I2Cx_SDA_PIN);
    /* Configure I2C SCL as alternate function */
     HAL_GPIO_DeInit(I2Cx_SCL_GPIO_PORT, I2Cx_SCL_PIN);

    /*##-3- Disable the NVIC for I2C ##########################################*/
    HAL_NVIC_DisableIRQ(I2Cx_IRQn);
}

void DMA1_Stream5_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&DMAHandle_RX);
}

void DMA1_Stream6_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&DMAHandle_TX);
}

void I2C1_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&I2CHandle);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    I2C_Event_t event;
    event.type = I2C_EVENT_RX;
    event.p_data = hi2c->pBuffPtr;
    event.size = hi2c->XferSize;

    m_callback(&event);
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    I2C_Event_t event;
    event.type = I2C_EVENT_TX;
    event.p_data = hi2c->pBuffPtr;
    event.size = hi2c->XferSize;

    m_callback(&event);
}

/**
  * This function initialize I2C device.
  */
bool I2C_Init(void)
{
    /* configure I2C */
    I2CHandle.Instance             = I2Cx;

    I2CHandle.Init.ClockSpeed      = 88000;
    I2CHandle.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    I2CHandle.Init.OwnAddress1     = 0xFF;
    I2CHandle.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    I2CHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    I2CHandle.Init.OwnAddress2     = 0xff;
    I2CHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    I2CHandle.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;

    HAL_I2C_MspInit(&I2CHandle);

    /* DMA controller clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* configure DMA RX */
    DMAHandle_RX.Instance = DMA1_Stream5;
    DMAHandle_RX.Init.Channel = DMA_CHANNEL_1;
    DMAHandle_RX.Init.Direction = DMA_PERIPH_TO_MEMORY;
    DMAHandle_RX.Init.PeriphInc = DMA_PINC_DISABLE;
    DMAHandle_RX.Init.MemInc = DMA_MINC_ENABLE;
    DMAHandle_RX.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    DMAHandle_RX.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    DMAHandle_RX.Init.Mode = DMA_NORMAL;
    DMAHandle_RX.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    DMAHandle_RX.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    DMAHandle_RX.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    DMAHandle_RX.Init.MemBurst = DMA_MBURST_SINGLE;
    DMAHandle_RX.Init.PeriphBurst = DMA_PBURST_SINGLE;
    if (HAL_DMA_Init(&DMAHandle_RX) != HAL_OK)
    {
        return false;
    }
    __HAL_LINKDMA(&I2CHandle,hdmarx,DMAHandle_RX);

    HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

    /* configure DMA TX */
    DMAHandle_TX.Instance = DMA1_Stream6;
    DMAHandle_TX.Init.Channel = DMA_CHANNEL_1;
    DMAHandle_TX.Init.Direction = DMA_MEMORY_TO_PERIPH;
    DMAHandle_TX.Init.PeriphInc = DMA_PINC_DISABLE;
    DMAHandle_TX.Init.MemInc = DMA_MINC_ENABLE;
    DMAHandle_TX.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    DMAHandle_TX.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    DMAHandle_TX.Init.Mode = DMA_NORMAL;
    DMAHandle_TX.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    DMAHandle_TX.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    DMAHandle_TX.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    DMAHandle_TX.Init.MemBurst = DMA_MBURST_SINGLE;
    DMAHandle_TX.Init.PeriphBurst = DMA_PBURST_SINGLE;
    if (HAL_DMA_Init(&DMAHandle_TX) != HAL_OK)
    {
        return false;
    }

    __HAL_LINKDMA(&I2CHandle,hdmatx,DMAHandle_TX);

    HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

    if(HAL_I2C_Init(&I2CHandle) != HAL_OK)
    {
        return false;
    }

    return true;
}

void I2C_Callback_Register(I2C_Event_Callback callback)
{
    m_callback = callback;
}

void I2C_Mem_Write(uint16_t DevAddress, 
                   uint16_t MemAddress, uint16_t MemAddSize,
                   uint8_t *pData, uint16_t Size)
{
    HAL_I2C_Mem_Write(&I2CHandle, DevAddress, MemAddress, MemAddSize, pData, Size, HAL_MAX_DELAY);
}

void I2C_Mem_Read(uint16_t DevAddress,
                  uint16_t MemAddress, uint16_t MemAddSize,
                  uint8_t *pData, uint16_t Size)
{
    HAL_I2C_Mem_Read(&I2CHandle, DevAddress, MemAddress, MemAddSize, pData, Size, HAL_MAX_DELAY);
}

void I2C_DMA_Mem_Write(uint16_t DevAddress,
                       uint16_t MemAddress, uint16_t MemAddSize,
                       uint8_t *pData, uint16_t Size)
{
    HAL_I2C_Mem_Write_DMA(&I2CHandle, DevAddress, MemAddress, MemAddSize, pData, Size);
}


void I2C_DMA_Mem_Read(uint16_t DevAddress,
                      uint16_t MemAddress, uint16_t MemAddSize,
                      uint8_t *pData, uint16_t Size)
{
    HAL_I2C_Mem_Read_DMA(&I2CHandle, DevAddress, MemAddress, MemAddSize, pData, Size);
}
