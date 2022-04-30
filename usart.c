/* Includes ------------------------------------------------------------------*/
#include "usart.h"
#include <stdio.h>
#include <string.h>
#include "ring_buffer.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_rcc.h"


/* Private definitions -------------------------------------------------------*/

/* Definition for USARTx clock resources */
#define USARTx                           USART2
#define USARTx_CLK_ENABLE()              __USART2_CLK_ENABLE();
#define USARTx_RX_GPIO_CLK_ENABLE()      __GPIOA_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE()      __GPIOA_CLK_ENABLE() 

#define USARTx_FORCE_RESET()             __USART2_FORCE_RESET()
#define USARTx_RELEASE_RESET()           __USART2_RELEASE_RESET()

/* Definition for USARTx Pins */
#define USARTx_TX_PIN                    GPIO_PIN_2
#define USARTx_TX_GPIO_PORT              GPIOA  
#define USARTx_TX_AF                     GPIO_AF7_USART2
#define USARTx_RX_PIN                    GPIO_PIN_3
#define USARTx_RX_GPIO_PORT              GPIOA 
#define USARTx_RX_AF                     GPIO_AF7_USART2

/* Definition for USARTx's NVIC */
#define USARTx_IRQn                      USART2_IRQn
#define USARTx_IRQHandler                USART2_IRQHandler

/* Private variables ---------------------------------------------------------*/
// USART transmit buffer descriptor
static RingBuffer USART_RingBuffer_Tx;
// USART transmit buffer memory pool
static char RingBufferData_Tx[1024];

// USART receive buffer descriptor
static RingBuffer USART_RingBuffer_Rx;
// USART receive buffer memory pool
static char RingBufferData_Rx[1024];

/// USART descriptor
static USART_HandleTypeDef UsartHandle;


bool USART_PutChar(char c)
{
    __disable_irq();
    bool result = RingBuffer_PutChar(&USART_RingBuffer_Tx, c);
    if (result)
    {
        __USART_ENABLE_IT(&UsartHandle, USART_IT_TXE);
    }
    __enable_irq();
    return result;
}


size_t USART_WriteData(const void *data, size_t dataSize)
{
    size_t i = 0;
    for (size_t iter=0;iter<dataSize;iter++)
    {
        if(USART_PutChar(*((const char *)data+iter)))
        {
            i++;
        }
    }
    return i;
}


size_t USART_WriteString(const char *string)
{
    return USART_WriteData(string, strlen(string));
}


bool USART_GetChar(char *c)
{
    __disable_irq();
    bool result =  RingBuffer_GetChar(&USART_RingBuffer_Rx, c);
    __enable_irq();
    return result;
}


size_t USART_ReadData(char *data, size_t maxSize)
{
    size_t i = 0;
    for(int iter=0;iter<maxSize;iter++)
    {
        if(USART_GetChar(data+iter))
        {
            i++;
        }
        else
        {
            break;
        }
    }
    return i;
}

bool USART_SetCallback_OnNewLine(int TODO)
{
    return false;
}

// USART Interrupt Service Routine (ISR)
void USARTx_IRQHandler(void)
{
    char c;
    if (__HAL_USART_GET_FLAG(&UsartHandle, USART_FLAG_RXNE))
    {
        // the RXNE interrupt has occurred
        if (__HAL_USART_GET_IT_SOURCE(&UsartHandle, USART_IT_RXNE))
        {
            // the RXNE interrupt is enabled
            c=USARTx->DR;
            RingBuffer_PutChar(&USART_RingBuffer_Rx, c);
        }
    }

    if (__HAL_USART_GET_FLAG(&UsartHandle, USART_FLAG_TXE))
    {
        // the TXE interrupt has occurred
        if (__HAL_USART_GET_IT_SOURCE(&UsartHandle, USART_IT_TXE)) 
        {
            // the TXE interrupt is enabled
            bool result = RingBuffer_GetChar(&USART_RingBuffer_Tx, &c);
            if(result)
            {
                USARTx->DR = c;
            }
            else
            {
                __USART_DISABLE_IT(&UsartHandle, USART_IT_TXE);
            }
        }
    }
}



/**
  * This function configures the hardware resources used in this example: 
  *  - Peripheral's clock enable
  *	 - Peripheral's GPIO Configuration  
  *  - NVIC configuration for USART interrupt request enable
  */
void HAL_USART_MspInit(USART_HandleTypeDef *husart)
{
    GPIO_InitTypeDef  GPIO_InitStruct;

    /*##-1- Enable peripherals and GPIO Clocks #################################*/
    /* Enable GPIO TX/RX clock */
    USARTx_TX_GPIO_CLK_ENABLE();
    USARTx_RX_GPIO_CLK_ENABLE();
    /* Enable USART1 clock */
    USARTx_CLK_ENABLE(); 

    /*##-2- Configure peripheral GPIO ##########################################*/  
    /* USART TX GPIO pin configuration  */
    GPIO_InitStruct.Pin       = USARTx_TX_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
    GPIO_InitStruct.Alternate = USARTx_TX_AF;

    HAL_GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStruct);

    /* USART RX GPIO pin configuration  */
    GPIO_InitStruct.Pin = USARTx_RX_PIN;
    GPIO_InitStruct.Alternate = USARTx_RX_AF;

    HAL_GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStruct);

    /*##-3- Configure the NVIC for USART ########################################*/
    /* NVIC for USART2 */
    HAL_NVIC_SetPriority(USARTx_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(USARTx_IRQn);
}

/**
  * This function frees the hardware resources used in this example:
  *   - Disable the Peripheral's clock
  *   - Revert GPIO and NVIC configuration to their default state  
  */
void HAL_USART_MspDeInit(USART_HandleTypeDef *husart)
{
    /*##-1- Reset peripherals ##################################################*/
    USARTx_FORCE_RESET();
    USARTx_RELEASE_RESET();

    /*##-2- Disable peripherals and GPIO Clocks ################################*/
    /* Configure USART Tx as alternate function */
    HAL_GPIO_DeInit(USARTx_TX_GPIO_PORT, USARTx_TX_PIN);
    /* Configure USART Rx as alternate function */
     HAL_GPIO_DeInit(USARTx_RX_GPIO_PORT, USARTx_RX_PIN);

    /*##-3- Disable the NVIC for USART ##########################################*/
    HAL_NVIC_DisableIRQ(USARTx_IRQn);
}

/**
  * This function initialize ring buffers, USART device and enable receive interrupt.  
  */
bool USART_Init(void)
{
    // initialize ring buffers
    RingBuffer_Init(&USART_RingBuffer_Tx, RingBufferData_Tx, sizeof(RingBufferData_Tx));
    RingBuffer_Init(&USART_RingBuffer_Rx, RingBufferData_Rx, sizeof(RingBufferData_Rx));

    // configure USART
    UsartHandle.Instance          = USARTx;

    UsartHandle.Init.BaudRate     = 115200;
    UsartHandle.Init.WordLength   = USART_WORDLENGTH_8B;
    UsartHandle.Init.StopBits     = USART_STOPBITS_1;
    UsartHandle.Init.Parity       = USART_PARITY_NONE;
    UsartHandle.Init.Mode         = USART_MODE_TX_RX;

    HAL_USART_MspInit(&UsartHandle);

    if(HAL_USART_Init(&UsartHandle) != HAL_OK)
    {
        return false;
    }

    UsartHandle.Instance->CR2 &= ~(USART_CR2_CLKEN);

    /* Enable the USART Data Register not empty Interrupt */
    __USART_ENABLE_IT(&UsartHandle, USART_IT_RXNE); 

    return true;
}
