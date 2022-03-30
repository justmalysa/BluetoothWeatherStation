#ifndef I2C_H
#define I2C_H

#include "stdbool.h"
#include "stdint.h"
#include "stdlib.h"

typedef enum
{
    I2C_EVENT_TX,
    I2C_EVENT_RX
}I2C_Event_Type_t;

typedef struct
{
    I2C_Event_Type_t type;
    uint8_t * p_data;
    size_t size;
} I2C_Event_t;

typedef void (* I2C_Event_Callback) (I2C_Event_t * p_event);

/* Function prototypes ------------------------------------------------------- */

/** Initializes the I2C interface.
 *  @return true if the initialization was successful and the port is operational,
 *          false otherwise
 */
bool I2C_Init(void);

/** Registers the callback function associated with finished DMA transfers.
 */
void I2C_Callback_Register(I2C_Event_Callback callback);

/** Writes the data to specified register of a given slave.
 */
void I2C_Mem_Write(uint16_t DevAddress, 
                   uint16_t MemAddress, uint16_t MemAddSize,
                   uint8_t *pData, uint16_t Size);

/** Reads the data from specified register of a given slave.
 */
void I2C_Mem_Read(uint16_t DevAddress,
                  uint16_t MemAddress, uint16_t MemAddSize,
                  uint8_t *pData, uint16_t Size);

/** Writes the data to specified register of a given slave using DMA.
 */
void I2C_DMA_Mem_Write(uint16_t DevAddress,
                       uint16_t MemAddress, uint16_t MemAddSize,
                       uint8_t *pData, uint16_t Size);

/** Reads the data from specified register of a given slave using DMA.
 */
void I2C_DMA_Mem_Read(uint16_t DevAddress,
                      uint16_t MemAddress, uint16_t MemAddSize,
                      uint8_t *pData, uint16_t Size);

#endif /* I2C_H */
