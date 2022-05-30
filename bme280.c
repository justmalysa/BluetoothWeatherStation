#include "bme280.h"
#include "i2c.h"
#include "stm32f4xx_hal.h"

#define BME280_ADDR                 0xEC
#define BME280_WRITE                0xEC
#define BME280_READ                 0xED
#define BME280_CTRL_MEAS            0xF4 // register for BME280 configuration
#define BME280_MODE_AND_OSRS        0x25 // oversampling x1 and forced mode
#define BME280_CTRL_HUM             0xF2
#define BME280_OSRS_H               0x01
#define BME280_STATUS               0xF3
#define BME280_STATUS_BUSY          (1 << 3)
#define BME280_TEMP                 0xFA
#define BME280_PRESS                0xF7
#define BME280_HUM                  0xFD
#define BME280_MEAS_VALUE_START     0xF7
#define BME280_MEAS_VALUE_COUNT     8
#define BME280_CALIB_VALUE_START    0x88
#define BME280_CALIB_VALUE_COUNT    26
#define BME280_CALIB_H_START        0xE1
#define BME280_CALIB_H_COUNT        7
#define BME280_RESET_REG            0xE0
#define BME280_RESET_VALUE          0xB6

typedef struct
{
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
} bme280_calib_temp_t;

typedef struct
{
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
} bme280_calib_press_t;

typedef struct
{
    uint8_t dig_H1;
    int16_t dig_H2;
    uint8_t dig_H3;
    int16_t dig_H4;
    int16_t dig_H5;
    int8_t  dig_H6;
} bme280_calib_hum_t;

typedef enum
{
    BME280_STATE_MEAS_STARTED,
    BME280_STATE_WAITING_FOR_READY,
    BME280_STATE_READOUT_STARTED
} bme280_state_t;

static bme280_calib_temp_t m_calib_temp;
static bme280_calib_press_t m_calib_press;
static bme280_calib_hum_t m_calib_hum;
static bme280_event_callback m_callback;
static volatile bme280_state_t m_state;
static uint8_t m_dma_buffer[32];

/* Returns temperature in DegC, resolution is 0.01 DegC.
   Equation taken from official documentation. */
static int32_t BME280_compensate_T(int32_t adc_T, bme280_calib_temp_t const * p_calib, int32_t * p_t_fine)
{
    int32_t var1, var2, T, t_fine;
    var1 = ((((adc_T>>3) - ((int32_t)p_calib->dig_T1<<1))) * ((int32_t)p_calib->dig_T2)) >> 11;
    var2 = (((((adc_T>>4) - ((int32_t)p_calib->dig_T1)) * ((adc_T>>4) - ((int32_t)p_calib->dig_T1)))
    >> 12) *
    ((int32_t)p_calib->dig_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    if (p_t_fine)
    {
        *p_t_fine = t_fine;
    }
    return T;
}

/* Returns absolute pressure in hPa.
   Equation taken from official documentation. */
static uint32_t BME280_compensate_P(uint32_t adc_P, bme280_calib_press_t const * p_calib, int32_t const * p_t_fine)
{
    int64_t var1, var2, p;
    var1 = ((int64_t)*p_t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)p_calib->dig_P6;
    var2 = var2 + ((var1*(int64_t)p_calib->dig_P5)<<17);
    var2 = var2 + (((int64_t)p_calib->dig_P4)<<35);
    var1 = ((var1 * var1 * (int64_t)p_calib->dig_P3)>>8) + ((var1 * (int64_t)p_calib->dig_P2)<<12);
    var1 = (((((int64_t)1)<<47)+var1))*((int64_t)p_calib->dig_P1)>>33;
    if (var1 == 0)
    {
        return 0; /* Avoid exception caused by division by zero. */
    }
    p = 1048576-adc_P;
    p = (((p<<31)-var2)*3125)/var1;
    var1 = (((int64_t)p_calib->dig_P9) * (p>>13) * (p>>13)) >> 25;
    var2 = (((int64_t)p_calib->dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)p_calib->dig_P7)<<4);
    return ((uint32_t)p >> 8);
}

/* Returns humidity in %RH.
   Equation taken from official documentation. */
static uint32_t BME280_compensate_H(int32_t adc_H,  bme280_calib_hum_t const * p_calib, int32_t const * p_t_fine)
{
    int32_t v_x1_u32r;
    v_x1_u32r = (*p_t_fine - ((int32_t)76800));
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)p_calib->dig_H4) << 20) - (((int32_t)p_calib->dig_H5) *
    v_x1_u32r)) + ((int32_t)16384)) >> 15) * (((((((v_x1_u32r *
    ((int32_t)p_calib->dig_H6)) >> 10) * (((v_x1_u32r * ((int32_t)p_calib->dig_H3)) >> 11) +
    ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)p_calib->dig_H2) +
    8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
    ((int32_t)p_calib->dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    return ((uint32_t)(v_x1_u32r>>12) >> 10);
}

static void bme280_i2c_event_callback(I2C_Event_t * p_event)
{
    if (p_event->type == I2C_EVENT_RX)
    {
        switch (m_state)
        {
            case BME280_STATE_WAITING_FOR_READY:
            {
                uint8_t status = p_event->p_data[0];
                if (BME280_STATUS_BUSY & status)
                {
                    I2C_DMA_Mem_Read(BME280_ADDR, BME280_STATUS, 1, &m_dma_buffer[0], 1);
                }
                else
                {
                    m_state = BME280_STATE_READOUT_STARTED;
                    I2C_DMA_Mem_Read(BME280_ADDR, BME280_MEAS_VALUE_START, 1,
                                                  &m_dma_buffer[0], BME280_MEAS_VALUE_COUNT);
                }
                break;
            }
            case BME280_STATE_READOUT_STARTED:
            {
                /* Process temperature measurement */
                int32_t temp_raw, temp_C, t_fine;
                temp_raw = (uint32_t)p_event->p_data[3] << 12;
                temp_raw |= (uint32_t)p_event->p_data[4] << 4;
                temp_raw |= (uint32_t)p_event->p_data[5] >> 4;
                temp_C = BME280_compensate_T(temp_raw, &m_calib_temp, &t_fine);

                /* Process pressure measurement */
                uint32_t press_raw, press_hPa;
                press_raw = (uint32_t)p_event->p_data[0] << 12;
                press_raw |= (uint32_t)p_event->p_data[1] << 4;
                press_raw |= (uint32_t)p_event->p_data[2] >> 4;
                press_hPa = BME280_compensate_P(press_raw, &m_calib_press, &t_fine);

                /* Process humidity measurement */
                int32_t hum_raw, hum_RH;
                hum_raw = (uint32_t)p_event->p_data[6] << 8;
                hum_raw |= (uint32_t)p_event->p_data[7] ;
                hum_RH = BME280_compensate_H(hum_raw, &m_calib_hum, &t_fine);

                bme280_event_t event;
                event.temp = temp_C;
                event.press = press_hPa;
                event.hum = hum_RH;

                m_callback(&event);
                m_state = BME280_STATE_MEAS_STARTED;
                m_dma_buffer[0] = BME280_MODE_AND_OSRS;
                I2C_DMA_Mem_Write(BME280_ADDR, BME280_CTRL_MEAS, 1, &m_dma_buffer[0], 1);
            }
            default:
                break;
        }
    }
    else if (p_event->type == I2C_EVENT_TX)
    {
        switch (m_state)
        {
            case BME280_STATE_MEAS_STARTED:
            {
                m_state = BME280_STATE_WAITING_FOR_READY;
                I2C_DMA_Mem_Read(BME280_ADDR, BME280_STATUS, 1, &m_dma_buffer[0], 1);
                break;
            }
            default:
                break;
        }
    }
}

void bme280_init(void)
{
    if (!I2C_Init())
    {
        while (1);
    }

    /* sensor reset */
    HAL_Delay(3);
    uint8_t byte = BME280_RESET_VALUE;
    I2C_Mem_Write(BME280_ADDR, BME280_RESET_REG, 1, &byte, 1);
    HAL_Delay(20);

    /* read calibration values */
    uint8_t calib_values[BME280_CALIB_VALUE_COUNT + BME280_CALIB_H_COUNT];
    I2C_Mem_Read(BME280_ADDR, BME280_CALIB_VALUE_START, 1, calib_values, BME280_CALIB_VALUE_COUNT);
    I2C_Mem_Read(BME280_ADDR, BME280_CALIB_H_START, 1, &calib_values[BME280_CALIB_VALUE_COUNT], BME280_CALIB_H_COUNT);

    m_calib_temp.dig_T1 = (uint16_t)calib_values[1] << 8 | calib_values[0];
    m_calib_temp.dig_T2 = (uint16_t)calib_values[3] << 8 | calib_values[2];
    m_calib_temp.dig_T3 = (uint16_t)calib_values[5] << 8 | calib_values[4];

    m_calib_press.dig_P1 = (uint16_t)calib_values[7] << 8 | calib_values[6];
    m_calib_press.dig_P2 = (uint16_t)calib_values[9] << 8 | calib_values[8];
    m_calib_press.dig_P3 = (uint16_t)calib_values[11] << 8 | calib_values[10];
    m_calib_press.dig_P4 = (uint16_t)calib_values[13] << 8 | calib_values[12];
    m_calib_press.dig_P5 = (uint16_t)calib_values[15] << 8 | calib_values[14];
    m_calib_press.dig_P6 = (uint16_t)calib_values[17] << 8 | calib_values[16];
    m_calib_press.dig_P7 = (uint16_t)calib_values[19] << 8 | calib_values[18];
    m_calib_press.dig_P8 = (uint16_t)calib_values[21] << 8 | calib_values[20];
    m_calib_press.dig_P9 = (uint16_t)calib_values[23] << 8 | calib_values[22];

    m_calib_hum.dig_H1 = calib_values[25];
    m_calib_hum.dig_H2 = (uint16_t)calib_values[27] << 8 | calib_values[26];
    m_calib_hum.dig_H3 = calib_values[28];
    m_calib_hum.dig_H4 = (uint16_t)calib_values[29] << 4 | (calib_values[30] & 0x0F);
    m_calib_hum.dig_H5 = (uint16_t)calib_values[31] << 4 | (calib_values[30] >> 4);
    m_calib_hum.dig_H6 = (uint8_t)calib_values[32];

    byte = BME280_OSRS_H;
    I2C_Mem_Write(BME280_ADDR, BME280_CTRL_HUM, 1, &byte, 1);
}

void bme280_async_start(bme280_event_callback callback)
{
    m_callback = callback;
    I2C_Callback_Register(bme280_i2c_event_callback);

    m_state = BME280_STATE_MEAS_STARTED;
    m_dma_buffer[0] = BME280_MODE_AND_OSRS;
    I2C_DMA_Mem_Write(BME280_ADDR, BME280_CTRL_MEAS, 1, &m_dma_buffer[0], 1);
}

void bme280_measure(void)
{
    uint8_t byte = BME280_MODE_AND_OSRS;
    I2C_Mem_Write(BME280_ADDR, BME280_CTRL_MEAS, 1, &byte, 1);

    uint8_t status;
    do
    {
        I2C_Mem_Read(BME280_ADDR, BME280_STATUS, 1, &status, 1);
    } while (BME280_STATUS_BUSY & status);
}

int32_t bme280_temp_get(int32_t * p_t_fine)
{
    uint8_t buffer[3];
    I2C_Mem_Read(BME280_ADDR, BME280_TEMP, 1, buffer, 3);
    int32_t temp_raw;
    temp_raw = (uint32_t)buffer[0] << 12;
    temp_raw |= (uint32_t)buffer[1] << 4;
    temp_raw |= (uint32_t)buffer[2] >> 4;

    return BME280_compensate_T(temp_raw, &m_calib_temp, p_t_fine);
}

uint32_t bme280_press_get(int32_t const * p_t_fine)
{
    uint8_t buffer[3];
    I2C_Mem_Read(BME280_ADDR, BME280_PRESS, 1, buffer, 3);
    uint32_t press_raw;
    press_raw = (uint32_t)buffer[0] << 12;
    press_raw |= (uint32_t)buffer[1] << 4;
    press_raw |= (uint32_t)buffer[2] >> 4;

    return BME280_compensate_P(press_raw, &m_calib_press, p_t_fine);
}

uint32_t bme280_hum_get(int32_t const * p_t_fine)
{
    uint8_t buffer[2];
    I2C_Mem_Read(BME280_ADDR, BME280_HUM, 1, buffer, 2);
    int32_t hum_raw;
    hum_raw = (uint32_t)buffer[0] << 8;
    hum_raw |= (uint32_t)buffer[1];

    return BME280_compensate_H(hum_raw,	&m_calib_hum, p_t_fine);
}
