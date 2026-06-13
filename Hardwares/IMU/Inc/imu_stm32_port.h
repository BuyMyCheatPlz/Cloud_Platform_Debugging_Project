/*
 * imu_stm32_port.h
 *
 *  STM32 HAL glue for the InvenSense MPU/DMP drivers.
 */

#ifndef INC_IMU_STM32_PORT_H_
#define INC_IMU_STM32_PORT_H_

#include "main.h"
#include "i2c.h"

#ifndef log_i
#define log_i(...) do { } while (0)
#endif

#ifndef log_e
#define log_e(...) do { } while (0)
#endif

#ifndef min
#define min(a, b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef __no_operation
#define __no_operation() do { } while (0)
#endif

struct int_param_s;

#if defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F407xx) || defined(STM32F417xx) || \
    defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) || \
    defined(STM32F446xx) || defined(STM32F469xx) || defined(STM32F479xx) || defined(STM32F412Zx) || \
    defined(STM32F412Vx)

static inline int i2c_write(unsigned char slave_addr, unsigned char reg_addr,
    unsigned char length, unsigned char const *data)
{
    if (HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(slave_addr << 1), reg_addr,
            I2C_MEMADD_SIZE_8BIT, (uint8_t *)data, length, HAL_MAX_DELAY) != HAL_OK)
        return -1;
    return 0;
}

static inline int i2c_read(unsigned char slave_addr, unsigned char reg_addr,
    unsigned char length, unsigned char *data)
{
    if (HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(slave_addr << 1), reg_addr,
            I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY) != HAL_OK)
        return -1;
    return 0;
}

static inline void delay_ms(unsigned long num_ms)
{
    HAL_Delay((uint32_t)num_ms);
}

static inline void get_ms(unsigned long *count)
{
    if (count)
        *count = HAL_GetTick();
}

static inline int reg_int_cb(struct int_param_s *int_param)
{
    (void)int_param;
    return 0;
}

#else
#error "Unsupported STM32 family for IMU port layer."
#endif

#endif /* INC_IMU_STM32_PORT_H_ */

