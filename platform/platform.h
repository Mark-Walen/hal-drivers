/**
 * Copyright (c) 2023 Blue Monster. All rights reserved.
 *
 * BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * @file       platform.h
 * @date       2023-08-31
 * @version    v0.0.1
 * @author     Mark Walen
 * @brief      MCU platform support.
 */
#ifndef __PLATFORM_H__
#define __PLATFORM_H__
#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/kernel.h>
#else
#include <stdint.h>
#include <stddef.h>
#endif

/* Function return values of info */
#define RET_INFO(NAME, MSG) NAME ## _ ## MSG
/* FUnction return values of warning */
#define RET_WARN(NAME, MSG) NAME ## _W_ ## MSG
/* Function return values of error */
#define RET_ERROR(NAME, MSG) NAME ## _E_ ## MSG

/* Common function ret values: success, error, and nullptr error (Instance was not initialized properly) */
#define COMMON_RET(NAME) RET_INFO(NAME, OK), \
                         RET_INFO(NAME, ERROR), \
                         RET_ERROR(NAME, NULLPTR), \
                         RET_ERROR(NAME, UNINIT_PLATFORM)   // Platform unintialized

#ifndef PLATFORM_INTF_RET_TYPE
/**
 * PLATFORM_INTF_RET_TYPE is the read/write interface return type which can be overwritten by the build system.
 * The default is set to int8_t.
 */
#define PLATFORM_INTF_RET_TYPE int
#endif

#ifndef PLATFORM_TICK_COUNT_TYPE
/**
 * PLATFORM_TICK_COUNT_TYPE is the system tick count return type which can be overwritten by the build system.
 * The default type is set to uint32_t which is satified with most of embeded device.
 */
#define PLATFORM_TICK_COUNT_TYPE uint32_t
#endif

#ifndef PLATFORM_PRINTF_RET_TYPE
/**
 * PLATFORM_PRINTF_RET_TYPE is the system tick count return type which can be overwritten by the build system.
 * The default type is set to int which is satified with most of embeded device.
 */
#define PLATFORM_PRINTF_RET_TYPE int
#endif

#ifndef PLATFORM_GPIO_CONTROL_RET_TYPE
/**
 * PLATFORM_GPIO_CONTROL_RET_TYPE is the gpio control interface return type which can be overwritten by the build system.
 * The default is set to int16_t.
 */
#define PLATFORM_GPIO_CONTROL_RET_TYPE int16_t
#endif

/*!
 * @brief Sytem tick count function pointer which should be mapped to
 * system tick count function of the user
 *
 * @return system
 */
typedef PLATFORM_TICK_COUNT_TYPE (*platform_get_timestamp_fptr_t)(void);

/*!
 * @brief Delay function pointer which should be mapped to
 * delay function of the user
 *
 * @param[in] period The time period in microseconds
 */
typedef void (*platform_delay_us_fptr_t)(uint32_t period);

/*!
 * @brief Printf function pointer which should be mapped to
 * printf function of the user. For debug only.
 *
 * @param[in] fmt : This is the string that contains the text to be written to uart.
 */
typedef PLATFORM_PRINTF_RET_TYPE (*platform_printf_fptr_t)(const char *fmt, ...);

/*!
 * @brief Bus communication function pointer which should be mapped to
 * the platform specific read functions of the user
 *
 * @param[out]    reg_data : Data from the specified address
 * @param[in]     length   : Length of the reg_data array
 * @param[in]     fp : Interface pointer: GPIOx, SPIx, I2Cx etc.
 * @param[in]     addr : Interface register address offset. GPIO pin number, SPI CS interface pointer, I2C slave address, etc.
 * @retval 0 for Success
 * @retval Non-zero for Failure
 */
typedef PLATFORM_INTF_RET_TYPE (*platform_ioctl_fptr_t)(uint8_t *reg_data, uint32_t length, void *fp, void *addr);

typedef struct platform_s platform_t;
typedef enum platform_ret platform_ret_t;

enum platform_ret
{
    COMMON_RET(PLATFORM),
    PLATFORM_E_OOM
};

struct platform_s
{
    /* Platform name. STM32F103 etc. */
    const char *name;

    /* System tick count function pointer */
    platform_get_timestamp_fptr_t get_timestamp;

    /* Delay function pointer */
    platform_delay_us_fptr_t delay_us;

    /* Printf function pointer */
    platform_printf_fptr_t println;
};

/**
 * @brief Get Platform instance
 * 
 * @return Standalone platform intance
*/
platform_t *get_platform(void);
platform_ret_t platform_init(char *name,
                           platform_get_timestamp_fptr_t get_timestamp,
                           platform_delay_us_fptr_t delay_us,
                           platform_printf_fptr_t println);
platform_ret_t platform_check_nullptr(platform_t *platform);
void *platform_malloc(size_t size);
void *platform_free(size_t size);

platform_ret_t platform_delay_us(uint32_t us);
platform_ret_t platform_delay_ms(uint32_t ms);
platform_ret_t platform_get_timestamp(PLATFORM_TICK_COUNT_TYPE *t);
#endif
