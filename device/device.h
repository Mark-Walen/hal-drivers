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
 * @file       device.h
 * @date       2023-06-13
 * @version    v0.0.1
 * @author     Mark Walen
 * @brief      A common device interface.
 */

/*! @cond DOXYGEN_SUPRESS */

#ifndef __DEVICE_H__
#define __DEVICE_H__

/********************************************************* */
/*!             Header includes                           */
/********************************************************* */
#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/kernel.h>
#else
#include <stdint.h>
#include <stddef.h>
#endif

/********************************************************* */
/*!               Common Macros                           */
/********************************************************* */
#ifdef __KERNEL__
#if !defined(UINT8_C) && !defined(INT8_C)
#define INT8_C(x) S8_C(x)
#define UINT8_C(x) U8_C(x)
#endif

#if !defined(UINT16_C) && !defined(INT16_C)
#define INT16_C(x) S16_C(x)
#define UINT16_C(x) U16_C(x)
#endif

#if !defined(INT32_C) && !defined(UINT32_C)
#define INT32_C(x) S32_C(x)
#define UINT32_C(x) U32_C(x)
#endif

#if !defined(INT64_C) && !defined(UINT64_C)
#define INT64_C(x) S64_C(x)
#define UINT64_C(x) U64_C(x)
#endif
#endif

/*! C standard macros */
#ifndef NULL
#ifdef __cplusplus
#define NULL 0
#else
#define NULL ((void *)0)
#endif
#endif

#ifndef DEVICE_INTF_RET_TYPE
/**
 * DEVICE_INTF_RET_TYPE is the read/write interface return type which can be overwritten by the build system.
 * The default is set to int8_t.
 */
#define DEVICE_INTF_RET_TYPE int
#endif

#ifndef DEVICE_TICK_COUNT_TYPE
/**
 * DEVICE_TICK_COUNT_TYPE is the system tick count return type which can be overwritten by the build system.
 * The default type is set to uint32_t which is satified with most of embeded device.
 */
#define DEVICE_TICK_COUNT_TYPE uint32_t
#endif

#ifndef DEVICE_GPIO_CONTROL_RET_TYPE
/**
 * DEVICE_GPIO_CONTROL_RET_TYPE is the gpio control interface return type which can be overwritten by the build system.
 * The default is set to int8_t.
 */
#define DEVICE_GPIO_CONTROL_RET_TYPE int
#endif

#ifndef DEVICE_GPIO_CONFIG_RET_TYPE
/**
 * DEVICE_GPIO_CONTROL_RET_TYPE is the gpio control interface return type which can be overwritten by the build system.
 * The default is set to int8_t.
 */
#define DEVICE_GPIO_CONFIG_RET_TYPE uint16_t
#endif

#ifndef DEVICE_GPIO_MODE_TYPE
#define DEVICE_GPIO_MODE_TYPE       uint16_t
#endif
/********************************************************* */
/*!               Function Pointers                       */
/********************************************************* */

/*!
 * @brief Bus communication function pointer which should be mapped to
 * the platform specific read functions of the user
 *
 * @param[in]     reg_addr : 8bit register address of the sensor
 * @param[out]    reg_data : Data from the specified address
 * @param[in]     length   : Length of the reg_data array
 * @param[in,out] intf_ptr : Void pointer that can enable the linking of descriptors
 *                           for interface related callbacks
 * @param[in, out] intf_handle : Interface handle
 * @retval 0 for Success
 * @retval Non-zero for Failure
 */
typedef DEVICE_INTF_RET_TYPE (*device_read_fptr_t)(uint8_t *rx_buffer, uint32_t length, void *fd);

/*!
 * @brief Bus communication function pointer which should be mapped to
 * the platform specific write functions of the user
 *
 * @param[out]    reg_data : Data to the specified address
 * @param[in]     length   : Length of the reg_data array
 * @param[in,out] fd : Void pointer that can enable the linking of descriptors
 *                           for interface related callbacks
 * @retval 0 for Success
 * @retval Non-zero for Failure
 *
 */
typedef DEVICE_INTF_RET_TYPE (*device_write_fptr_t)(const uint8_t *tx_buffer, uint32_t length, void *fd);

/*!
 * @brief Delay function pointer which should be mapped to
 * delay function of the user
 *
 * @param period The time period in microseconds
 * @param[in,out] intf_ptr : Void pointer that can enable the linking of descriptors
 *                           for interface related callbacks
 */
typedef void (*device_delay_us_fptr_t)(uint32_t period);
typedef DEVICE_GPIO_CONTROL_RET_TYPE (*device_gpio_control_fptr_t)(void *port, uint8_t pin);
typedef DEVICE_GPIO_CONFIG_RET_TYPE (*device_gpio_config_fptr_t)(void *port, uint8_t pin);

/*!
 * @brief Sytem tick count function pointer which should be mapped to
 * system tick count function of the user
 *
 * @return system
 */
typedef DEVICE_TICK_COUNT_TYPE (*device_get_timestamp_fptr_t)(void);

/*!
 * @brief Printf function pointer which should be mapped to
 * printf function of the user. For debug only.
 *a
 * @param[in] fmt : This is the string that contains the text to be written to uart.
 */
typedef DEVICE_GPIO_CONTROL_RET_TYPE (*device_printf_fptr_t)(const char *fmt, ...);

/*!
 * @brief generic device structure
 */
typedef struct device device_t;

typedef struct device_gpio_typedef device_gpio_typedef_t;

/*!
 * @brief Interface selection Enumerations
 */
typedef enum device_intf device_intf_t;
typedef enum device_ret device_ret_t;

enum device_intf
{
    /*! SPI interface */
    SPI_INTF,
    /*! I2C interface */
    I2C_INTF
};

enum device_ret
{
    /* Success */
    DEVICE_OK,

    /* Null pointer which will not cause fatal error. */
    DEVICE_W_NULL_PTR,

    /* Unknow error */
    DEVICE_ERROR,

    /* Null pointer passed */
    DEVICE_E_NULL_PTR,

    /* Communication failure */
    DEVICE_E_COM_FAIL,

    /* Sensor not found */
    DEVICE_E_NOT_FOUND,

    /* Incorrect length parameter */
    DEVICE_E_INVALID_LENGTH,
};

struct device
{
    /* Chip Id */
    uint16_t chip_id;

    /* device id to store i2c address or chip select pin */
    uint8_t dev_id;

    /*
     * The interface pointer is used to enable the user
     * to link their interface descriptors for reference during the
     * implementation of the read and write interfaces to the
     * hardware.
     */
    void *intf_ptr;
    void *fd;

    /* Read function pointer */
    device_read_fptr_t read;

    /* Write function pointer */
    device_write_fptr_t write;

    /* Delay function pointer */
    device_delay_us_fptr_t delay_us;

    /* System tick count function pointer */
    device_get_timestamp_fptr_t get_timestamp;

    /* Printf function pointer */
    device_printf_fptr_t println;

    /* To store interface pointer error */
    DEVICE_INTF_RET_TYPE intf_rslt;
};

struct device_gpio_typedef
{
    void* port;
    uint16_t pin;
};

/********************************************************* */
/*!               Function Definition                      */
/********************************************************* */
/*!
 *  @brief Function to init device.
 *
 *  @param[in,out] device   : Structure instance of device
 *  @param[in] intf         : Interface selection parameter
 *  @param[in] read         : Device register read function pointer
 *  @param[in] write        : Device register write function pointer
 *  @param[in] delay_us     : Delay function pointer
 *  @param[in] println      : Debug printf function pointer
 *  @param[in,out] intf_ptr : Interface pointer
 *
 *  @return Status of execution
 *  @retval = DEVICE_INTF_RET_TYPE -> Success
 *  @retval != DEVICE_INTF_RET_TYPE  -> Failure Info
 */
DEVICE_INTF_RET_TYPE device_init(device_t *device,
                                 device_read_fptr_t read,
                                 device_write_fptr_t write,
                                 device_delay_us_fptr_t delay_us,
                                 device_printf_fptr_t println,
                                 device_get_timestamp_fptr_t get_timestamp,
                                 void *intf_ptr, void *fd);

/*!
 *  @brief Deinitialize device.
 *
 *  @param[in] device   : Structure instance of device
 *
 *  @return Status of execution
 *  @retval = DEVICE_INTF_RET_TYPE -> Success
 *  @retval != DEVICE_INTF_RET_TYPE  -> Failure
 */
DEVICE_INTF_RET_TYPE device_deinit(device_t *device);

/*!
 *  @brief Function to select the interface between SPI and I2C.
 *
 *  @param[in,out] device   : Structure instance of device
 *  @param[in] intf         : Interface selection parameter
 *  @param[in,out] intf_ptr : Interface pointer
 *
 *  @return Status of execution
 *  @retval = DEVICE_INTF_RET_TYPE -> Success
 *  @retval != DEVICE_INTF_RET_TYPE  -> Failure Info
 */
DEVICE_INTF_RET_TYPE device_interface_init(device_t *device, void *intf_ptr);

/*!
 *  @brief Prints the execution status of the APIs.
 *
 *  @param[in] api_name : Name of the API whose execution status has to be printed.
 *  @param[in] println  : Function pointer for println debug message.
 *  @param[in] rslt     : Error code returned by the API whose execution status has to be printed.
 *
 *  @return void.
 */
void device_check_rslt(const char api_name[], device_printf_fptr_t println, int8_t rslt);

/**
 *  @brief API is used to check the device for null pointers
 *  @param[in] device     : Structure instance of device
 * 
 *  @return Status of execution
 *  @retval = DEVICE_INTF_RET_TYPE -> Success
 *  @retval != DEVICE_INTF_RET_TYPE  -> Failure Info
 */
DEVICE_INTF_RET_TYPE null_ptr_check(const device_t *dev);

/**
 *  @brief A safe check wrapper device.read function
 *  @param[in] device       : Structure instance of device
 *  @param[in]     reg_addr : 8bit register address of the sensor
 *  @param[out]    reg_data : Data to the specified address
 *  @param[in]     length   : Length of the reg_data array
 * 
 *  @return Status of execution
 *  @retval = DEVICE_INTF_RET_TYPE -> Success
 *  @retval != DEVICE_INTF_RET_TYPE  -> Failure Info
 */
DEVICE_INTF_RET_TYPE device_transfer(device_t *dev,
                                 uint8_t *reg_addr,
                                 uint16_t n_address,
                                 uint8_t *buffer,
                                 uint32_t buffer_size,
                                 uint8_t read);
#endif
/*! @endcond */
