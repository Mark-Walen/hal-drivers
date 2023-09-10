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
#include <string.h>
#include "platform.h"

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

/* Device instance function common return values */
#define DEVICE_COMMON_RET(NAME) COMMON_RET(NAME), \
                                RET_ERROR(NAME, COM_FAIL), \
                                RET_ERROR(NAME, NOT_FOUND)

#define DEVICE_INTF_RET_TYPE PLATFORM_INTF_RET_TYPE

#ifndef DEVICE_GPIO_CONTROL_RET_TYPE
/**
 * DEVICE_GPIO_CONTROL_RET_TYPE is the gpio control interface return type which can be overwritten by the build system.
 * The default is set to int8_t.
 */
#define DEVICE_GPIO_CONTROL_RET_TYPE int
#endif

/********************************************************* */
/*!               Function Pointers                       */
/********************************************************* */

/*!
 * @brief generic device structure
 */
typedef struct device device_t;

/*!
 * @brief Interface selection Enumerations
 */
typedef enum device_type device_type_t;
typedef enum device_ret device_ret_t;

enum device_ret
{
    DEVICE_COMMON_RET(DEVICE),

    /* Incorrect length parameter */
    RET_ERROR(DEVICE, INVALID_LENGTH),

    /* Insufficient heap space. */
    RET_ERROR(DEVICE, HEAP_OVERFLOW)
};

enum device_type
{
    GPIO=1,
    SPI,
    I2C,
    UART
};

struct device_info
{
    /* Chip Id */
    uint16_t chip_id;

    /* Device name */
    const char *name;

    /* Device interface type */
    const char *interface;

    /* Device interface type. Check device_type_t for more details. */
    device_type_t type;
};

struct device
{
    /*!
     * The interface pointer is used to enable the user
     * to link their interface descriptors for reference during the
     * implementation of the read and write interfaces to the
     * hardware.
     */
    void *fp;

    /* Interface register address offset. GPIO pin number, SPI CS interface pointer, I2C slave address, etc. */
    void *addr;

    struct device_info *info;
    
    /* Read function pointer */
    platform_ioctl_fptr_t read;

    /* Write function pointer */
    platform_ioctl_fptr_t write;
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
 *  @param[in,out] addr : Interface pointer
 *
 *  @return Status of execution
 *  @retval = DEVICE_INTF_RET_TYPE -> Success
 *  @retval != DEVICE_INTF_RET_TYPE  -> Failure Info
 */
DEVICE_INTF_RET_TYPE device_init(device_t *device,
                                 platform_ioctl_fptr_t read,
                                 platform_ioctl_fptr_t write,
                                 void *fp, void *addr);

DEVICE_INTF_RET_TYPE config_device_info(device_t *device, const char *name, ...);

/**
 * @brief Get device infomation. Name, interface type and chip id.
*/
struct device_info *get_device_info(device_t *device);

/**
 *  @brief API is used to check the device for null pointers
 *  @param[in] device     : Structure instance of device
 * 
 *  @return Status of execution
 *  @retval = DEVICE_INTF_RET_TYPE -> Success
 *  @retval != DEVICE_INTF_RET_TYPE  -> Failure Info
 */
DEVICE_INTF_RET_TYPE device_null_ptr_check(const device_t *dev);

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
