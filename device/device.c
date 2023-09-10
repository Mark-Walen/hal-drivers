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
 * @file       device.c
 * @date       2023-06-13
 * @version    v0.0.1
 * @author     Mark Walen
 * @brief      A common device interface.
 */
#include "stdio.h"
#include <stdlib.h>
#include <stdarg.h>
#include "device.h"

static int device_printf(const char *fmt, ...){ return 0; }

DEVICE_INTF_RET_TYPE device_init(device_t *device,
                                 platform_ioctl_fptr_t read,
                                 platform_ioctl_fptr_t write,
                                 void *addr, void *fp)
{
    if (device == NULL)
    {
        return DEVICE_E_NULLPTR;
    }
    memset(device, 0, sizeof(device));
    
    device->read = read;
    device->write = write;
    device->addr = addr;
    device->fp = fp;

    return null_ptr_check(device);
}

/**
 * @brief Config device info: name
*/
DEVICE_INTF_RET_TYPE vconfig_device_info(device_t *device, const char *fmt, va_list args)
{
    struct device_info *info = NULL;
    uint8_t free_flag = 0;
    if (device == NULL) return DEVICE_E_NULLPTR;

    if (device->info)
    {
        info = device->info;
    }
    else
    {
        info = malloc(sizeof(struct device_info));
        if (info == NULL) return DEVICE_E_HEAP_OVERFLOW;
        
        memset(info, 0, sizeof(struct device_info));
        device->info = info;
        free_flag = 1;
    }

    while (*fmt)
    {
        if (*fmt == '%') {
            fmt++;
            switch (*fmt) {
                case 'n':
                    info->name = va_arg(args, const char *);
                    free_flag = 0;
                    break;
                case 'i':
                    info->interface = va_arg(args, const char *);
                    free_flag = 0;
                    break;
                case 'c':
                    info->chip_id = va_arg(args, uint8_t);
                    free_flag = 0;
                    break;
                default:
                    // Handle unsupported format specifier
                    break;
            }
        }
        fmt++;
    }
    
    if (free_flag)
    {
        device->info = NULL;
        free(info);
    }
    
    return DEVICE_OK;
}

DEVICE_INTF_RET_TYPE config_device_info(device_t *device, const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vconfig_device_info(device, fmt, args);
    va_end(args);
    
    return DEVICE_OK;
}

DEVICE_INTF_RET_TYPE device_null_ptr_check(const device_t *dev)
{
    DEVICE_INTF_RET_TYPE rslt = DEVICE_OK;

    if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL))
    {
        /* Device structure pointer is not valid */
        return DEVICE_E_NULLPTR;
    }

    return  platform_check_nullptr(platform);
}

DEVICE_INTF_RET_TYPE device_transfer(device_t *dev,
                                 uint8_t *reg_addr,
                                 uint16_t n_address,
                                 uint8_t *buffer,
                                 uint32_t buffer_size,
                                 uint8_t read)
{
    uint8_t ret = DEVICE_OK;
    
    ret = null_ptr_check(dev);
    return ret;
}
