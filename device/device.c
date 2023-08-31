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
#include <stdlib.h>
#include "device.h"

static int device_printf(const char *fmt, ...){ return 0; }

DEVICE_INTF_RET_TYPE device_init(device_t *device,
                                 device_read_fptr_t read,
                                 device_write_fptr_t write,
                                 device_delay_us_fptr_t delay_us,
                                 device_printf_fptr_t println,
                                 device_get_timestamp_fptr_t get_timestamp,
                                 void *intf_ptr, void *fd)
{
    if (device == NULL)
    {
        return DEVICE_E_NULL_PTR;
    }
    
    device->read = read;
    device->write = write;
    device->delay_us = delay_us;
    if (println == NULL)
    {
        device->println = device_printf;
    }
    else
    {
        device->println = println;
    }
    
    device->get_timestamp = get_timestamp;
    device->intf_ptr = intf_ptr;
    device->fd = fd;

    return null_ptr_check(device);
}

DEVICE_INTF_RET_TYPE device_interface_init(device_t *device, void *intf_ptr)
{
    DEVICE_INTF_RET_TYPE ret = DEVICE_OK;

    if (intf_ptr == NULL)
    {
        ret = DEVICE_E_NULL_PTR;
    }
    device->intf_ptr = intf_ptr;
    return ret;
}

DEVICE_INTF_RET_TYPE device_deinit(device_t *device)
{
    if (device->intf_ptr)
    {
        free(device->intf_ptr);
    }
    if (device)
    {
        free(device);
    }
    return DEVICE_OK;
}

void device_check_rslt(const char api_name[], device_printf_fptr_t println, int8_t rslt)
{
    switch (rslt)
    {
        case DEVICE_OK:

            /* Do nothing */
            break;
        case DEVICE_E_NULL_PTR:
            println("API name [%s]  Error [%d] : Null pointer", api_name, rslt);
            break;
        case DEVICE_E_COM_FAIL:
            println("API name [%s]  Error [%d] : Communication failure", api_name, rslt);
            break;
        case DEVICE_E_INVALID_LENGTH:
            println("API name [%s]  Error [%d] : Incorrect length parameter", api_name, rslt);
            break;
        case DEVICE_E_NOT_FOUND:
            println("API name [%s]  Error [%d] : Device not found", api_name, rslt);
            break;
        default:
            println("API name [%s]  Error [%d] : Unknown error code", api_name, rslt);
            break;
    }
}

DEVICE_INTF_RET_TYPE null_ptr_check(const device_t *dev)
{
    DEVICE_INTF_RET_TYPE rslt = DEVICE_OK;

    if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->delay_us == NULL))
    {
        /* Device structure pointer is not valid */
        rslt = DEVICE_E_NULL_PTR;
    }
    else if (dev->get_timestamp == NULL || dev->println == NULL)
    {
        rslt = DEVICE_W_NULL_PTR;
    }

    return rslt;
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
	if (ret != DEVICE_OK) return ret;
    if (reg_addr != NULL && n_address != 0)
    {
        dev->intf_rslt = dev->write(reg_addr, 1, dev->intf_ptr);
        if (dev->intf_rslt != DEVICE_OK)
        {
            ret = DEVICE_E_COM_FAIL;
            return ret;
        }
    }

    if (read)
    {
        dev->intf_rslt = dev->read(buffer, buffer_size, dev->intf_ptr);
    }
    else
    {
        dev->intf_rslt = dev->write(buffer, buffer_size, dev->intf_ptr);
    }

    if (dev->intf_rslt != DEVICE_OK)
    {
        ret = DEVICE_E_COM_FAIL;
    }
    return ret;
}
