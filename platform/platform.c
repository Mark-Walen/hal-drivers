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
 * @file       platform.c
 * @date       2023-06-13
 * @version    v0.0.1
 * @author     Mark Walen
 * @brief      A common device interface.
 */
#include <stdlib.h>
#include <string.h>
#include "platform.h"

platform_t *platform = NULL;

platform_t *get_platform(void)
{
    return platform;
}

platform_ret_t platform_check_nullptr(platform_t *platform)
{
    if (platform == NULL || platform->delay_us == NULL || platform->name == NULL)
    {
        return PLATFORM_E_NULLPTR;
    }
    return PLATFORM_OK;
}

platform_ret_t platform_init(char *name,
                             platform_get_timestamp_fptr_t get_timestamp,
                             platform_delay_us_fptr_t delay_us,
                             platform_printf_fptr_t println)
{
    if (platform == NULL)
    {
        platform = malloc(sizeof(platform_t));
        memset(platform, 0, sizeof(platform_t));
    }
    
    platform->name = name;
    platform->get_timestamp = get_timestamp;
    platform->delay_us = delay_us;
    platform->println = println;

    return platform_check_nullptr(platform);
}
