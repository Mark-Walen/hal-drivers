#ifndef __W25Q16_H__
#define __W25Q16_H__

#include "device.h"

#define WRITE_ENABLE 0x06
#define WRITE_DISABLE 0x04
#define PAGE_PROGRAM 0x02
#define READ_STATUS_REGISTER_1 0x05
#define READ_DATA 0x03
#define CHIP_ERASE 0xC7
#define POWER_DOWN 0xB9
#define RELEASE_POWER_DOWN 0xAB
#define MANUFACTURER_ID 0x90

typedef struct w25qxx_handle_s w25qxx_handle_t;

struct w25qxx_handle_s
{
    device_t *dev;
    device_gpio_control_fptr_t gpio_set_pin;
    device_gpio_control_fptr_t gpio_reset_pin;
};

DEVICE_INTF_RET_TYPE w25qxx_init(w25qxx_handle_t *w25qxx, device_t *dev,
                                 device_gpio_control_fptr_t gpio_set_pin,
                                 device_gpio_control_fptr_t gpio_reset_pin);
DEVICE_INTF_RET_TYPE w25qxx_read_id(w25qxx_handle_t *w25qxx);
DEVICE_INTF_RET_TYPE w25qxx_null_ptr_check(w25qxx_handle_t *w25qxx);
#endif
