// #include <stdio.h>
#include <stdlib.h>
#include "w25qxx.h"

DEVICE_INTF_RET_TYPE w25qxx_init(w25qxx_handle_t *w25qxx, device_t *dev,
                                 device_gpio_control_fptr_t gpio_set_pin, device_gpio_control_fptr_t gpio_reset_pin)
{
    if (w25qxx == NULL || dev == NULL)
    {
        return DEVICE_E_NULL_PTR;
    }
    w25qxx->dev = dev;
    w25qxx->gpio_set_pin = gpio_reset_pin;
    w25qxx->gpio_reset_pin = gpio_reset_pin;
    return w25qxx_null_ptr_check(w25qxx);
}

DEVICE_INTF_RET_TYPE w25qxx_read_id(w25qxx_handle_t *w25qxx)
{
    uint8_t ret = w25qxx_null_ptr_check(w25qxx);
    if (ret != DEVICE_OK) return ret;
    device_t *dev = w25qxx->dev;
    device_gpio_typedef_t *nss = (device_gpio_typedef_t *) dev->addr;
    uint8_t tx_data[4] = {0x90, 0x00, 0x00, 0x00};
    uint8_t rx_data[2] = {0xFF, 0xFF};

    w25qxx->gpio_reset_pin(nss->port, nss->pin);
    dev->write(tx_data, 4, dev->fp);
    dev->read(rx_data, 2, dev->fp);
    w25qxx->gpio_set_pin(nss->port, nss->pin);

    w25qxx->dev->chip_id = ((rx_data[0] << 8) | rx_data[1]);
    return DEVICE_OK;
}

DEVICE_INTF_RET_TYPE w25qxx_null_ptr_check(w25qxx_handle_t *w25qxx)
{
    if (w25qxx == NULL || w25qxx->gpio_set_pin == NULL || w25qxx->gpio_reset_pin == NULL)
    {
        return DEVICE_E_NULL_PTR;
    }
    return null_ptr_check(w25qxx->dev);
}