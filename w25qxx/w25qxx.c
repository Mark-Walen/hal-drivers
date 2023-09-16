// #include <stdio.h>
#include <stdlib.h>
#include "w25qxx.h"

DEVICE_INTF_RET_TYPE w25qxx_init(w25qxx_handle_t *w25qxx, device_t *dev)
{
    if (w25qxx == NULL || dev == NULL)
    {
        return DEVICE_E_NULLPTR;
    }
    w25qxx->dev = dev;
    return w25qxx_null_ptr_check(w25qxx);
}

DEVICE_INTF_RET_TYPE w25qxx_read_id(w25qxx_handle_t *w25qxx)
{
    uint8_t gpio_level = 0;
    uint8_t ret = w25qxx_null_ptr_check(w25qxx);
    device_t *dev = w25qxx->dev;
	device_t *nss = (device_t *) dev->addr;
	platform_t *mcu = get_platform();
    uint8_t tx_data[4] = {0x90, 0x00, 0x00, 0x00};
    uint8_t rx_data[2] = {0xFF, 0xFF};
    if (ret != DEVICE_OK) return ret;

    nss->write(&gpio_level, 1, nss->fp, nss->addr);
	dev->write(tx_data, 4, dev->fp, dev->addr);
	dev->read(rx_data, 2, dev->fp, dev->addr);
	gpio_level = 1;
	nss->write(&gpio_level, 1, nss->fp, nss->addr);

    config_device_info(dev, "%n%i%c", "w25qxx", "SPI", ((rx_data[0] << 8) | rx_data[1]));

    return DEVICE_OK;
}

DEVICE_INTF_RET_TYPE w25qxx_null_ptr_check(w25qxx_handle_t *w25qxx)
{
    if (w25qxx == NULL)
    {
        return DEVICE_E_NULLPTR;
    }
    return device_null_ptr_check(w25qxx->dev);
}