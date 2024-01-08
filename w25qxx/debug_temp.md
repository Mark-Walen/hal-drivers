```c
/**
 * @brief W25Qxx Read/Write StatusRegister
 */
DEVICE_INTF_RET_TYPE w25qxx_read_status_register(w25qxx_handle_t *w25qxx, uint8_t select_sr_1_2_3)
{
    DEVICE_INTF_RET_TYPE ret = W25QXX_OK;

    switch (Select_SR_1_2_3)
    {
    case 1:
        w25qxx_read_cmd(w25qxx, W25Q_CMD_RSREG1, &w25qxx.status_register1);
        break;
    case 2:
        w25qxx_read_cmd(w25qxx, W25Q_CMD_RSREG2, &w25qxx.status_register2); break;
    case 3: 
        w25qxx_read_cmd(w25qxx, W25Q_CMD_RSREG3, &w25qxx.status_register3); break;
        default: break;
    }

    return ret;
}

DEVICE_INTF_RET_TYPE w25qxx_send_cmd(w25qxx_handle_t *w25qxx, uint8_t cmd)
{
    DEVICE_INTF_RET_TYPE ret = w25qxx_null_ptr_check(w25qxx);
    device_t *dev = w25qxx->dev;
    device_t *nss = (device_t *)dev->addr;
    if (ret != DEVICE_OK)
        return ret;

    ret = device_write_byte(nss, 0);
    ret = device_write_byte(dev, cmd);
    ret = device_write_byte(nss, 1);

    return ret;
}

DEVICE_INTF_RET_TYPE w25qxx_read_cmd(w25qxx_handle_t *w25qxx, uint8_t cmd, uint8_t *reg_data)
{
    DEVICE_INTF_RET_TYPE ret = w25qxx_null_ptr_check(w25qxx);
    *reg_data = W25Q_DUMMY;
    device_t *dev = w25qxx->dev;
    device_t *nss = (device_t *)dev->addr;
    if (ret != DEVICE_OK)
        return ret;

    ret = device_write_byte(nss, 0);
    ret = device_write_byte(dev, cmd);
    ret = device_read_byte(dev, reg_data);
    ret = device_write_byte(nss, 1);

    return ret;
}

DEVICE_INTF_RET_TYPE w25qxx_write_cmd(w25qxx_handle_t *w25qxx, uint8_t cmd, uint8_t data)
{
    DEVICE_INTF_RET_TYPE ret = w25qxx_null_ptr_check(w25qxx);
    device_t *dev = w25qxx->dev;
    device_t *nss = (device_t *)dev->addr;
    if (ret != DEVICE_OK)
        return ret;

    ret = device_write_byte(nss, 0);
    ret = device_write_byte(dev, cmd);
    ret = device_write_byte(dev, data);
    ret = device_write_byte(nss, 1);

    return ret;
}
```

