/**
 * Copyright (c) 2022 iammingge
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * @file    W25Qxx.h
 * @brief   W25Qxx family driver source file
 * @author  iammingge
 *
 *      DATE             NAME                      DESCRIPTION
 *
 *   11-27-2022        iammingge                Initial Version 1.0
 *   04-14-2023        iammingge      			1. modify the function W25Qxx_ReadStatus "2^ret" -> "1<<ret"
 *											    2. align code 4 bytes
 *											    3. support 4 address mode
 *   05-01-2023        iammingge                1. Fix function W25Qxx_ Reset timing error, resulting in invalid device restart
 *                                              2. Add W25Qxx status register to restore factory parameter function W25Qxx_ SetFactory_ WriteStatusRegister
 *                                              3. Support for multiple device mounting
 *
 **/
#include <stdlib.h>
#include "w25qxx.h"

/* ----------------------------------------------------------------------------------------------------------------------
   |                                                     NOR FLASH                                                      |
   ----------------------------------------------------------------------------------------------------------------------
   |  Model  | Block |   Sector  |      Page      |             Byte             |            Bit           | Addr Mode |
   | W25Q02  | 4096  | 4096 * 16 | 4096 * 16 * 16 | 4096 * 16 * 16 * 256 (256MB) | 4096 * 16 * 16 * 256 * 8 |    3/4    |
   | W25Q01  | 2048  | 2048 * 16 | 2048 * 16 * 16 | 2048 * 16 * 16 * 256 (128MB) | 2048 * 16 * 16 * 256 * 8 |    3/4    |
   | W25Q512 | 1024  | 1024 * 16 | 1024 * 16 * 16 | 1024 * 16 * 16 * 256  (64MB) | 1024 * 16 * 16 * 256 * 8 |    3/4    |
   | W25Q256 |  512  |  512 * 16 |  512 * 16 * 16 |  512 * 16 * 16 * 256  (32MB) |  512 * 16 * 16 * 256 * 8 |    3/4    |
   | W25Q128 |  256  |  256 * 16 |  256 * 16 * 16 |  256 * 16 * 16 * 256  (16MB) |  256 * 16 * 16 * 256 * 8 |     3     |
   | W25Q64  |  128  |  128 * 16 |  128 * 16 * 16 |  128 * 16 * 16 * 256   (8MB) |  128 * 16 * 16 * 256 * 8 |     3     |
   | W25Q32  |   64  |   64 * 16 |   64 * 16 * 16 |   64 * 16 * 16 * 256   (4MB) |   64 * 16 * 16 * 256 * 8 |     3     |
   | W25Q16  |   32  |   32 * 16 |   32 * 16 * 16 |   32 * 16 * 16 * 256   (2MB) |   32 * 16 * 16 * 256 * 8 |     3     |
   | W25Q80  |   16  |   16 * 16 |   16 * 16 * 16 |   16 * 16 * 16 * 256   (1MB) |   16 * 16 * 16 * 256 * 8 |     3     |
   | W25Q40  |    8  |    8 * 16 |    8 * 16 * 16 |    8 * 16 * 16 * 256 (512KB) |    8 * 16 * 16 * 256 * 8 |     3     |
   | W25Q20  |    4  |    4 * 16 |    4 * 16 * 16 |    4 * 16 * 16 * 256 (256KB) |    4 * 16 * 16 * 256 * 8 |     3     |
   | W25X10  |    2  |    2 * 16 |    2 * 16 * 16 |    2 * 16 * 16 * 256 (128KB) |    2 * 16 * 16 * 256 * 8 |     3     |
   | W25X05  |    1  |    1 * 16 |    1 * 16 * 16 |    1 * 16 * 16 * 256  (64KB) |    1 * 16 * 16 * 256 * 8 |     3     |
   ----------------------------------------------------------------------------------------------------------------------
**/
/* Tool Function */
#define rbit(val, x) (((val) & (1 << (x))) >> (x))                   /* Read  1 bit */
#define wbit(val, x, a) (val = ((val) & ~(1 << (x))) | ((a) << (x))) /* Write 1 bit */
#define noruint(val) (val = !!(val))                                 /* Normalize data while keeping the logical value unchanged */
static uint8_t BcdToByte(uint16_t num)                               /* Calculate BCD(0 - 597) convert 1Byte(0 - 255) example : 20(0x14) ---> 0x20 */
{
    uint8_t d1, d2, d3;

    if (num > 597)
        return 0x00;

    d1 = (uint16_t)num & 0x000F;
    d2 = ((uint16_t)num & 0x00F0) >> 4;
    d3 = ((uint16_t)num & 0x0F00) >> 8;
    d1 = d1 + d2 * 10 + d3 * 100;

    return d1;
}
/* W25Qxx Cache */
static uint8_t W25QXX_CACHE[W25Qxx_SECTORSIZE];
/* W25Qxx Info List */
static w25qxx_info_t w25q_info_list[] = {
    /* Type    |Name    | ProgramPage | EraseSector | EraseBlock64 | EraseBlock32 | EraseChip */
    {W25X05, "W25X05", 1, 300, 1000, 800, 1000},       // W25X05CL     0.8
    {W25X10, "W25X10", 1, 300, 1000, 800, 1000},       // W25X10CL     0.8
    {W25Q20, "W25Q20", 1, 300, 1000, 800, 2000},       // W25Q20CL     0.8
    {W25Q40, "W25Q40", 1, 300, 1000, 800, 4000},       // W25Q40CL     0.8
    {W25Q80, "W25Q80", 4, 500, 2000, 1500, 8000},      // W25Q80DV     4
    {W25Q16, "W25Q16", 3, 400, 2000, 1600, 25000},     // W25Q16JV     3
    {W25Q32, "W25Q32", 3, 400, 2000, 1600, 50000},     // W25Q32JV     3
    {W25Q64, "W25Q64", 3, 400, 2000, 1600, 100000},    // W25Q64JV     3
    {W25Q128, "W25Q128", 3, 400, 2000, 1600, 200000},  // W25Q128JV    3
    {W25Q256, "W25Q256", 3, 400, 2000, 1600, 400000},  // W25Q256JV    3
    {W25Q512, "W25Q512", 4, 400, 2000, 1600, 1000000}, // W25Q512JV    3.5
    {W25Q01, "W25Q01", 4, 400, 2000, 1600, 1000000},   // W25Q01JV_DTR 3.5
    {W25Q02, "W25Q02", 4, 400, 2000, 1600, 1000000},   // W25Q02JV_DTR 3.5
};

static void w25qxx_fill_dummy(uint8_t *data, size_t size)
{
    if (data == NULL || size == 0)
        return;

    for (size_t i = 0; i < size; i++)
    {
        data[i] = W25Q_DUMMY;
    }
}

DEVICE_INTF_RET_TYPE w25qxx_null_ptr_check(w25qxx_handle_t *w25qxx)
{
    if (w25qxx == NULL)
    {
        return W25QXX_E_NULLPTR;
    }
    return device_null_ptr_check(w25qxx->dev);
}

DEVICE_INTF_RET_TYPE w25qxx_interface_init(w25qxx_handle_t *w25qxx, device_t *dev)
{
    if (w25qxx == NULL)
    {
        return W25QXX_E_NULLPTR;
    }
    w25qxx->dev = dev;

    return w25qxx_null_ptr_check(w25qxx);
}

DEVICE_INTF_RET_TYPE w25qxx_transfer(w25qxx_handle_t *w25qxx, w25qxx_transaction_t *transaction)
{
    DEVICE_INTF_RET_TYPE ret = w25qxx_null_ptr_check(w25qxx);
    device_t *dev = w25qxx->dev;
    device_t *nss = (device_t *)dev->addr;
    if (ret != DEVICE_OK)
        return ret;

    device_write_byte(nss, 0);
    /* set reset enable */
    device_write_byte(dev, transaction->cmd);
    if (transaction->tx_buffer != NULL && transaction->tx_len != 0)
    {
        ret = dev->write(transaction->tx_buffer, transaction->tx_len, dev->fp, dev->addr);
        if (ret != W25QXX_OK)
            return ret;
    }

    if (transaction->rx_buffer != NULL && transaction->rx_len != 0)
    {
        ret = dev->read(transaction->rx_buffer, transaction->rx_len, dev->fp, dev->addr);
        if (ret != W25QXX_OK)
            return ret;
    }
    if (transaction->cmd == W25Q_CMD_RSREG3 && transaction->rx_buffer)
    {
        get_platform()->println("%d\r\n", *(uint8_t *)transaction->rx_buffer);
    }
    

    return device_write_byte(nss, 1);
}

DEVICE_INTF_RET_TYPE w25qxx_send_cmd(w25qxx_handle_t *w25qxx, uint8_t cmd)
{
    w25qxx_transaction_t transaction = W25QXX_CMD_TRANSACTION_INIT(cmd);

    return w25qxx_transfer(w25qxx, &transaction);
}

DEVICE_INTF_RET_TYPE w25qxx_write_cmd(w25qxx_handle_t *w25qxx, uint8_t cmd, uint8_t data)
{
    w25qxx_transaction_t transaction = W25QXX_WRITE_TRANSACTION_INIT(cmd, &data, 1);

    return w25qxx_transfer(w25qxx, &transaction);
}

DEVICE_INTF_RET_TYPE w25qxx_read_cmd(w25qxx_handle_t *w25qxx, uint8_t cmd, uint8_t *reg_data)
{
    w25qxx_transaction_t transaction = W25QXX_READ_TRANSACTION_INIT(cmd, reg_data, 1);

    return w25qxx_transfer(w25qxx, &transaction);
}

DEVICE_INTF_RET_TYPE w25qxx_read_id_manufacturer(w25qxx_handle_t *w25qxx)
{
    DEVICE_INTF_RET_TYPE ret = W25QXX_OK;
    uint8_t tx_data[3] = {W25Q_DUMMY, W25Q_DUMMY, 0x00};
    uint8_t rx_data[2] = {0xFF, 0xFF};
    w25qxx_transaction_t transaction = W25QXX_TRANSACTION_INIT(W25Q_CMD_MANUFACTURER, tx_data, 3, rx_data, 2);

    ret = w25qxx_transfer(w25qxx, &transaction);
    if (ret != W25QXX_OK)
        return ret;

    w25qxx->id_manufacturer = ((rx_data[0] << 8) | rx_data[1]);

    config_device_info(w25qxx->dev, "%n%i%c", "w25qxx", "SPI", w25qxx->id_manufacturer);

    return W25QXX_OK;
}

DEVICE_INTF_RET_TYPE w25qxx_read_id_jedec(w25qxx_handle_t *w25qxx)
{
    DEVICE_INTF_RET_TYPE ret = W25QXX_OK;
    uint8_t rx_data[3] = {0xFF, 0xFF, 0xFF};
    w25qxx_transaction_t transaction = W25QXX_READ_TRANSACTION_INIT(W25Q_CMD_JEDECID, rx_data, 3);

    ret = w25qxx_transfer(w25qxx, &transaction);
    if (ret != DEVICE_OK)
        return ret;

    w25qxx->id_jedec = (rx_data[0] << 16) | (rx_data[1] << 8) | rx_data[2];

    return W25QXX_OK;
}

DEVICE_INTF_RET_TYPE w25qxx_read_id_unique(w25qxx_handle_t *w25qxx)
{
    uint8_t ret = W25QXX_OK;
    uint8_t tx_data[5] = {W25Q_DUMMY, W25Q_DUMMY, W25Q_DUMMY, W25Q_DUMMY, W25Q_DUMMY};
    uint8_t rx_data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    w25qxx_transaction_t transaction = W25QXX_TRANSACTION_INIT(W25Q_CMD_UNIQUEID, tx_data, 5, rx_data, 8);

#if W25QXX_4BADDR == 0
    transaction.tx_len = 4;
#endif
    ret = w25qxx_transfer(w25qxx, &transaction);
    if (ret != DEVICE_OK)
        return ret;

    w25qxx->id_unique = 0;
    for (uint8_t i = 0; i < 8; i++)
    {
        w25qxx->id_unique |= ((uint64_t)rx_data[i] << ((7 - i) * 8));
    }
    return W25QXX_OK;
}

/* Software reset */
DEVICE_INTF_RET_TYPE w25qxx_reset(w25qxx_handle_t *w25qxx)
{
    DEVICE_INTF_RET_TYPE ret = w25qxx_send_cmd(w25qxx, W25Q_CMD_ENRESET);
    if (ret != W25QXX_OK)
        return ret;

    ret = w25qxx_send_cmd(w25qxx, W25Q_CMD_RESETDEV);

    platform_delay_ms(1);

    return ret;
}

/* Power Enable */
DEVICE_INTF_RET_TYPE w25qxx_power_enable(w25qxx_handle_t *w25qxx)
{
    DEVICE_INTF_RET_TYPE ret = w25qxx_send_cmd(w25qxx, W25Q_CMD_POWEREN);

    platform_delay_ms(1);
    return ret;
}

/* Power Disable */
DEVICE_INTF_RET_TYPE w25qxx_power_disable(w25qxx_handle_t *w25qxx)
{
    DEVICE_INTF_RET_TYPE ret = w25qxx_send_cmd(w25qxx, W25Q_CMD_POWERDEN);

    platform_delay_ms(1);
    return ret;
}

/* Write Enable for Volatile Status Register */
/** This gives more flexibility to change the system configuration and memory protection schemes quickly without
 * waiting for the typical non-volatile bit write cycles or affecting the endurance of the Status Register non-volatile bits
 **/
DEVICE_INTF_RET_TYPE w25qxx_volatile_sr_write_enable(w25qxx_handle_t *w25qxx)
{
    return w25qxx_send_cmd(w25qxx, W25Q_CMD_VOLATILESREN);
}

/**
 * @param[in] sr_read_cmd: status register cmd:
 *              Options: -W25Q_CMD_RSREG1
 *                       -W25Q_CMD_RSREG2
 *                       -W25Q_CMD_RSREG3
 */
w25qxx_sr_t *w25qxx_sr_init(uint8_t sr_read_cmd, uint8_t *sr, uint8_t read)
{
    w25qxx_sr_t *status_register = calloc(1, sizeof(w25qxx_sr_t));
    if (status_register == NULL)
    {
        return NULL;
    }
    if (read == 0)
        sr_read_cmd -= 4;

    status_register->sr = sr;
    status_register->sr_cmd = sr_read_cmd;

    return status_register;
}

void w25qxx_sr_deinit(w25qxx_sr_t **status_register)
{
    if (*status_register == NULL)
    {
        return;
    }

    free(*status_register);
    *status_register = NULL;
}

void w25qxx_u8_deinit(uint8_t **buffer)
{
    if (*buffer == NULL)
    {
        return;
    }

    free(*buffer);
    *buffer = NULL;
}

DEVICE_INTF_RET_TYPE w25qxx_sr_update(w25qxx_sr_t *status_register, uint8_t sr)
{
    if (status_register == NULL)
    {
        return W25QXX_E_NULLPTR;
    }

    memcpy(status_register->sr, &sr, sizeof(uint8_t));
    return W25QXX_OK;
}

w25qxx_sr_t *w25qxx_select_sr(w25qxx_handle_t *w25qxx, uint8_t select_sr_1_2_3, uint8_t read)
{
    w25qxx_sr_t *status_register = NULL;
    switch (select_sr_1_2_3)
    {
    case 1:
        status_register = w25qxx_sr_init(W25Q_CMD_RSREG1, &w25qxx->status_register1, read);
        break;
    case 2:
        status_register = w25qxx_sr_init(W25Q_CMD_RSREG2, &w25qxx->status_register2, read);
        break;
    case 3:
        status_register = w25qxx_sr_init(W25Q_CMD_RSREG3, &w25qxx->status_register3, read);
        break;
    default:
        break;
    }

    return status_register;
}

/* Write Enable */
DEVICE_INTF_RET_TYPE w25qxx_write_enable(w25qxx_handle_t *w25qxx)
{
    DEVICE_INTF_RET_TYPE ret = w25qxx_send_cmd(w25qxx, W25Q_CMD_WEN);
    if (ret != W25QXX_OK)
        return ret;

    return w25qxx_read_status_register(w25qxx, 1);
}

/* Write Disable */
DEVICE_INTF_RET_TYPE w25qxx_write_disable(w25qxx_handle_t *w25qxx)
{
    DEVICE_INTF_RET_TYPE ret = w25qxx_send_cmd(w25qxx, W25Q_CMD_WDEN);
    if (ret != W25QXX_OK)
        return ret;

    return w25qxx_read_status_register(w25qxx, 1);
}

/* Set 4 bytes address mode */
DEVICE_INTF_RET_TYPE w25qxx_4_byte_mode(w25qxx_handle_t *w25qxx)
{
    DEVICE_INTF_RET_TYPE ret = w25qxx_send_cmd(w25qxx, W25Q_CMD_4ByteAddrEN);
    if (ret != W25QXX_OK)
        return ret;

    return w25qxx_read_status_register(w25qxx, 3);
}

/* Set 3 bytes address mode */
DEVICE_INTF_RET_TYPE w25qxx_3_byte_mode(w25qxx_handle_t *w25qxx)
{
    DEVICE_INTF_RET_TYPE ret = w25qxx_send_cmd(w25qxx, W25Q_CMD_4ByteAddrEN);
    if (ret != W25QXX_OK)
        return ret;

    ret = w25qxx_read_status_register(w25qxx, 3);
    if (ret != W25QXX_OK)
        return ret;

    return w25qxx_write_extended_register(w25qxx, 0x00);
}

/* Erase/Program suspend (SUS = 0 & BUSY = 1) */
/* Erase Suspend instruction use scope       : 1. Erase operation (20h, 52h, D8h, 44h)             (��)
 * 											   2. Erase operation (C7h, 60h)                       (x)
 * Commands Supported During Erase Suspend   : 1. Write Status Register instruction (01h)          (x)
 *                                             2. Erase instruction (20h, 52h, D8h, C7h, 60h, 44h) (x)
 *                                             3. Read instruction (03h, 0Bh, 5Ah, 48h)            (��)
 * Program Suspend instruction use scope     : 1. Page Program operation (02h, 42h)                (��)
 *                                             2. Quad Page Program operation (32h)                (��)
 * Commands Supported During Program Suspend : 1. Write Status Register instruction (01h)          (x)
 *                                             2. Program instructions (02h, 32h, 42h)             (x)
 *                                             3. Read instruction (03h, 0Bh, 5Ah, 48h)            (��)
 **/
DEVICE_INTF_RET_TYPE w25qxx_suspend(w25qxx_handle_t *w25qxx)
{
    DEVICE_INTF_RET_TYPE ret = w25qxx_send_cmd(w25qxx, W25Q_CMD_EWSUSPEND);
    if (ret != W25QXX_OK)
        return ret;

    /* tSUS max = 20us */
    platform_delay_ms(1);

    /* read back Busy Bit */
    ret = w25qxx_read_status_register(w25qxx, 1);
    if (ret != W25QXX_OK)
        return ret;

    /* read back Suspend Bit */
    return w25qxx_read_status_register(w25qxx, 2);
}

DEVICE_INTF_RET_TYPE w25qxx_resume(w25qxx_handle_t *w25qxx)
{
    DEVICE_INTF_RET_TYPE ret = w25qxx_send_cmd(w25qxx, W25Q_CMD_EWRESUME);
    if (ret != W25QXX_OK)
        return ret;

    /* tSUS max = 20us */
    platform_delay_ms(1);

    /* read back Busy Bit */
    ret = w25qxx_read_status_register(w25qxx, 1);
    if (ret != W25QXX_OK)
        return ret;

    /* read back Suspend Bit */
    return w25qxx_read_status_register(w25qxx, 2);
}

static void pack_u32(uint8_t *data, uint32_t byte_addr)
{
    *data++ = (uint8_t)((byte_addr) >> 24);
    *data++ = (uint8_t)((byte_addr) >> 16);
    *data++ = (uint8_t)((byte_addr) >> 8);
    *data++ = (uint8_t)byte_addr;
}

/**
 * @brief W25Qxx Read sector/block lock of the current address status
 */
DEVICE_INTF_RET_TYPE w25qxx_read_lock(w25qxx_handle_t *w25qxx, uint32_t byte_addr, uint8_t *status)
{
    uint8_t tx_data[5];
    w25qxx_transaction_t transaction = W25QXX_TRANSACTION_INIT(W25Q_CMD_RBLOCKLOCK, tx_data, 5, status, 1);
    DEVICE_INTF_RET_TYPE ret = W25QXX_OK;

    pack_u32(tx_data, byte_addr);
    tx_data[4] = W25Q_DUMMY;

#if W25QXX_4BADDR == 0
    transaction.tx_buffer = (void *)(tx_data + 1);
    transaction.tx_len--;
    /* Address > 0xFFFFFF */
    if (byte_addr > 0xFFFFFF)
    {
        ret = w25qxx_write_extended_register(w25qxx, (uint8_t)((byte_addr) >> 24));
    }
    else
    {
        ret = w25qxx_write_extended_register(w25qxx, 0x00);
    }
#endif
    if (ret != W25QXX_OK)
        return ret;

    return w25qxx_transfer(w25qxx, &transaction);
}

/* W25Qxx Read/Write ExtendedRegister
 * 1. The Extended Address Register is only effective when the device is in the 3-Byte Address Mode.
 * 2. When the device operates in the 4-Byte Address Mode (ADS=1), any command with address input of
 *    A31-A24 will replace the Extended Address Register values.
 * 3. It is recommended to check and update the Extended Address Register if necessary when the device
 *    is switched from 4-Byte to 3-Byte Address Mode.
 * 4. Upon power up or the execution of a Software/Hardware Reset, the Extended Address Register bit
 *    values will be cleared to 0.
 **/
DEVICE_INTF_RET_TYPE w25qxx_read_extended_register(w25qxx_handle_t *w25qxx)
{
    uint8_t tx_data = W25Q_DUMMY;
    w25qxx_transaction_t transaction = W25QXX_TRANSACTION_INIT(W25Q_CMD_REXTREG, &tx_data, 1, &w25qxx->extended_register, 1);

    return w25qxx_transfer(w25qxx, &transaction);
}

DEVICE_INTF_RET_TYPE w25qxx_write_extended_register(w25qxx_handle_t *w25qxx, uint8_t extended_addr)
{
    DEVICE_INTF_RET_TYPE ret = W25QXX_OK;
    w25qxx_transaction_t transaction = W25QXX_WRITE_TRANSACTION_INIT(W25Q_CMD_WEXTREG, &extended_addr, 1);

    w25qxx->extended_register = extended_addr;
    /* write enable */
    ret = w25qxx_write_enable(w25qxx);
    if (ret != W25QXX_OK)
    {
        return ret;
    }

    /* write extended address register */
    ret = w25qxx_transfer(w25qxx, &transaction);

    /* write disable */
    return w25qxx_write_disable(w25qxx);
}

DEVICE_INTF_RET_TYPE w25qxx_read_status_register2(w25qxx_handle_t *w25qxx, w25qxx_sr_t *status_register)
{
    if (status_register == NULL)
    {
        return W25QXX_E_NULLPTR;
    }

    return w25qxx_read_cmd(w25qxx, status_register->sr_cmd, status_register->sr);
}

/**
 * @brief W25Qxx Read/Write StatusRegister
 */
DEVICE_INTF_RET_TYPE w25qxx_read_status_register(w25qxx_handle_t *w25qxx, uint8_t select_sr_1_2_3)
{
    DEVICE_INTF_RET_TYPE ret = W25QXX_OK;
    w25qxx_sr_t *status_register = w25qxx_select_sr(w25qxx, select_sr_1_2_3, true);

    ret = w25qxx_read_status_register2(w25qxx, status_register);
    w25qxx_sr_deinit(&status_register);

    return ret;
}

DEVICE_INTF_RET_TYPE w25qxx_write_status_register2(w25qxx_handle_t *w25qxx, w25qxx_sr_t *status_register, uint8_t data)
{
    DEVICE_INTF_RET_TYPE ret = w25qxx_write_enable(w25qxx);

    if (ret != W25QXX_OK)
        return ret;

    ret = w25qxx_sr_update(status_register, data);
    if (ret != W25QXX_OK)
        return ret;

    /* write status register */
    ret = w25qxx_write_cmd(w25qxx, status_register->sr_cmd, *status_register->sr);

    ret = w25qxx_write_disable(w25qxx);

    return ret;
}

/* Write Status Register1/2/3 */
DEVICE_INTF_RET_TYPE w25qxx_write_status_register(w25qxx_handle_t *w25qxx, uint8_t select_sr_1_2_3, uint8_t data)
{
    DEVICE_INTF_RET_TYPE ret = W25QXX_OK;
    w25qxx_sr_t *status_register = w25qxx_select_sr(w25qxx, select_sr_1_2_3, false);

    ret = w25qxx_write_status_register2(w25qxx, status_register, data);

    w25qxx_sr_deinit(&status_register);
    /* tW max = 15ms */
    platform_delay_ms(15);

    return ret;
}

DEVICE_INTF_RET_TYPE w25qxx_volatile_sr_write_status_register2(w25qxx_handle_t *w25qxx, w25qxx_sr_t *status_register, uint8_t data)
{
    DEVICE_INTF_RET_TYPE ret = w25qxx_volatile_sr_write_enable(w25qxx);
    if (ret != W25QXX_OK)
        return ret;

    ret = w25qxx_sr_update(status_register, data);
    if (ret != W25QXX_OK)
        return ret;

    return w25qxx_write_cmd(w25qxx, status_register->sr_cmd, *status_register->sr);
}

/* Volatile write Status Register1/2/3 */
/* This gives more flexibility to change the system configuration and memory protection schemes quickly without
 * waiting for the typical non-volatile bit write cycles or affecting the endurance of the Status Register non-volatile bits
 **/
DEVICE_INTF_RET_TYPE w25qxx_volatile_sr_write_status_register(w25qxx_handle_t *w25qxx, uint8_t select_sr_1_2_3, uint8_t data)
{
    w25qxx_sr_t *status_register = w25qxx_select_sr(w25qxx, select_sr_1_2_3, false);
    DEVICE_INTF_RET_TYPE ret = w25qxx_volatile_sr_write_status_register2(w25qxx, status_register, data);

    w25qxx_sr_deinit(&status_register);
    return ret;
}

/* Device set to factory parameter (QE = 1) */
void w25qxx_set_factory_write_status_register(w25qxx_handle_t *w25qxx) /* Device set to factory parameter (QE = 1) */
{
    /* Status Register 1 */
    w25qxx_write_status_register(w25qxx, 1, 0x00);

    /* Status Register 2
     * Note: The QE bit is set to 1 at the factory and cannot be set to 0.
     *       Some models have register QE set to 0.
     **/
    w25qxx_write_status_register(w25qxx, 2, 0x02);

    /* Status Register 3 */
    w25qxx_write_status_register(w25qxx, 3, 0xE0);

    /* Read Register */
    w25qxx_read_status_register(w25qxx, 1);
    w25qxx_read_status_register(w25qxx, 2);
    w25qxx_read_status_register(w25qxx, 3);
}

DEVICE_INTF_RET_TYPE w25qxx_rbit_wel(w25qxx_handle_t *w25qxx, uint8_t *wel)
{
    DEVICE_INTF_RET_TYPE ret = w25qxx_read_status_register(w25qxx, 1);

    /* read WEL bit */
    *wel = rbit(w25qxx->status_register1, 1);
    return ret;
}

DEVICE_INTF_RET_TYPE w25qxx_rbit_busy(w25qxx_handle_t *w25qxx, uint8_t *busy)
{
    /* read StatusRegister1 */
    DEVICE_INTF_RET_TYPE ret = w25qxx_read_status_register(w25qxx, 1);

    /* read BUSY bit */
    *busy = rbit(w25qxx->status_register1, 0);

    return ret;
}

DEVICE_INTF_RET_TYPE w25qxx_rbit_sus(w25qxx_handle_t *w25qxx, uint8_t *sus)
{
    /* read StatusRegister1 */
    DEVICE_INTF_RET_TYPE ret = w25qxx_read_status_register(w25qxx, 2);

    /* read SUS bit */
    *sus = rbit(w25qxx->status_register2, 0);

    return ret;
}

DEVICE_INTF_RET_TYPE w25qxx_rbit_ads(w25qxx_handle_t *w25qxx, uint8_t *ads)
{
    /* read StatusRegister1 */
    DEVICE_INTF_RET_TYPE ret = w25qxx_read_status_register(w25qxx, 3);

    /* read ADS bit */
    *ads = rbit(w25qxx->status_register3, 7);

    return ret;
}

DEVICE_INTF_RET_TYPE w25qxx_wbit(w25qxx_handle_t *w25qxx, w25qxx_srm_t srm, uint8_t select_sr_1_2_3, uint8_t bit, uint8_t bit_pos)
{
    w25qxx_sr_t *status_register = w25qxx_select_sr(w25qxx, select_sr_1_2_3, true);
    /* read StatusRegister */
    DEVICE_INTF_RET_TYPE ret = w25qxx_read_status_register2(w25qxx, status_register);

    if (ret != W25QXX_OK)
        return ret;

    if (bit_pos <= 7)
    {
        noruint(bit);
        wbit(*status_register->sr, bit_pos, bit);
    }

    /* write StatusRegister */
    status_register->sr_cmd -= 4;
    if (srm == W25Qxx_VOLATILE)
    {
        ret = w25qxx_volatile_sr_write_status_register2(w25qxx, status_register, *status_register->sr);
    }
    else
    {
        ret = w25qxx_write_status_register2(w25qxx, status_register, *status_register->sr);
    }

    if (ret != W25QXX_OK)
        return ret;

    /* read back StatusRegister1 */
    status_register->sr_cmd += 4;
    ret = w25qxx_read_status_register2(w25qxx, status_register);
    w25qxx_sr_deinit(&status_register);
    get_platform()->println("%x\r\n", w25qxx->status_register3);

    return ret;
}

DEVICE_INTF_RET_TYPE w25qxx_wbit_srp(w25qxx_handle_t *w25qxx, w25qxx_srm_t srm, uint8_t bit)
{
    /* read StatusRegister1 */
    DEVICE_INTF_RET_TYPE ret = w25qxx_read_status_register(w25qxx, 1);

    if (ret != W25QXX_OK)
        return ret;

    return w25qxx_wbit(w25qxx, srm, 1, bit, 7);
}

DEVICE_INTF_RET_TYPE w25qxx_wbit_tb(w25qxx_handle_t *w25qxx, w25qxx_srm_t srm, uint8_t bit)
{
    /* read StatusRegister1 */
    DEVICE_INTF_RET_TYPE ret = w25qxx_read_status_register(w25qxx, 1);

    if (ret != W25QXX_OK)
        return ret;

    return w25qxx_wbit(w25qxx, srm, 1, bit, 6);
}

DEVICE_INTF_RET_TYPE w25qxx_wbit_cmp(w25qxx_handle_t *w25qxx, w25qxx_srm_t srm, uint8_t bit)
{
    /* read StatusRegister2 */
    DEVICE_INTF_RET_TYPE ret = w25qxx_read_status_register(w25qxx, 2);

    if (ret != W25QXX_OK)
        return ret;

    return w25qxx_wbit(w25qxx, srm, 2, bit, 6);
}

DEVICE_INTF_RET_TYPE w25qxx_wbit_qe(w25qxx_handle_t *w25qxx, w25qxx_srm_t srm, uint8_t bit)
{
    /* read StatusRegister2 */
    DEVICE_INTF_RET_TYPE ret = w25qxx_read_status_register(w25qxx, 2);

    if (ret != W25QXX_OK)
        return ret;

    return w25qxx_wbit(w25qxx, srm, 2, bit, 1);
}

DEVICE_INTF_RET_TYPE w25qxx_wbit_srl(w25qxx_handle_t *w25qxx, w25qxx_srm_t srm, uint8_t bit)
{
    /* read StatusRegister2 */
    DEVICE_INTF_RET_TYPE ret = w25qxx_read_status_register(w25qxx, 2);

    if (ret != W25QXX_OK)
        return ret;

    return w25qxx_wbit(w25qxx, srm, 2, bit, 0);
}

DEVICE_INTF_RET_TYPE w25qxx_wbit_wps(w25qxx_handle_t *w25qxx, w25qxx_srm_t srm, uint8_t bit)
{
    /* read StatusRegister3 */
    DEVICE_INTF_RET_TYPE ret = w25qxx_read_status_register(w25qxx, 3);

    if (ret != W25QXX_OK)
        return ret;

    return w25qxx_wbit(w25qxx, srm, 3, bit, 2);
}

// DEVICE_INTF_RET_TYPE w25qxx_wbit_wps(w25qxx_handle_t *w25qxx, w25qxx_srm_t srm, uint8_t bit)
// {
//     uint8_t wel = 0;
//     w25qxx_rbit_wel(w25qxx, &wel);
//     get_platform()->println("____%d\r\n", wel);
//     w25qxx_send_cmd(w25qxx, 0x50);
//     w25qxx_write_cmd(w25qxx, 0x11, bit);
//     w25qxx_read_status_register(w25qxx, 3);
//     get_platform()->println("____%d\r\n", w25qxx->status_register3);

//     return W25QXX_OK;
// }

DEVICE_INTF_RET_TYPE w25qxx_wbit_drv(w25qxx_handle_t *w25qxx, w25qxx_srm_t srm, uint8_t bit)
{
    /* read StatusRegister3 */
    DEVICE_INTF_RET_TYPE ret = w25qxx_read_status_register(w25qxx, 3);

    if (ret != W25QXX_OK)
        return ret;

    /* write DRV bit (0110 0000 -> 0x60) */
    bit &= 0x03;
    w25qxx->status_register3 &= ~0x60;
    w25qxx->status_register3 |= (bit << 5);

    return w25qxx_wbit(w25qxx, srm, 3, bit, 8);
}

DEVICE_INTF_RET_TYPE w25qxx_wbit_bp(w25qxx_handle_t *w25qxx, w25qxx_srm_t srm, uint8_t bit)
{
    /* read StatusRegister1 */
    DEVICE_INTF_RET_TYPE ret = w25qxx_read_status_register(w25qxx, 1);

    if (ret != W25QXX_OK)
        return ret;

    /* write BP bit (0011 1100 -> 0x3C) */
    w25qxx->status_register3 &= ~0x3C;
    w25qxx->status_register3 |= (bit << 2);

    return w25qxx_wbit(w25qxx, srm, 1, bit, 8);
}

DEVICE_INTF_RET_TYPE w25qxx_wbit_lb(w25qxx_handle_t *w25qxx, w25qxx_srm_t srm, uint8_t bit)
{
    /* read StatusRegister2 */
    DEVICE_INTF_RET_TYPE ret = w25qxx_read_status_register(w25qxx, 2);

    if (ret != W25QXX_OK)
        return ret;

    /* write LB bit (0011 1000 -> 0x38)*/
    bit &= 0x07;
    w25qxx->status_register3 &= ~0x38;
    w25qxx->status_register3 |= (bit << 3);

    return w25qxx_wbit(w25qxx, srm, 2, bit, 8);
}

DEVICE_INTF_RET_TYPE w25qxx_wbit_adp(w25qxx_handle_t *w25qxx, uint8_t bit)
{
    DEVICE_INTF_RET_TYPE ret = w25qxx_read_status_register(w25qxx, 3);

    if (ret != W25QXX_OK)
        return ret;

    return w25qxx_wbit(w25qxx, W25Qxx_NON_VOLATILE, 3, bit, 1);
}

DEVICE_INTF_RET_TYPE w25qxx_read_status(w25qxx_handle_t *w25qxx, uint8_t *status)
{
    DEVICE_INTF_RET_TYPE ret = W25QXX_OK;
    uint8_t res = 0;

    ret = w25qxx_rbit_busy(w25qxx, &res);
    if (ret != W25QXX_OK)
        return ret;

    *status = 0;
    *status |= res;

    ret = w25qxx_rbit_sus(w25qxx, &res);
    *status = 1 << (*status | (res << 1));
    return ret;
}

/**
 * @brief W25Qxx read current status
 */
DEVICE_INTF_RET_TYPE w25qxx_is_status(w25qxx_handle_t *w25qxx, uint8_t select_status, uint32_t timeout)
{
    DEVICE_INTF_RET_TYPE ret = W25QXX_OK;
    uint32_t time = timeout;
    w25qxx_status_t curstatus = W25Qxx_STATUS_IDLE;

    do
    {
        /* Read current chip running status */
        ret = w25qxx_read_status(w25qxx, (uint8_t *)&curstatus);
        if (ret != W25QXX_OK)
            return ret;

        if (curstatus & select_status)
        {
            /* current status correct */
            return W25QXX_OK;
        }

        if (time-- == 0)
            break;

        platform_delay_ms(1);

    } while (time);

    /* current status err */
    return W25Qxx_E_STATUS;
}

/* Global Sector/Block Unlock */
/* W25Qxx Sector/Blcok Lock protect for " WPS = 1 "
 * WPS = 0 : The Device will only utilize CMP, TB, BP[3:0] bits to protect specific areas of the array.
 * WPS = 1 : The Device will utilize the Individual Block Locks for write protection.
 * Example : W25Q256FVEIQ
 * 		     Sector(Block 0)   : 0-15
 *           Block 1-510
 * 		     Sector(Block 511) : 0-15
 *           W25Q128JVSIQ
 * 		     Sector(Block 0)   : 0-15
 *           Block 1-254
 * 		     Sector(Block 255) : 0-15
 **/
DEVICE_INTF_RET_TYPE w25qxx_global_unlock(w25qxx_handle_t *w25qxx)
{
    DEVICE_INTF_RET_TYPE ret = W25QXX_OK;

    if (rbit(w25qxx->status_register3, 2) == 0x00)
    {
        return W25Qxx_E_WPSMODE;
    }

    ret = w25qxx_write_enable(w25qxx);
    if (ret != W25QXX_OK)
        return ret;

    ret = w25qxx_send_cmd(w25qxx, W25Q_CMD_WALLBLOCKUNLOCK);
    return w25qxx_write_disable(w25qxx);
}

/* Global Sector/Block Locked */
DEVICE_INTF_RET_TYPE w25qxx_global_locked(w25qxx_handle_t *w25qxx)
{
    DEVICE_INTF_RET_TYPE ret = W25QXX_OK;

    if (rbit(w25qxx->status_register3, 2) == 0x00)
    {
        return W25Qxx_E_WPSMODE;
    }

    ret = w25qxx_write_enable(w25qxx);
    if (ret != W25QXX_OK)
        return ret;

    ret = w25qxx_send_cmd(w25qxx, W25Q_CMD_WALLBLOCKLOCK);
    return w25qxx_write_disable(w25qxx);
}

/* Individual Sector/Block Unlock */
DEVICE_INTF_RET_TYPE w25qxx_set_individual_lock(w25qxx_handle_t *w25qxx, uint32_t byte_addr, uint8_t locked)
{
    uint8_t tx_data[4];
    w25qxx_transaction_t transaction = W25QXX_WRITE_TRANSACTION_INIT(W25Q_CMD_WSIGBLOCKUNLOCK, tx_data, 4);
    DEVICE_INTF_RET_TYPE ret = W25QXX_OK;

    if (rbit(w25qxx->status_register3, 2) == 0x00)
    {
        return W25Qxx_E_WPSMODE;
    }

    pack_u32(tx_data, byte_addr);

    if (locked)
    {
        transaction.cmd = W25Q_CMD_WSIGBLOCKLOCK;
    }
    
    



#if W25QXX_4BADDR == 0
    transaction.tx_buffer = (void *)(tx_data + 1);
    transaction.tx_len--;
    /* Address > 0xFFFFFF */
    if (byte_addr > 0xFFFFFF)
    {
        ret = w25qxx_write_extended_register(w25qxx, (uint8_t)((byte_addr) >> 24));
    }
    else
    {
        ret = w25qxx_write_extended_register(w25qxx, 0x00);
    }
#endif

    if (ret != W25QXX_OK)
        return ret;

    ret = w25qxx_write_enable(w25qxx);
    w25qxx_transfer(w25qxx, &transaction);
    return w25qxx_write_disable(w25qxx);
}

DEVICE_INTF_RET_TYPE w25qxx_individual_unlock(w25qxx_handle_t *w25qxx, uint32_t byte_addr)
{
    return w25qxx_set_individual_lock(w25qxx, byte_addr, false);
}

DEVICE_INTF_RET_TYPE w25qxx_individual_locked(w25qxx_handle_t *w25qxx, uint32_t byte_addr)
{
    return w25qxx_set_individual_lock(w25qxx, byte_addr, true);
}

/**
 * @brief W25Qxx Main/SFDP/Security Storage read/program/erase function
 */
DEVICE_INTF_RET_TYPE w25qxx_erase_chip(w25qxx_handle_t *w25qxx)
{
    DEVICE_INTF_RET_TYPE ret = w25qxx_is_status(w25qxx, W25Qxx_STATUS_IDLE, 0);
    if (ret != W25QXX_OK)
    {
        return ret;
    }

    ret = w25qxx_write_enable(w25qxx);
    ret = w25qxx_send_cmd(w25qxx, W25Q_CMD_ECHIP);
    ret = w25qxx_write_disable(w25qxx);

    return w25qxx_is_status(w25qxx, W25Qxx_STATUS_IDLE, 0);
}

DEVICE_INTF_RET_TYPE w25qxx_erase(w25qxx_handle_t *w25qxx, uint8_t cmd, uint32_t byte_addr, uint32_t timeout)
{
    DEVICE_INTF_RET_TYPE ret = W25QXX_OK;
    uint8_t tx_data[4];
    w25qxx_transaction_t transaction = W25QXX_WRITE_TRANSACTION_INIT(cmd, tx_data, 4);

    ret = w25qxx_is_status(w25qxx, W25Qxx_STATUS_IDLE | W25Qxx_STATUS_SUSPEND, 0);
    if (ret != W25QXX_OK)
    {
        return ret;
    }
    pack_u32(tx_data, byte_addr);

#if W25QXX_4BADDR == 0
    transaction.tx_buffer = (void *)(tx_data + 1);
    transaction.tx_len--;
    /* Address > 0xFFFFFF */
    if (byte_addr > 0xFFFFFF)
    {
        ret = w25qxx_write_extended_register(w25qxx, (uint8_t)((byte_addr) >> 24));
    }
    else
    {
        ret = w25qxx_write_extended_register(w25qxx, 0x00);
    }
#endif

    if (ret != W25QXX_OK)
        return ret;

    ret = w25qxx_write_enable(w25qxx);
    ret = w25qxx_transfer(w25qxx, &transaction);
    ret = w25qxx_write_disable(w25qxx);

    return w25qxx_is_status(w25qxx, W25Qxx_STATUS_IDLE | W25Qxx_STATUS_SUSPEND, timeout);
}

/* Erase block of 64k */
DEVICE_INTF_RET_TYPE w25qxx_erase_block64(w25qxx_handle_t *w25qxx, uint32_t block64_addr)
{
    if (block64_addr >= w25qxx->num_block)
    {
        return W25Qxx_E_BLOCK64ADDRBOUND;
    }

    /* calculate sector address */
    block64_addr *= w25qxx->size_block;

    return w25qxx_erase(w25qxx, W25Q_CMD_E64KBLOCK, block64_addr, w25qxx->info.erase_max_time_block64);
}

/* Erase block of 32k */
DEVICE_INTF_RET_TYPE w25qxx_erase_block32(w25qxx_handle_t *w25qxx, uint32_t block32_addr)
{
    /* Determine if Block 32 Addrress Bound */
    if (block32_addr >= w25qxx->num_block * 2)
    {
        return W25Qxx_E_BLOCK32ADDRBOUND;
    }

    /* calculate sector address */
    block32_addr *= (w25qxx->num_block >> 1); /* Block32Addr *= (dev->sizeBlock / 2); */

    return w25qxx_erase(w25qxx, W25Q_CMD_E32KBLOCK, block32_addr, w25qxx->info.erase_max_time_block32);
}

/* Erase sector of 4k (Notes : 150ms) */
DEVICE_INTF_RET_TYPE w25qxx_erase_sector(w25qxx_handle_t *w25qxx, uint32_t sector_addr)
{
    if (sector_addr >= w25qxx->num_sector)
    {
        return W25Qxx_E_SECTORADDRBOUND;
    }

    sector_addr *= w25qxx->size_sector;
    return w25qxx_erase(w25qxx, W25Q_CMD_ESECTOR, sector_addr, w25qxx->info.erase_max_time_sector);
}

/* Erase security page of 256Byte (Notes : 150ms) */
DEVICE_INTF_RET_TYPE w25qxx_erase_security(w25qxx_handle_t *w25qxx, uint32_t sector_addr)
{
    DEVICE_INTF_RET_TYPE ret = W25QXX_OK;
    uint8_t tx_data[4] = {0x00, 0x00, (uint8_t)(sector_addr << 4), 0x00};
    w25qxx_transaction_t transaction = W25QXX_WRITE_TRANSACTION_INIT(W25Q_CMD_ESECREG, tx_data, 4);

    if (sector_addr > 3 || sector_addr == 0)
    {
        return W25Qxx_E_PAGEADDRBOUND;
    }

    ret = w25qxx_is_status(w25qxx, W25Qxx_STATUS_IDLE | W25Qxx_STATUS_SUSPEND, 0);
    if (ret != W25QXX_OK)
    {
        return ret;
    }

#if W25QXX_4BADDR == 0
    transaction.tx_buffer = (void *)(tx_data + 1);
    transaction.tx_len--;
#endif

    if (ret != W25QXX_OK)
        return ret;

    ret = w25qxx_write_enable(w25qxx);
    ret = w25qxx_transfer(w25qxx, &transaction);
    ret = w25qxx_write_disable(w25qxx);

    return w25qxx_is_status(w25qxx, W25Qxx_STATUS_IDLE | W25Qxx_STATUS_SUSPEND, w25qxx->info.erase_max_time_sector);
}

DEVICE_INTF_RET_TYPE w25qxx_read(w25qxx_handle_t *w25qxx, uint8_t *p_buffer, uint32_t byte_addr, uint16_t num_byte_to_read)
{
    DEVICE_INTF_RET_TYPE ret = W25QXX_OK;
    uint8_t tx_data[5];
#if W25QXX_FASTREAD
    uint8_t cmd = W25Q_CMD_FASTREAD;
    uint8_t tx_len = 5;
#else
    uint8_t cmd = W25Q_CMD_READ;
    uint8_t tx_len = 4;
#endif
    w25qxx_transaction_t transaction = W25QXX_TRANSACTION_INIT(cmd, tx_data, tx_len, p_buffer, num_byte_to_read);

    pack_u32(tx_data, byte_addr);
    tx_data[4] = W25Q_DUMMY;
    w25qxx_fill_dummy(p_buffer, num_byte_to_read);

    if (num_byte_to_read == 0x00)
    {
        return W25Qxx_E_INVALID;
    }

    ret = w25qxx_is_status(w25qxx, W25Qxx_STATUS_IDLE | W25Qxx_STATUS_SUSPEND, 0);
    if (ret != W25QXX_OK)
        return ret;

    if (byte_addr + num_byte_to_read > w25qxx->size_chip)
    {
        return W25Qxx_E_BYTEADDRBOUND;
    }

#if W25QXX_4BADDR == 0
    transaction.tx_buffer = (void *)(tx_data + 1);
    transaction.tx_len--;
    /* Address > 0xFFFFFF */
    if (byte_addr > 0xFFFFFF)
    {
        ret = w25qxx_write_extended_register(w25qxx, (uint8_t)((byte_addr) >> 24));
    }
    else
    {
        ret = w25qxx_write_extended_register(w25qxx, 0x00);
    }
#endif

    if (ret != W25QXX_OK)
        return ret;

    return w25qxx_transfer(w25qxx, &transaction);
}

DEVICE_INTF_RET_TYPE w25qxx_read_security(w25qxx_handle_t *w25qxx, uint8_t *p_buffer, uint32_t byte_addr, uint16_t num_byte_to_read)
{
    uint32_t num_page = 0;
    uint32_t start_addr = 0;
    DEVICE_INTF_RET_TYPE ret = W25QXX_OK;
    uint8_t tx_data[5] = {0x00, 0x00, (uint8_t)((byte_addr) >> 8), byte_addr & 0xFF, W25Q_DUMMY};
    w25qxx_transaction_t transaction = W25QXX_TRANSACTION_INIT(W25Q_CMD_RSECREG, tx_data, 5, p_buffer, num_byte_to_read);

    if (num_byte_to_read == 0x00)
    {
        return W25Qxx_E_INVALID;
    }

    ret = w25qxx_is_status(w25qxx, W25Qxx_STATUS_IDLE | W25Qxx_STATUS_SUSPEND, 0);
    if (ret != W25QXX_OK)
        return ret;

    /* Determine if the address > startAddr + W25Qxx_PAGESIZE */
    num_page = byte_addr >> W25Qxx_SECTORPOWER;
    start_addr = num_page * 0x00001000;
    if (num_page > 3 || num_page == 0 || byte_addr + num_byte_to_read > start_addr + W25Qxx_PAGESIZE)
    {
        return W25Qxx_E_BYTEADDRBOUND;
    }
#if W25QXX_4BADDR == 0
    transaction.tx_buffer = (void *)(tx_data + 1);
    transaction.tx_len--;
#endif

    return w25qxx_transfer(w25qxx, &transaction);
}

DEVICE_INTF_RET_TYPE w25qxx_read_sfdp(w25qxx_handle_t *w25qxx, uint8_t *p_buffer, uint32_t byte_addr, uint16_t num_byte_to_read)
{
    DEVICE_INTF_RET_TYPE ret = W25QXX_OK;
    uint8_t tx_data[4] = {0x00, 0x00, byte_addr & 0xFF, 0x00};
    w25qxx_transaction_t transaction = W25QXX_TRANSACTION_INIT(W25Q_CMD_RSFDP, tx_data, 4, p_buffer, num_byte_to_read);

    if (num_byte_to_read == 0x00)
    {
        return W25Qxx_E_INVALID;
    }

    ret = w25qxx_is_status(w25qxx, W25Qxx_STATUS_IDLE | W25Qxx_STATUS_SUSPEND, 0);
    if (ret != W25QXX_OK)
        return ret;

    /* Determine if the address > startAddr + W25Qxx_PAGESIZE */
    if (byte_addr + num_byte_to_read > W25Qxx_PAGESIZE)
    {
        return W25Qxx_E_BYTEADDRBOUND;
    }

    return w25qxx_transfer(w25qxx, &transaction);
}

/* No check Direct program Page   (0-256), Notes : no beyond page address */
/* No check Direct Page write
 * 1. Write data of the specified length at the specified address,
 *    but ensure that the data is in the same page.
 * 2. You must ensure that all data within the written address range is 0xFF,
 *    otherwise the data written at a location other than 0xFF will fail.
 **/
DEVICE_INTF_RET_TYPE w25qxx_dir_program_page(w25qxx_handle_t *w25qxx, uint8_t *p_buffer, uint32_t byte_addr, uint16_t num_byte_to_write)
{
    uint16_t rem_page;
    DEVICE_INTF_RET_TYPE ret = W25QXX_OK;
    uint8_t *tx_data = calloc(num_byte_to_write + 4, sizeof(uint8_t));
    w25qxx_transaction_t transaction = W25QXX_WRITE_TRANSACTION_INIT(W25Q_CMD_WPAGE, tx_data, num_byte_to_write + 4);

    pack_u32(tx_data, byte_addr);
    memcpy(tx_data + 4, p_buffer, num_byte_to_write);

    if (num_byte_to_write == 0x00)
    {
        return W25Qxx_E_INVALID;
    }

    ret = w25qxx_is_status(w25qxx, W25Qxx_STATUS_IDLE | W25Qxx_STATUS_SUSPEND, 0);
    if (ret != W25QXX_OK)
        return ret;

    /* Determine if the address > remainPage
     * (Notes : remainPage maxsize = 256) */
    rem_page = W25Qxx_PAGESIZE - (byte_addr & (W25Qxx_PAGESIZE - 1)); /* remPage = W25Qxx_PAGESIZE - ByteAddr % W25Qxx_PAGESIZE; */
    if (num_byte_to_write > rem_page)
    {
        return W25Qxx_E_BYTEADDRBOUND;
    }

#if W25QXX_4BADDR == 0
    transaction.tx_buffer = (void *)(tx_data + 1);
    transaction.tx_len--;
    /* Address > 0xFFFFFF */
    if (byte_addr > 0xFFFFFF)
    {
        ret = w25qxx_write_extended_register(w25qxx, (uint8_t)((byte_addr) >> 24));
    }
    else
    {
        ret = w25qxx_write_extended_register(w25qxx, 0x00);
    }
#endif

    if (ret != W25QXX_OK)
        return ret;

    ret = w25qxx_write_enable(w25qxx);
    ret = w25qxx_transfer(w25qxx, &transaction);
    ret = w25qxx_write_disable(w25qxx);
    w25qxx_u8_deinit(&tx_data);

    return w25qxx_is_status(w25qxx, W25Qxx_STATUS_IDLE | W25Qxx_STATUS_SUSPEND, w25qxx->info.progr_max_time_page);
}

/* No check Direct program */
/* No check Direct write
 * 1. With automatic page change function.
 * 2. You must ensure that all data within the written address range is 0xFF,
 *    otherwise the data written at a location other than 0xFF will fail.
 **/
DEVICE_INTF_RET_TYPE w25qxx_dir_program(w25qxx_handle_t *w25qxx, uint8_t *p_buffer, uint32_t byte_addr, uint32_t num_byte_to_write)
{
    uint16_t rem_page;
    DEVICE_INTF_RET_TYPE ret = W25QXX_OK;

    if (num_byte_to_write == 0x00)
    {
        return W25Qxx_E_INVALID;
    }

    rem_page = W25Qxx_PAGESIZE - (byte_addr & (W25Qxx_PAGESIZE - 1)); /* remPage = W25Qxx_PAGESIZE - ByteAddr % W25Qxx_PAGESIZE; */
    if (num_byte_to_write <= rem_page)
        rem_page = num_byte_to_write;

    while (num_byte_to_write > 0)
    {
        /*---------------------------------- Check Data Area ------------------------------------*/

        // No Check Data

        /*------------------------------------ Write data ---------------------------------------*/

        ret = w25qxx_dir_program_page(w25qxx, p_buffer, byte_addr, rem_page);
        if (ret != W25QXX_OK)
            return ret;

        /*------------------------------- Update next parameters --------------------------------*/

        /* Determine if writing is completed */
        p_buffer += rem_page;
        byte_addr += rem_page;
        num_byte_to_write -= rem_page;
        rem_page = (num_byte_to_write > W25Qxx_PAGESIZE) ? W25Qxx_PAGESIZE : num_byte_to_write;
    }

    return W25QXX_OK;
}

/* No check Direct program Security (0-256), Notes : no beyond page address */
/* Security Area : No check Direct Page write
 * 1. Write data of the specified length at the specified address,
 *    but ensure that the data is in the same page.
 * 2. You must ensure that all data within the written address range is 0xFF,
 *    otherwise the data written at a location other than 0xFF will fail.
 **/
DEVICE_INTF_RET_TYPE w25qxx_dir_program_security(w25qxx_handle_t *w25qxx, uint8_t *p_buffer, uint32_t byte_addr, uint16_t num_byte_to_write)
{
    uint16_t num_page = 0;
    uint32_t start_addr = 0;
    DEVICE_INTF_RET_TYPE ret = W25QXX_OK;
    uint8_t *tx_data = calloc(num_byte_to_write + 4, sizeof(uint8_t));
    w25qxx_transaction_t transaction = W25QXX_WRITE_TRANSACTION_INIT(W25Q_CMD_WSECREG, tx_data, num_byte_to_write + 4);

    tx_data[0] = 0x00;
    tx_data[1] = 0x00;
    tx_data[2] = (uint8_t)((byte_addr) >> 8);
    tx_data[3] = byte_addr & 0xFF;
    memcpy(tx_data + 4, p_buffer, num_byte_to_write);

    if (num_byte_to_write == 0x00)
    {
        return W25Qxx_E_INVALID;
    }

    ret = w25qxx_is_status(w25qxx, W25Qxx_STATUS_IDLE | W25Qxx_STATUS_SUSPEND, 0);
    if (ret != W25QXX_OK)
        return ret;

    /* Determine if the address > startAddr + W25Qxx_PAGESIZE */
    num_page = byte_addr >> W25Qxx_SECTORPOWER;
    start_addr = num_page * 0x00001000;
    if (num_page > 3 || num_page == 0 || byte_addr + num_byte_to_write > start_addr + W25Qxx_PAGESIZE)
    {
        return W25Qxx_E_BYTEADDRBOUND;
    }

#if W25QXX_4BADDR == 0
    transaction.tx_buffer = (void *)(tx_data + 1);
    transaction.tx_len--;
#endif

    ret = w25qxx_write_enable(w25qxx);
    ret = w25qxx_transfer(w25qxx, &transaction);
    ret = w25qxx_write_disable(w25qxx);
    w25qxx_u8_deinit(&tx_data);

    return w25qxx_is_status(w25qxx, W25Qxx_STATUS_IDLE | W25Qxx_STATUS_SUSPEND, w25qxx->info.progr_max_time_page);
}

DEVICE_INTF_RET_TYPE w25qxx_program(w25qxx_handle_t *w25qxx, uint8_t *p_buffer, uint32_t byte_addr, uint32_t num_byte_to_write)
{
    uint32_t num_sec = 0;
    uint16_t off_sec = 0;
    uint16_t rem_sec = 0;
    uint16_t i = 0;
    DEVICE_INTF_RET_TYPE ret = W25QXX_OK;

    if (num_byte_to_write == 0x00)
    {
        return W25Qxx_E_INVALID;
    }

    /* First Sector remain bytes */
    num_sec = byte_addr >> W25Qxx_SECTORPOWER;     /* calculate sector number address ( num_sec = byte_addr / W25Qxx_SECTORSIZE; ) */
    off_sec = byte_addr & (W25Qxx_SECTORSIZE - 1); /* calculate sector offset address ( off_sec = byte_addr % W25Qxx_SECTORSIZE; ) */
    rem_sec = W25Qxx_SECTORSIZE - off_sec;
    if (num_byte_to_write <= rem_sec)
        rem_sec = num_byte_to_write;

    while (num_byte_to_write > 0)
    {
        ret = w25qxx_read(w25qxx, W25QXX_CACHE, num_sec * W25Qxx_SECTORSIZE, W25Qxx_SECTORSIZE);
        if (ret != W25QXX_OK)
            return ret;

        /* Check whether the current sector data is 0xFF */
        for (i = 0; i < rem_sec; i++)
        {
            if (W25QXX_CACHE[off_sec + i] != 0xFF)
                break;
        }

        /*------------------------------------ Write data ---------------------------------------*/
        if (i < rem_sec) /* need to be erased */
        {
            ret = w25qxx_erase_sector(w25qxx, num_sec);
            if (ret != W25QXX_OK)
                return ret;

            /* copy data to buffer area */
            for (i = 0; i < rem_sec; i++)
            {
                W25QXX_CACHE[off_sec + i] = p_buffer[i];
            }

            /* Write the entire sector */
            ret = w25qxx_dir_program(w25qxx, W25QXX_CACHE, num_sec * W25Qxx_SECTORSIZE, W25Qxx_SECTORSIZE);
        }
        else
        {
            ret = w25qxx_dir_program(w25qxx, p_buffer, byte_addr, rem_sec);
        }
        if (ret != W25QXX_OK)
            return ret;

        num_sec++;   /* update sector address */
        off_sec = 0; /* reset  sector offset  */

        /*------------------------------- Update next parameters --------------------------------*/
        /* Determine if writing is completed */
        p_buffer += rem_sec;
        byte_addr += rem_sec;
        num_byte_to_write -= rem_sec;
        rem_sec = (num_byte_to_write > W25Qxx_SECTORSIZE) ? W25Qxx_SECTORSIZE : num_byte_to_write;
    }

    return W25QXX_OK;
}

/* Check Security program
 * Built in data erasure operation !!!
 * pBuffer        : Data storage area
 * WriteAddr      : Write address (24bit)
 * NumByteToWrite : Number of writes (max : 256)
 **/
DEVICE_INTF_RET_TYPE w25qxx_program_security(w25qxx_handle_t *w25qxx, uint8_t *p_buffer, uint32_t byte_addr, uint16_t num_byte_to_write)
{
    uint8_t num_page = 0;
    uint16_t off_page = 0;
    uint16_t rem_page = 0;
    uint16_t i = 0;
    DEVICE_INTF_RET_TYPE ret = W25QXX_OK;

    if (num_byte_to_write == 0x00)
    {
        return W25Qxx_E_INVALID;
    }

    /* First Sector remain bytes */
    num_page = byte_addr >> W25Qxx_SECTORPOWER;   /* calculate page number address ( numPage = ByteAddr / W25Qxx_SECTORPOWER; ) */
    off_page = byte_addr & (W25Qxx_PAGESIZE - 1); /* calculate page offset address ( offPage = ByteAddr % W25Qxx_PAGESIZE; ) */
    rem_page = W25Qxx_PAGESIZE - off_page;

    if (num_page > 3 || num_page == 0)
    {
        return W25Qxx_E_BYTEADDRBOUND;
    }

    if (num_byte_to_write <= rem_page)
        rem_page = num_byte_to_write;
    else
        return W25Qxx_E_BYTEADDRBOUND;

    /*---------------------------------- Check Data Area ------------------------------------*/
    ret = w25qxx_read_security(w25qxx, W25QXX_CACHE, num_page * 0x00001000, W25Qxx_PAGESIZE);
    if (ret != W25QXX_OK)
        return ret;

    for (i = 0; i < rem_page; i++)
    {
        if (W25QXX_CACHE[off_page + i] != 0xFF)
            break;
    }
    /*------------------------------------ Write data ---------------------------------------*/
    if (i < rem_page) /* need to be erased */
    {
        ret = w25qxx_erase_sector(w25qxx, num_page);
        if (ret != W25QXX_OK)
            return ret;

        /* copy data to buffer area */
        for (i = 0; i < rem_page; i++)
        {
            W25QXX_CACHE[off_page + i] = p_buffer[i];
        }

        /* Write the entire sector */
        ret = w25qxx_dir_program_security(w25qxx, W25QXX_CACHE, num_page * W25Qxx_SECTORSIZE, W25Qxx_PAGESIZE);
    }
    else
    {
        /* Ensure that the written data is in the same page, directly write the remaining section of the page */
        ret = w25qxx_dir_program_security(w25qxx, p_buffer, byte_addr, rem_page);
    }

    /*------------------------------- Update next parameters --------------------------------*/

    // No Check Data

    return ret;
}

DEVICE_INTF_RET_TYPE w25qxx_query_chip(w25qxx_handle_t *w25qxx)
{
    uint8_t numlist = 0;
    uint8_t i = 0;

    /* search for chip model */
    numlist = sizeof(w25q_info_list) / sizeof(w25qxx_info_t);
    for (i = 0; i < numlist; i++)
    {
        if (w25qxx->id_jedec == (w25q_info_list[i].type & w25qxx->id_jedec))
        {
            w25qxx->info = w25q_info_list[i];
            break;
        }
    }
    if (i == numlist)
    {
        return W25QXX_E_NOT_FOUND;
    }

    /* calculate Block/Sector/Page num */
    w25qxx->num_block = 1 << (BcdToByte(w25qxx->id_jedec & 0x000000FF) - 10);
    w25qxx->num_sector = w25qxx->num_block * 16;
    w25qxx->num_page = w25qxx->num_sector * 16;

    /* calculate Block/Sector/Page size */
    /* calculate block/sector/page size */
    w25qxx->size_page = 0x100;
    w25qxx->size_sector = w25qxx->size_page * 16;
    w25qxx->size_block = w25qxx->size_sector * 16;
    w25qxx->size_chip = w25qxx->size_sector * w25qxx->num_sector;

    return W25QXX_OK;
}

DEVICE_INTF_RET_TYPE w25qxx_config(w25qxx_handle_t *w25qxx)
{
    DEVICE_INTF_RET_TYPE ret = w25qxx_reset(w25qxx);
    DEVICE_INTF_RET_TYPE (*address_mode)
    (w25qxx_handle_t *w25qxx) = w25qxx_3_byte_mode;

#if W25QXX_4BADDR
    if (w25qxx->num_block >= 512) /* Block >= 512 have 4 address */
        address_mode = w25qxx_4_byte_mode;
#endif

    if (ret != W25QXX_OK)
    {
        return ret;
    }
#if W25QXX_SUPPORT_SFDP

    /* Support SFDP */

#else
    /* Read Unique ID */
    w25qxx_read_id_unique(w25qxx);

    /* Read Manufacturer ID */
    w25qxx_read_id_manufacturer(w25qxx);

    /* Read JEDEC ID */
    w25qxx_read_id_jedec(w25qxx);

    /* Query W25Qxx Type */
    ret = w25qxx_query_chip(w25qxx);
    if (ret != W25QXX_OK)
        return ret;

    /* Read Register */
    w25qxx_read_status_register(w25qxx, 1);
    w25qxx_read_status_register(w25qxx, 2);
    w25qxx_read_status_register(w25qxx, 3);

    /* Set address mode  */
    return address_mode(w25qxx);
#endif
}

/* W25Qxx Suspend/Resume Test */
DEVICE_INTF_RET_TYPE w25qxx_sus_resum_erase_sector(w25qxx_handle_t *w25qxx, uint32_t sector_addr)
{
    DEVICE_INTF_RET_TYPE ret = W25QXX_OK;
    uint8_t tx_data[4];
    w25qxx_transaction_t transaction = W25QXX_WRITE_TRANSACTION_INIT(W25Q_CMD_ESECTOR, tx_data, 4);

    pack_u32(tx_data, sector_addr);
    /* Determine if Sector Address Bound */
    if (sector_addr >= w25qxx->num_sector)
    {
        return W25Qxx_E_SECTORADDRBOUND;
    }

    /* Determine if it is busy */
    ret = w25qxx_is_status(w25qxx, W25Qxx_STATUS_BUSY, 0);
    if (ret != W25QXX_OK)
        return ret;

    /* Calculate sector address */
    sector_addr *= w25qxx->size_sector;

#if W25QXX_4BADDR == 0
    transaction.tx_buffer = (void *)(tx_data + 1);
    transaction.tx_len--;
    /* Address > 0xFFFFFF */
    if (sector_addr > 0xFFFFFF)
    {
        ret = w25qxx_write_extended_register(w25qxx, (uint8_t)((sector_addr) >> 24));
    }
    else
    {
        ret = w25qxx_write_extended_register(w25qxx, 0x00);
    }
#endif

    if (ret != W25QXX_OK)
        return ret;

    ret = w25qxx_write_enable(w25qxx);
    ret = w25qxx_transfer(w25qxx, &transaction);

    return w25qxx_write_disable(w25qxx);
}
