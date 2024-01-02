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
 * @brief   W25Qxx family driver header file
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
#ifndef __W25Q16_H__
#define __W25Q16_H__

#include <stdint.h>
#include <stddef.h>
#include "device.h"

#ifndef true
#define true    (!!1)
#endif

#ifndef false
#define false    (!!0)
#endif

#define WRITE_ENABLE 0x06
#define WRITE_DISABLE 0x04
#define PAGE_PROGRAM 0x02
#define READ_STATUS_REGISTER_1 0x05
#define READ_DATA 0x03
#define CHIP_ERASE 0xC7
#define POWER_DOWN 0xB9
#define RELEASE_POWER_DOWN 0xAB
#define MANUFACTURER_ID 0x90

#ifdef W25QXX_GLOBALS
#define W25QXX_EXT
#else
#define W25QXX_EXT extern
#endif

/**
 * @brief W25Q and W25X Series Drive (The W25Q family is a "superset" of the W25X family)
 *
 * 				Number of supported device mounts (1)
 *
 * 				Standard SPI  (��)      3ByteAddress (��)
 * 				Dual     SPI  (x)      4ByteAddress (��)
 * 				Quad     SPI  (x)
 * Note: 
 * 1. 3 address mode and 4 address mode are not supported to switch between each 
 *    other when the program is running, this version uses the macro to select the 
 *    address mode.
 * 2. In the 3-address mode, the data with address over 0xFFFFFF will be accessed 
 *    by pre-setting the extended registers by default.
 *
 */
#define W25QXX_FASTREAD    							 0		/* 0 : No Fast Read Mode   ; 1 : Fast Read Mode */
#define W25QXX_4BADDR      							 0		/* 0 : 3 Byte Address Mode ; 1 : 4 Byte Address Mode */
#define W25QXX_SUPPORT_SFDP							 0		/* 0 : No support SFDP     ; 1 : Support SFDP */

/**
 * @brief W25Qxx CMD
 */
#define W25Q_CMD_WEN		             			 0x06
#define W25Q_CMD_VOLATILESREN        				 0x50
#define W25Q_CMD_WDEN   		         			 0x04				 
#define W25Q_CMD_POWEREN    	       				 0xAB
#define W25Q_CMD_MANUFACTURER						 0x90
#define W25Q_CMD_JEDECID		         			 0x9F 
#define W25Q_CMD_UNIQUEID            				 0x4B				 
#define W25Q_CMD_READ			           			 0x03
#define W25Q_CMD_4BREAD              				 0x13
#define W25Q_CMD_FASTREAD            				 0x0B
#define W25Q_CMD_4BFASTREAD          				 0x0C				 
#define W25Q_CMD_WPAGE  		         			 0x02
#define W25Q_CMD_4BWPAGE             				 0x12				 
#define W25Q_CMD_ESECTOR		   	 		 		 0x20
#define W25Q_CMD_4BESECTOR		 	 		 		 0x21
#define W25Q_CMD_E32KBLOCK			 		 		 0x52
#define W25Q_CMD_E64KBLOCK			 		 		 0xD8
#define W25Q_CMD_4BE64KBLOCK		 		 		 0xDC
#define W25Q_CMD_ECHIP			     		 		 0xC7   /* or 0x60 */
#define W25Q_CMD_RSREG1		           				 0x05
#define W25Q_CMD_WSREG1              				 0x01
#define W25Q_CMD_RSREG2		           				 0x35
#define W25Q_CMD_WSREG2              				 0x31
#define W25Q_CMD_RSREG3	             				 0x15
#define W25Q_CMD_WSREG3              				 0x11
#define W25Q_CMD_REXTREG    	       				 0xC8
#define W25Q_CMD_WEXTREG 		         			 0xC5
#define W25Q_CMD_RSFDP               				 0x5A
#define W25Q_CMD_ESECREG         		 			 0x44
#define W25Q_CMD_WSECREG             				 0x42
#define W25Q_CMD_RSECREG             				 0x48
#define W25Q_CMD_WALLBLOCKLOCK    	 				 0x7E
#define W25Q_CMD_WALLBLOCKUNLOCK  	 				 0x98
#define W25Q_CMD_RBLOCKLOCK          				 0x3D
#define W25Q_CMD_WSIGBLOCKLOCK       				 0x36
#define W25Q_CMD_WSIGBLOCKUNLOCK     				 0x39	 
#define W25Q_CMD_EWSUSPEND           				 0x75
#define W25Q_CMD_EWRESUME            				 0x7A
#define W25Q_CMD_POWERDEN			       			 0xB9 	 
#define W25Q_CMD_4ByteAddrEN         				 0xB7
#define W25Q_CMD_4ByteAddrDEN        				 0xE9
#define W25Q_CMD_ENRESET             				 0x66
#define W25Q_CMD_RESETDEV            				 0x99
#define W25Q_DUMMY                   				 0xA5

/**
 * @brief W25Qxx SIZE
 */
#define W25Qxx_PAGESIZE							 	 0x00100
#define W25Qxx_PAGEPOWER							 0x08
#define W25Qxx_SECTORSIZE						 	 0x01000
#define W25Qxx_SECTORPOWER							 0x0C
#define W25Qxx_BLOCKSIZE						 	 0x10000
#define W25Qxx_BLOCKPOWER							 0x10
#define W25Qxx_SECTURITYSIZE						 0x00300

/**
 * @brief W25Qxx Address Convert
 */
#define W25Qxx_PAGEADDR(ByteAddr)					 (uint32_t)(ByteAddr >> W25Qxx_PAGEPOWER)				/* Convert Byte address to page address */
#define W25Qxx_SECTORADDR(ByteAddr) 				 (uint32_t)(ByteAddr >> W25Qxx_SECTORPOWER)				/* Convert Byte address to sector address */
#define W25Qxx_BLOCK64ADDR(ByteAddr) 				 (uint32_t)(ByteAddr >> W25Qxx_BLOCKPOWER)				/* Convert Byte address to block64 address */
#define W25Qxx_BLOCK32ADDR(ByteAddr) 				 (uint32_t)(ByteAddr >> (W25Qxx_BLOCKPOWER - 1))		/* Convert Byte address to block32 address */

#define W25QXX_TRANSACTION_INIT(command, tx_data, tlen, rx_data, rlen) \
    {                                 \
        .cmd = command,                   \
        .tx_buffer = tx_data,            \
        .tx_len = tlen,                  \
        .rx_buffer = rx_data,            \
        .rx_len = rlen,                  \
    }

#define W25QXX_CMD_TRANSACTION_INIT(cmd) W25QXX_TRANSACTION_INIT(cmd, NULL, 0, NULL, 0)

#define W25QXX_READ_TRANSACTION_INIT(cmd, rx_buffer, rx_len) W25QXX_TRANSACTION_INIT(cmd, NULL, 0, rx_buffer, rx_len)

#define W25QXX_WRITE_TRANSACTION_INIT(cmd, tx_buffer, tx_len) W25QXX_TRANSACTION_INIT(cmd, tx_buffer, tx_len, NULL, 0)

typedef struct w25qxx_transaction w25qxx_transaction_t;
typedef struct w25qxx_handle_s w25qxx_handle_t;
typedef struct chip_info_s w25qxx_info_t;
typedef struct w25qxx_sr_s w25qxx_sr_t;
typedef enum w25qxx_chip w25qxx_chip_t;
typedef enum w25qxx_srm w25qxx_srm_t;
typedef enum w25qxx_ret w25qxx_ret_t;
typedef enum w25qxx_status w25qxx_status_t;

struct w25qxx_transaction
{
    uint16_t cmd;
    size_t tx_len;
    size_t rx_len;
    void *tx_buffer;      ///< Pointer to transmit buffer, or NULL for no MOSI phase
    void *rx_buffer;            ///< Pointer to receive buffer, or NULL for no MISO phase. Written by 4 bytes-unit if DMA is used.
};

struct w25qxx_sr_s
{
    uint8_t *sr;
    uint8_t sr_cmd;
};

/**
 * @brief W25Qxx Chip Type
 */
enum w25qxx_chip
{
    UNKNOWN = 0x000000,
    W25X05 = 0xEFFF10,								 /* W25X05 */
    W25X10 = 0xEFFF11,								 /* W25X10 */
    W25Q20 = 0xEFFF12,								 /* W25Q20 */
    W25Q40 = 0xEFFF13,								 /* W25Q40 */
    W25Q80 = 0xEFFF14,								 /* W25Q80 */
    W25Q16 = 0xEFFF15,								 /* W25Q16 */
    W25Q32 = 0xEFFF16,								 /* W25Q32 */
    W25Q64 = 0xEFFF17,								 /* W25Q64 */
    W25Q128 = 0xEFFF18,								 /* W25Q128 ( 0xEF4018 0xEF7018 ) */
    W25Q256 = 0xEFFF19,								 /* W25Q256 */
    W25Q512 = 0xEFFF20,								 /* W25Q512 */
    W25Q01 = 0xEFFF21,								 /* W25Q01 */
    W25Q02 = 0xEFFF22								 /* W25Q02 */
};

/**                                                  
 * @brief W25Qxx Chip Parameter                      
 */
struct chip_info_s
{
    w25qxx_chip_t type;               /* Chip type */
    const char* name;                 /* Chip name */
    uint32_t progr_max_time_page;     /* Program max time (ms) */
    uint32_t erase_max_time_sector;   /* Erase Sector max time (ms) */
    uint32_t erase_max_time_block64;  /* Erase Block64 max time (ms) */
    uint32_t erase_max_time_block32;  /* Erase Block32 max time (ms) */
    uint32_t erase_max_time_chip;     /* Erase Chip max time (ms) */
};

struct w25qxx_handle_s
{
    device_t *dev;
    w25qxx_info_t info;               /* Chip Parameter */
    uint16_t id_manufacturer;         /* Manufacturer ID */
    uint32_t id_jedec;                /* JEDEC ID */
    uint64_t id_unique;               /* Unique ID */
    uint32_t num_block;               /* Block number */
    uint32_t num_page;                /* Page number */
    uint32_t num_sector;              /* Sector number */
    uint16_t size_page;               /* Page size (Byte) */
    uint32_t size_sector;             /* Sector size (Byte) */
    uint32_t size_block;              /* Block size (Byte) */
    uint32_t size_chip;               /* Chip size (Byte) */
    uint8_t status_register1;         /* StatusRegister 1 */
    uint8_t status_register2;         /* StatusRegister 2 */
    uint8_t status_register3;         /* StatusRegister 3 */
    uint8_t extended_register;        /* ExtendedRegister */
};

/**
 * @brief W25Qxx Error Information
 */
enum w25qxx_ret
{
    DEVICE_COMMON_RET(W25QXX),
    RET_ERROR(W25Qxx, STATUS),				         /* Device is suspended */
    RET_ERROR(W25Qxx, LOCK),			                 /* Device is locked */
    RET_ERROR(W25Qxx, INVALID),				         /* Invalid value entered */
    RET_ERROR(W25Qxx, BYTEADDRBOUND),		         /* Byte address out of bounds */
    RET_ERROR(W25Qxx, PAGEADDRBOUND),		         /* Page address out of bounds */
    RET_ERROR(W25Qxx, SECTORADDRBOUND),		         /* Sector address out of bounds */
    RET_ERROR(W25Qxx, BLOCK64ADDRBOUND),		     /* Block64 address out of bounds */
    RET_ERROR(W25Qxx, BLOCK32ADDRBOUND),		     /* Block32 address out of bounds */
    RET_ERROR(W25Qxx, WPSMODE),				         /* Write protect mode is error */
    RET_ERROR(W25Qxx, HARDWARE)				         /* SPI/QSPI BUS is hardware error */
};

/**
 * @brief W25Qxx Status Information
 */
enum w25qxx_status
{
    W25Qxx_STATUS_IDLE = 0x01,
    W25Qxx_STATUS_BUSY = 0x02,
    W25Qxx_STATUS_SUSPEND = 0x04,
    W25Qxx_STATUS_BUSY_AND_SUSPEND = 0x08
};

/**
 * @brief W25Qxx StatusRegister Storage Mode
 */
enum w25qxx_srm
{
    W25Qxx_VOLATILE = 0x00,							 /* StatusRegister Voliate Mode */
    W25Qxx_NON_VOLATILE = 0x01 						 /* StatusRegister Non-Voliate Mode */
};

DEVICE_INTF_RET_TYPE w25qxx_null_ptr_check(w25qxx_handle_t *w25qxx);
DEVICE_INTF_RET_TYPE w25qxx_interface_init(w25qxx_handle_t *w25qxx, device_t *dev);
DEVICE_INTF_RET_TYPE w25qxx_read_id_manufacturer(w25qxx_handle_t *w25qxx);
DEVICE_INTF_RET_TYPE w25qxx_read_id_jedec(w25qxx_handle_t *w25qxx);
DEVICE_INTF_RET_TYPE w25qxx_read_id_unique(w25qxx_handle_t *w25qxx);

/**
 * @brief W25Qxx Individual Control Instruction
 */
DEVICE_INTF_RET_TYPE w25qxx_reset(w25qxx_handle_t *w25qxx);
DEVICE_INTF_RET_TYPE w25qxx_power_enable(w25qxx_handle_t *w25qxx);
DEVICE_INTF_RET_TYPE w25qxx_power_disable(w25qxx_handle_t *w25qxx);
DEVICE_INTF_RET_TYPE w25qxx_volatile_sr_write_enable(w25qxx_handle_t *w25qxx);
DEVICE_INTF_RET_TYPE w25qxx_write_enable(w25qxx_handle_t *w25qxx);
DEVICE_INTF_RET_TYPE w25qxx_write_disable(w25qxx_handle_t *w25qxx);
DEVICE_INTF_RET_TYPE w25qxx_4_byte_mode(w25qxx_handle_t *w25qxx);
DEVICE_INTF_RET_TYPE w25qxx_3_byte_mode(w25qxx_handle_t *w25qxx);
DEVICE_INTF_RET_TYPE w25qxx_suspend(w25qxx_handle_t *w25qxx);
DEVICE_INTF_RET_TYPE w25qxx_resume(w25qxx_handle_t *w25qxx);

/**
 * @brief W25Qxx Read sector/block lock of the current address status
 */
DEVICE_INTF_RET_TYPE w25qxx_read_lock(w25qxx_handle_t *w25qxx, uint32_t byte_addr, uint8_t *status);

/**
 * @brief W25Qxx Read/Write ExtendedRegister
 */
DEVICE_INTF_RET_TYPE w25qxx_read_extended_register(w25qxx_handle_t *w25qxx);
DEVICE_INTF_RET_TYPE w25qxx_write_extended_register(w25qxx_handle_t *w25qxx, uint8_t extended_addr);

/**
 * @brief W25Qxx Read/Write StatusRegister
 */
DEVICE_INTF_RET_TYPE w25qxx_read_status_register(w25qxx_handle_t *w25qxx, uint8_t select_sr_1_2_3);
DEVICE_INTF_RET_TYPE w25qxx_write_status_register(w25qxx_handle_t *w25qxx, uint8_t select_sr_1_2_3, uint8_t data);
DEVICE_INTF_RET_TYPE w25qxx_volatile_sr_write_status_register(w25qxx_handle_t *w25qxx, uint8_t select_sr_1_2_3, uint8_t data);
void w25qxx_set_factory_write_status_register(w25qxx_handle_t *w25qxx);
DEVICE_INTF_RET_TYPE w25qxx_rbit_wel(w25qxx_handle_t *w25qxx, uint8_t *wel);
DEVICE_INTF_RET_TYPE w25qxx_rbit_busy(w25qxx_handle_t *w25qxx, uint8_t *busy);
DEVICE_INTF_RET_TYPE w25qxx_rbit_sus(w25qxx_handle_t *w25qxx, uint8_t *sus);
DEVICE_INTF_RET_TYPE w25qxx_rbit_ads(w25qxx_handle_t *w25qxx, uint8_t *ads);
DEVICE_INTF_RET_TYPE w25qxx_wbit_srp(w25qxx_handle_t *w25qxx, w25qxx_srm_t srm, uint8_t bit);
DEVICE_INTF_RET_TYPE w25qxx_wbit_tb(w25qxx_handle_t *w25qxx, w25qxx_srm_t srm, uint8_t bit);
DEVICE_INTF_RET_TYPE w25qxx_wbit_cmp(w25qxx_handle_t *w25qxx, w25qxx_srm_t srm, uint8_t bit);
DEVICE_INTF_RET_TYPE w25qxx_wbit_qe(w25qxx_handle_t *w25qxx, w25qxx_srm_t srm, uint8_t bit);
DEVICE_INTF_RET_TYPE w25qxx_wbit_srl(w25qxx_handle_t *w25qxx, w25qxx_srm_t srm, uint8_t bit);
DEVICE_INTF_RET_TYPE w25qxx_wbit_wps(w25qxx_handle_t *w25qxx, w25qxx_srm_t srm, uint8_t bit);
DEVICE_INTF_RET_TYPE w25qxx_wbit_drv(w25qxx_handle_t *w25qxx, w25qxx_srm_t srm, uint8_t bit);
DEVICE_INTF_RET_TYPE w25qxx_wbit_bp(w25qxx_handle_t *w25qxx, w25qxx_srm_t srm, uint8_t bit);
DEVICE_INTF_RET_TYPE w25qxx_wbit_lb(w25qxx_handle_t *w25qxx, w25qxx_srm_t srm, uint8_t bit);
DEVICE_INTF_RET_TYPE w25qxx_wbit_adp(w25qxx_handle_t *w25qxx, uint8_t bit);
DEVICE_INTF_RET_TYPE w25qxx_read_status(w25qxx_handle_t *w25qxx, uint8_t *status);

/**
 * @brief W25Qxx read current status
 */
DEVICE_INTF_RET_TYPE w25qxx_is_status(w25qxx_handle_t *w25qxx, uint8_t select_status, uint32_t timeout);

/**
 * @brief W25Qxx Sector/Block Lock protect for "WPS = 1"
 */
DEVICE_INTF_RET_TYPE w25qxx_global_unlock(w25qxx_handle_t *w25qxx);
DEVICE_INTF_RET_TYPE w25qxx_global_locked(w25qxx_handle_t *w25qxx);
DEVICE_INTF_RET_TYPE w25qxx_individual_unlock(w25qxx_handle_t *w25qxx, uint32_t byte_addr);
DEVICE_INTF_RET_TYPE w25qxx_individual_locked(w25qxx_handle_t *w25qxx, uint32_t byte_addr);

/**
 * @brief W25Qxx Main/SFDP/Security Storage read/program/erase function
 */
DEVICE_INTF_RET_TYPE w25qxx_erase_chip(w25qxx_handle_t *w25qxx);
DEVICE_INTF_RET_TYPE w25qxx_erase_block64(w25qxx_handle_t *w25qxx, uint32_t block64_addr);
DEVICE_INTF_RET_TYPE w25qxx_erase_block32(w25qxx_handle_t *w25qxx, uint32_t block32_addr);
DEVICE_INTF_RET_TYPE w25qxx_erase_sector(w25qxx_handle_t *w25qxx, uint32_t sector_addr);
DEVICE_INTF_RET_TYPE w25qxx_erase_security(w25qxx_handle_t *w25qxx, uint32_t sector_addr);
DEVICE_INTF_RET_TYPE w25qxx_read(w25qxx_handle_t *w25qxx, uint8_t *p_buffer, uint32_t byte_addr, uint16_t num_byte_to_read);
DEVICE_INTF_RET_TYPE w25qxx_read_security(w25qxx_handle_t *w25qxx, uint8_t *p_buffer, uint32_t byte_addr, uint16_t num_byte_to_read);
DEVICE_INTF_RET_TYPE w25qxx_read_sfdp(w25qxx_handle_t *w25qxx, uint8_t *p_buffer, uint32_t byte_addr, uint16_t num_byte_to_read);
DEVICE_INTF_RET_TYPE w25qxx_dir_program_page(w25qxx_handle_t *w25qxx, uint8_t *p_buffer, uint32_t byte_addr, uint16_t num_byte_to_write);
DEVICE_INTF_RET_TYPE w25qxx_dir_program(w25qxx_handle_t *w25qxx, uint8_t *p_buffer, uint32_t byte_addr, uint32_t num_byte_to_write);
DEVICE_INTF_RET_TYPE w25qxx_dir_program_security(w25qxx_handle_t *w25qxx, uint8_t *p_buffer, uint32_t byte_addr, uint16_t num_byte_to_write);
DEVICE_INTF_RET_TYPE w25qxx_program(w25qxx_handle_t *w25qxx, uint8_t *p_buffer, uint32_t byte_addr, uint32_t num_byte_to_write);
DEVICE_INTF_RET_TYPE w25qxx_program_security(w25qxx_handle_t *w25qxx, uint8_t *p_buffer, uint32_t byte_addr, uint16_t num_byte_to_write);

/**
 * @brief W25Qxx config function
 */
DEVICE_INTF_RET_TYPE w25qxx_query_chip(w25qxx_handle_t *w25qxx);
DEVICE_INTF_RET_TYPE w25qxx_config(w25qxx_handle_t *w25qxx);
DEVICE_INTF_RET_TYPE w25qxx_sus_resum_erase_sector(w25qxx_handle_t *w25qxx, uint32_t sector_addr);
#endif
