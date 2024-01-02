#ifndef _BASE_LORA_H_
#define _BASE_LORA_H_

#include <stdbool.h>
#include <stdint.h>

// Modem options
#define FSK_MODEM                               0x00        // GFSK packet type
#define LORA_MODEM                              0x01        // LoRa packet type

// RX gain options
#define LORA_RX_GAIN_POWER_SAVING               0x00        // gain used in Rx mode: power saving gain
#define LORA_RX_GAIN_BOOSTED                    0x01        //                       boosted gain

// Header type
#define LORA_HEADER_EXPLICIT                    0x00        // explicit header mode
#define LORA_HEADER_IMPLICIT                    0x01        // implicit header mode

// TX and RX operation mode
#define LORA_TX_SINGLE                          0x000000    // Tx timeout duration: no timeout (Rx single mode)
#define LORA_RX_SINGLE                          0x000000    // Rx timeout duration: no timeout (Rx single mode)
#define LORA_RX_CONTINUOUS                      0xFFFFFF    //                      infinite (Rx continuous mode)

// Status TX and RX operation
#define LORA_STATUS_DEFAULT                     0           // default status (false)
#define LORA_STATUS_TX_WAIT                     1
#define LORA_STATUS_TX_TIMEOUT                  2
#define LORA_STATUS_TX_DONE                     3
#define LORA_STATUS_RX_WAIT                     4
#define LORA_STATUS_RX_CONTINUOUS               5
#define LORA_STATUS_RX_TIMEOUT                  6
#define LORA_STATUS_RX_DONE                     7
#define LORA_STATUS_HEADER_ERR                  8
#define LORA_STATUS_CRC_ERR                     9
#define LORA_STATUS_CAD_WAIT                    10
#define LORA_STATUS_CAD_DETECTED                11
#define LORA_STATUS_CAD_DONE                    12

// Uncomment one of line below to use one or more LoRa model for a network library
#define USE_LORA_SX126X
#define USE_LORA_SX127X

typedef void (*begin_packet_fptr_t)();
typedef bool (*end_packet_fptr_t)(uint32_t timeout);
typedef void (*write_byte_fptr_t)(uint8_t data);
typedef void (*write_buffer_fptr_t)(uint8_t *data, uint8_t len);

typedef bool (*request_fptr_t)(uint32_t timeout);
typedef uint8_t (*available_fptr_t)();
typedef uint8_t (*read_byte_fptr_t)();
typedef uint8_t (*read_buffer_fptr_t)(uint8_t* data, uint8_t length);

typedef bool (*wait_fptr_t)(uint32_t timeout);
typedef uint8_t (*status_fptr_t)();

typedef struct base_lora base_lora_t;

struct base_lora
{
    begin_packet_fptr_t begin_packet;
    end_packet_fptr_t end_packet;
    write_byte_fptr_t write_byte;
    write_buffer_fptr_t write_buffer;

    request_fptr_t request;
    read_byte_fptr_t read_byte;
    read_buffer_fptr_t read_buffer;

    wait_fptr_t wait;
    status_fptr_t status;
};

#endif
