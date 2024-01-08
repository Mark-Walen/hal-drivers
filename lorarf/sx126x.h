#ifndef _SX126X_H_
#define _SX126X_H_

#include "base_lora.h"
#include "sx126x_driver.h"

// Status TX and RX operation
#define SX126X_STATUS_DEFAULT                   LORA_STATUS_DEFAULT
#define SX126X_STATUS_TX_WAIT                   LORA_STATUS_TX_WAIT
#define SX126X_STATUS_TX_TIMEOUT                LORA_STATUS_TX_TIMEOUT
#define SX126X_STATUS_TX_DONE                   LORA_STATUS_TX_DONE
#define SX126X_STATUS_RX_WAIT                   LORA_STATUS_RX_WAIT
#define SX126X_STATUS_RX_CONTINUOUS             LORA_STATUS_RX_CONTINUOUS
#define SX126X_STATUS_RX_TIMEOUT                LORA_STATUS_RX_TIMEOUT
#define SX126X_STATUS_RX_DONE                   LORA_STATUS_RX_DONE
#define SX126X_STATUS_HEADER_ERR                LORA_STATUS_HEADER_ERR
#define SX126X_STATUS_CRC_ERR                   LORA_STATUS_CRC_ERR
#define SX126X_STATUS_CAD_WAIT                  LORA_STATUS_CAD_WAIT
#define SX126X_STATUS_CAD_DETECTED              LORA_STATUS_CAD_DETECTED
#define SX126X_STATUS_CAD_DONE                  LORA_STATUS_CAD_DONE

// Default Hardware Configuration
#define SX126X_PIN_RF_IRQ                             1

#ifdef ESP8266
        typedef void ICACHE_RAM_ATTR (*_interruptTx_fptr_t)();
        typedef void ICACHE_RAM_ATTR (*_interruptRx_fptr_t)();
        typedef void ICACHE_RAM_ATTR (*_interruptRxContinuous__fptr_t)();
#else
        typedef void (*_interruptTx_fptr_t)();
        typedef void (*_interruptRx_fptr_t)();
        typedef void (*_interruptRxContinuous__fptr_t)();
#endif
typedef void (*_onTransmit_fptr_t)();
typedef void (*_onReceive_fptr_t)();

typedef struct sx126x sx126x_t;

struct sx126x
{
    #if defined(USE_LORA_SX126X) && defined(USE_LORA_SX127X)
    base_lora_t *base_lora_p;
    #endif

    // sx126x_driver
    sx126x_driver_t *sx126x_drv;

    device_t *_reset, *_irq, *_txen, *_rxen, *_pinToLow;
    // private variables
    int8_t _dio;
    uint8_t _statusWait;
    volatile uint16_t _statusIrq;
    uint32_t _transmitTime;
    uint8_t _bufferIndex;
    uint8_t _payloadTxRx;
    int8_t _irqStatic;
    uint16_t _random;
    _interruptRx_fptr_t _interruptRx;
    _interruptTx_fptr_t _interruptTx;
    _interruptRxContinuous__fptr_t _interruptRxContinuous;

    // protected variables
    uint8_t _modem;
    uint8_t _sf;
    uint32_t _bw;
    uint8_t _cr;
    bool _ldro;
    uint8_t _headerType;
    uint16_t _preambleLength;
    uint8_t _payloadLength;
    bool _crcType;
    bool _invertIq;
    _onTransmit_fptr_t _onTransmit;
    _onReceive_fptr_t _onReceive;
};

/**
 * @brief Initialize LoRa device
 * 
 * @param[in,out] lora Device need to be connect and set.
 * @param[in] lora_gpio_config Config irq txen and rxen pins. If all of these pin are set to -1, NULL can be pass.
 * @param[in] sx126x_begin Config chip select pin and busy pin. Begin spi.
 * @param[in] sx126x_reset Soft reset modem.
 * 
 * @return If lora device is not found or set modem to lora device failed.
*/
SX126X_RET_TYPE sx126x_null_ptr_check(sx126x_t *lora);
SX126X_RET_TYPE begin(sx126x_t *lora);
SX126X_RET_TYPE begin_set_pins(sx126x_t *lora, device_t *spidev, device_t *reset, device_t *busy, device_t *irq, device_t *txen, device_t *rxen);
void end(sx126x_t lora, close_spi_fptr_t close_spi);
bool reset(sx126x_t lora);
void sleep(sx126x_t lora, uint8_t option);
SX126X_RET_TYPE wake(sx126x_t lora);
void standby(sx126x_t lora, uint8_t option);
bool busyCheck(sx126x_t lora, uint32_t timeout);
void setFallbackMode(sx126x_t lora, uint8_t fallbackMode);
uint8_t getMode(sx126x_t lora);

// Hardware configuration methods
void set_pins(sx126x_t *lora, device_t *spidev, device_t *reset,
              device_t *busy, device_t *irq,
              device_t *txen, device_t *rxen);
void setRfIrqPin(sx126x_t lora, int8_t dioPinSelect);
void setDio2RfSwitch(sx126x_t lora, bool enable);
void setDio3TcxoCtrl(sx126x_t lora, uint8_t tcxoVoltage, uint32_t delayTime);
void setXtalCap(sx126x_t lora, uint8_t xtalA, uint8_t xtalB);
void setRegulator(sx126x_t lora, uint8_t regMode);
void setCurrentProtection(sx126x_t lora, uint8_t current);

// Modem, modulation parameter, and packet parameter setup methods
uint8_t getModem(sx126x_t lora);
void setModem(sx126x_t lora, uint8_t modem);
void setFrequency(sx126x_t lora, uint32_t frequency);
void setTxPower(sx126x_t lora, uint8_t txPower, uint8_t version);
void setRxGain(sx126x_t lora, uint8_t boost);
void setLoRaModulation(sx126x_t lora, uint8_t sf, uint32_t bw, uint8_t cr, bool ldro);
void setLoRaPacket(sx126x_t lora, uint8_t headerType, uint16_t preambleLength, uint8_t payloadLength, bool crcType, bool invertIq);
void setSpreadingFactor(sx126x_t lora, uint8_t sf);
void setBandwidth(sx126x_t lora, uint32_t bw);
void setCodeRate(sx126x_t lora, uint8_t cr);
void setLdroEnable(sx126x_t lora, bool ldro);
void setHeaderType(sx126x_t lora, uint8_t headerType);
void setPreambleLength(sx126x_t lora, uint16_t preambleLength);
void setPayloadLength(sx126x_t lora, uint8_t payloadLength);
void setCrcEnable(sx126x_t lora, bool crcType);
void setInvertIq(sx126x_t lora, bool invertIq);
void setSyncWord(sx126x_t lora, uint16_t syncWord);
void setFskModulation(sx126x_t lora, uint32_t br, uint8_t pulseShape, uint8_t bandwidth, uint32_t Fdev);
void setFskPacket(sx126x_t lora, uint16_t preambleLength, uint8_t preambleDetector, uint8_t syncWordLength, uint8_t addrComp, uint8_t packetType, uint8_t payloadLength, uint8_t crcType, uint8_t whitening);
void setFskSyncWord(sx126x_t lora, uint8_t* sw, uint8_t swLen);
void setFskAdress(sx126x_t lora, uint8_t nodeAddr, uint8_t broadcastAddr);
void setFskCrc(sx126x_t lora, uint16_t crcInit, uint16_t crcPolynom);
void setFskWhitening(sx126x_t lora, uint16_t whitening);

// Transmit related methods
void beginPacket(sx126x_t lora);
bool endPacket(sx126x_t lora, uint32_t timeout);
void write_byte(sx126x_t lora, uint8_t data);
void write_ubuffer(sx126x_t lora, uint8_t* data, uint8_t length);
void write_sbuffer(sx126x_t lora, char* data, uint8_t length);


// Common Operational methods
void put(sx126x_t lora, void *data, const uint8_t length);
void onTransmit(sx126x_t lora, void(*callback)());

// Receive related methods
bool request(sx126x_t lora, uint32_t timeout);
bool listen(sx126x_t lora, uint32_t rxPeriod, uint32_t sleepPeriod);
uint8_t available(sx126x_t lora);
uint8_t read_byte(sx126x_t lora);
uint8_t read_ubuffer(sx126x_t lora, uint8_t* data, uint8_t length);
uint8_t read_sbuffer(sx126x_t lora, char* data, uint8_t length);
void purge(sx126x_t lora, uint8_t length);
uint8_t get(sx126x_t lora, void *data, const uint8_t length);
void onReceive(sx126x_t lora, void(*callback)());

// Wait, operation status, and packet status methods
bool wait(sx126x_t lora, uint32_t timeout, void (*yield)());
uint8_t status(sx126x_t lora);
uint32_t transmitTime(sx126x_t lora);
float dataRate(sx126x_t lora);
int16_t packetRssi(sx126x_t lora);
float snr(sx126x_t lora);
int16_t signalRssi(sx126x_t lora);
int16_t rssiInst(sx126x_t lora);
uint16_t getError(sx126x_t lora);
uint32_t random(sx126x_t lora);

#endif