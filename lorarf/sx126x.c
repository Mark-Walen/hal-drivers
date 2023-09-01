#include "sx126x.h"
#include <stdlib.h>

static void _irqSetup(sx126x_t lora, uint16_t irqMask);

SX126X_RET_TYPE sx126x_init(sx126x_t *lora)
{
    if (lora == NULL) return SX126X_E_NULL_PTR;
    
    lora->_sf = 7;
    lora->_bw = 125000;
    lora->_cr = 4;
    lora->_headerType = SX126X_HEADER_EXPLICIT;
    lora->_preambleLength = 12;

    lora->_transmitTime = 0;
    lora->_bufferIndex = 0;
    lora->_payloadTxRx = 0;
    lora->_irqStatic = -1;
    lora->_pinToLow = -1;
    return SX126X_OK;
}

SX126X_RET_TYPE begin(sx126x_t *lora)
{
    sx126x_driver_t *sx126x_drv = lora->sx126x_drv;
    if (sx126x_drv == NULL)
    {
        return SX126X_E_NULL_PTR;
    }

    if (lora->_irq != NULL && sx126x_drv->gpio_config_input != NULL) {
        sx126x_drv->gpio_config_input(lora->_irq->port, lora->_irq->pin);
    }

    if (lora->_txen != NULL && sx126x_drv->gpio_config_input != NULL) {
        sx126x_drv->gpio_config_input(lora->_txen->port, lora->_txen->pin);
    }

    if (lora->_irq != NULL && sx126x_drv->gpio_config_input != NULL) {
        sx126x_drv->gpio_config_input(lora->_rxen->port, lora->_rxen->pin);
    }

    lora->_dio = SX126X_PIN_RF_IRQ;

    // begine spi and perform device reset
    if (sx126x_drv->sx126x_begin == NULL)
    {
        sx126x_drv->sx126x_driver->println("Please ensure nss pin, busy pin and spi are initialized Properly");
    }
    else
    {
        sx126x_drv->sx126x_begin();
    }

    sx126x_reset(sx126x_drv, lora->_reset);

    // check if device connect and set modem to LoRa
    sx126x_setStandby(sx126x_drv, SX126X_STANDBY_RC);
    if (getMode(*lora) != SX126X_STATUS_MODE_STDBY_RC)
        return SX126X_E_DEVICE_NOT_FOUND;
    sx126x_setPacketType(sx126x_drv, SX126X_LORA_MODEM);

    sx126x_fixResistanceAntenna(sx126x_drv);
    return SX126X_OK;
}

/**
 * @param[in] irq: default NULL
 * @param[in] txen: defaulat NULL
 * @param[in] rxen: default NULL
*/
SX126X_RET_TYPE begin_set_pins(sx126x_t *lora, device_gpio_typedef_t *nss, device_gpio_typedef_t *reset,
                               device_gpio_typedef_t *busy, device_gpio_typedef_t *irq, device_gpio_typedef_t *txen,
                               device_gpio_typedef_t *rxen)
{
    set_pins(lora, nss, reset, busy, irq, txen, rxen);
    return begin(lora);
}

void end(sx126x_t lora, close_spi_fptr_t close_spi)
{
    sleep(lora, SX126X_SLEEP_COLD_START);
}

bool reset(sx126x_t lora)
{
    sx126x_reset(lora.sx126x_drv, lora._reset);
    return sx126x_busyCheck(lora.sx126x_drv, SX126X_BUSY_TIMEOUT) == SX126X_OK;
}

/**
 * @param[in] option: default SX126X_SLEEP_WARM_START
*/
void sleep(sx126x_t lora, uint8_t option)
{
    uint8_t ret = SX126X_OK;
    standby(lora, SX126X_STANDBY_RC);
    sx126x_setSleep(lora.sx126x_drv, option);
    ret = null_ptr_check(lora.sx126x_drv->sx126x_driver);
    if (ret != SX126X_OK)
    {
        lora.sx126x_drv->sx126x_driver->delay_us(500, NULL);
    }    
}

SX126X_RET_TYPE wake(sx126x_t lora)
{
    sx126x_driver_t *sx126x_drv = lora.sx126x_drv;
    uint8_t ret = sx126x_drv_null_ptr_check(sx126x_drv);
    device_gpio_typedef_t *nss = (device_gpio_typedef_t *) sx126x_drv->sx126x_driver->addr;
    if (ret != SX126X_OK) return ret;
    
    sx126x_drv->gpio_set_pin(nss->port, nss->port);
    return SX126X_OK;
}

/**
 * @param[in] option: default SX126X_STANDBY_RC
*/
void standby(sx126x_t lora, uint8_t option)
{
    sx126x_setStandby(lora.sx126x_drv, option);
}

/**
 * @param[in] timeout: default SX126X_BUSY_TIMEOUT
*/
bool busyCheck(sx126x_t lora, uint32_t timeout)
{
    return sx126x_busyCheck(lora.sx126x_drv, timeout) == SX126X_OK;
}

void setFallbackMode(sx126x_t lora, uint8_t fallbackMode)
{
    sx126x_setRxTxFallbackMode(lora.sx126x_drv, fallbackMode);
}

uint8_t getMode(sx126x_t lora)
{
    uint8_t mode;
    sx126x_getStatus(lora.sx126x_drv, &mode);
    return mode & 0x70;
}

void set_pins(sx126x_t *lora, device_gpio_typedef_t *nss, device_gpio_typedef_t *reset,
              device_gpio_typedef_t *busy, device_gpio_typedef_t *irq,
              device_gpio_typedef_t *txen, device_gpio_typedef_t *rxen)
{
    sx126x_set_pins(lora->sx126x_drv, busy, nss);
    lora->_reset = reset;
    lora->_irq = irq;
    lora->_txen = txen;
    lora->_rxen = rxen;
    lora->_irqStatic = -1;
}

void setRfIrqPin(sx126x_t lora, int8_t dioPinSelect)
{
    if ((dioPinSelect == 2) || (dioPinSelect == 3)) lora._dio = dioPinSelect;
    else lora._dio = 1;
}

/**
 * @param[in] enable: default true;
*/
void setDio2RfSwitch(sx126x_t lora, bool enable)
{
    if (enable) sx126x_setDio2AsRfSwitchCtrl(lora.sx126x_drv, SX126X_DIO2_AS_RF_SWITCH);
    else sx126x_setDio2AsRfSwitchCtrl(lora.sx126x_drv, SX126X_DIO2_AS_IRQ);
}

void setDio3TcxoCtrl(sx126x_t lora, uint8_t tcxoVoltage, uint32_t delayTime)
{
    sx126x_setDio3AsTcxoCtrl(lora.sx126x_drv, tcxoVoltage, delayTime);
    sx126x_setStandby(lora.sx126x_drv, SX126X_STANDBY_RC);
    sx126x_calibrate(lora.sx126x_drv, 0xFF);
}

void setXtalCap(sx126x_t lora, uint8_t xtalA, uint8_t xtalB)
{
    sx126x_setStandby(lora.sx126x_drv, SX126X_STANDBY_XOSC);
    uint8_t buf[2] = {xtalA, xtalB};
    sx126x_writeRegister(lora.sx126x_drv, SX126X_REG_XTA_TRIM, buf, 2);
    sx126x_setStandby(lora.sx126x_drv, SX126X_STANDBY_RC);
    sx126x_calibrate(lora.sx126x_drv, 0xFF);
}

void setRegulator(sx126x_t lora, uint8_t regMode)
{
    sx126x_setRegulatorMode(lora.sx126x_drv, regMode);
}

void setCurrentProtection(sx126x_t lora, uint8_t current)
{
    uint8_t currentmA = current * 2 / 5;
    sx126x_writeRegister(lora.sx126x_drv, SX126X_REG_OCP_CONFIGURATION, &currentmA, 1);
}

// Modem, modulation parameter, and packet parameter setup methods
uint8_t getModem(sx126x_t lora)
{
    uint8_t modem;
    sx126x_getPacketType(lora.sx126x_drv, &modem);
    return modem;
}

/**
 * @param[in] modem: default SX126X_LORA_MODEM
*/
void setModem(sx126x_t lora, uint8_t modem)
{
    lora._modem = modem;
    sx126x_setStandby(lora.sx126x_drv, SX126X_STANDBY_RC);
    sx126x_setPacketType(lora.sx126x_drv, modem);
}

void setFrequency(sx126x_t lora, uint32_t frequency)
{
    uint8_t calFreq[2];
    if (frequency < 446000000) {        // 430 - 440 Mhz
        calFreq[0] = SX126X_CAL_IMG_430;
        calFreq[1] = SX126X_CAL_IMG_440;
    }
    else if (frequency < 734000000) {   // 470 - 510 Mhz
        calFreq[0] = SX126X_CAL_IMG_470;
        calFreq[1] = SX126X_CAL_IMG_510;
    }
    else if (frequency < 828000000) {   // 779 - 787 Mhz
        calFreq[0] = SX126X_CAL_IMG_779;
        calFreq[1] = SX126X_CAL_IMG_787;
    }
    else if (frequency < 877000000) {   // 863 - 870 Mhz
        calFreq[0] = SX126X_CAL_IMG_863;
        calFreq[1] = SX126X_CAL_IMG_870;
    }
    else if (frequency < 1100000000) {  // 902 - 928 Mhz
        calFreq[0] = SX126X_CAL_IMG_902;
        calFreq[1] = SX126X_CAL_IMG_928;
    }
    // calculate frequency for setting configuration
    uint32_t rfFreq = ((uint64_t) frequency << SX126X_RF_FREQUENCY_SHIFT) / SX126X_RF_FREQUENCY_XTAL;

    // perform image calibration before set frequency
    sx126x_calibrateImage(lora.sx126x_drv, calFreq[0], calFreq[1]);
    sx126x_setRfFrequency(lora.sx126x_drv, rfFreq);
}

/**
 * @param version: default SX126X_TX_POWER_SX1262
*/
void setTxPower(sx126x_t lora, uint8_t txPower, uint8_t version)
{
    // maximum TX power is 22 dBm and 15 dBm for SX1261
    if (txPower > 22) txPower = 22;
    else if (txPower > 15 && version == SX126X_TX_POWER_SX1261) txPower = 15;

    uint8_t paDutyCycle = 0x00;
    uint8_t hpMax = 0x00;
    uint8_t deviceSel = version == SX126X_TX_POWER_SX1261 ? 0x01 : 0x00;
    uint8_t power = 0x0E;
    // set parameters for PA config and TX params configuration
    if (txPower == 22) {
        paDutyCycle = 0x04;
        hpMax = 0x07;
        power = 0x16;
    } else if (txPower >= 20) {
        paDutyCycle = 0x03;
        hpMax = 0x05;
        power = 0x16;
    } else if (txPower >= 17) {
        paDutyCycle = 0x02;
        hpMax = 0x03;
        power = 0x16;
    } else if (txPower >= 14 && version == SX126X_TX_POWER_SX1261) {
        paDutyCycle = 0x04;
        hpMax = 0x00;
        power = 0x0E;
    } else if (txPower >= 14 && version == SX126X_TX_POWER_SX1262) {
        paDutyCycle = 0x02;
        hpMax = 0x02;
        power = 0x16;
    } else if (txPower >= 14 && version == SX126X_TX_POWER_SX1268) {
        paDutyCycle = 0x04;
        hpMax = 0x06;
        power = 0x0F;
    } else if (txPower >= 10 && version == SX126X_TX_POWER_SX1261) {
        paDutyCycle = 0x01;
        hpMax = 0x00;
        power = 0x0D;
    } else if (txPower >= 10 && version == SX126X_TX_POWER_SX1268) {
        paDutyCycle = 0x00;
        hpMax = 0x03;
        power = 0x0F;
    } else {
        return;
    }

    // set power amplifier and TX power configuration
    sx126x_setPaConfig(lora.sx126x_drv, paDutyCycle, hpMax, deviceSel, 0x01);
    sx126x_setTxParams(lora.sx126x_drv, power, SX126X_PA_RAMP_800U);
}

void setRxGain(sx126x_t lora, uint8_t boost)
{
    // set power saving or boosted gain in register
    uint8_t gain = boost ? SX126X_BOOSTED_GAIN : SX126X_POWER_SAVING_GAIN;
    sx126x_writeRegister(lora.sx126x_drv, SX126X_REG_RX_GAIN, &gain, 1);
    if (boost){
        // set certain register to retain configuration after wake from sleep mode
        uint8_t buf[3] = {0x01, 0x08, 0xAC};
        sx126x_writeRegister(lora.sx126x_drv, 0x029F, buf, 3);
    }
}

/**
 * @param ldro: default false
*/
void setLoRaModulation(sx126x_t lora, uint8_t sf, uint32_t bw, uint8_t cr, bool ldro)
{
    lora._sf = sf;
    lora._bw = bw;
    lora._cr = cr;
    lora._ldro = ldro;

    // valid spreading factor is between 5 and 12
    if (sf > 12) sf = 12;
    else if (sf < 5) sf = 5;
    // select bandwidth options
    if (bw < 9100) bw = SX126X_BW_7800;             // 7.8 kHz
    else if (bw < 13000) bw = SX126X_BW_10400;      // 10.4 kHz
    else if (bw < 18200) bw = SX126X_BW_15600;      // 15.6 kHz
    else if (bw < 26000) bw = SX126X_BW_20800;      // 20.8 kHz
    else if (bw < 36500) bw = SX126X_BW_31250;      // 31.25 kHz
    else if (bw < 52100) bw = SX126X_BW_41700;      // 41.7 kHz
    else if (bw < 93800) bw = SX126X_BW_62500;      // 62.5 kHz
    else if (bw < 187500) bw = SX126X_BW_125000;    // 125 kHz
    else if (bw < 375000) bw = SX126X_BW_250000;    // 250 kHz
    else bw = SX126X_BW_500000;                     // 500 kHz
    // valid code rate denominator is between 4 and 8
    cr -= 4;
    if (cr > 4) cr = 0;

    sx126x_setModulationParamsLoRa(lora.sx126x_drv, sf, (uint8_t) bw, cr, (uint8_t) ldro);
}

/**
 * @param crcType: default false
 * @param invertIq: default false
*/
void setLoRaPacket(sx126x_t lora, uint8_t headerType, uint16_t preambleLength, uint8_t payloadLength, bool crcType, bool invertIq)
{
    lora._headerType = headerType;
    lora._preambleLength = preambleLength;
    lora._payloadLength = payloadLength;
    lora._crcType = crcType;
    lora._invertIq = invertIq;

    // filter valid header type config
    if (headerType != SX126X_HEADER_IMPLICIT) headerType = SX126X_HEADER_EXPLICIT;

    sx126x_setPacketParamsLoRa(lora.sx126x_drv, preambleLength, headerType, payloadLength, (uint8_t) crcType, (uint8_t) invertIq);
    sx126x_fixInvertedIq(lora.sx126x_drv, (uint8_t) invertIq);
}

void setSpreadingFactor(sx126x_t lora, uint8_t sf)
{
    setLoRaModulation(lora, sf, lora._bw, lora._cr, lora._ldro);
}

void setBandwidth(sx126x_t lora, uint32_t bw)
{
    setLoRaModulation(lora, lora._sf, bw, lora._cr, lora._ldro);
}

void setCodeRate(sx126x_t lora, uint8_t cr)
{
    setLoRaModulation(lora, lora._sf, lora._bw, cr, lora._ldro);
}

/**
 * @param[in] ldro: default true
*/
void setLdroEnable(sx126x_t lora, bool ldro)
{
    setLoRaModulation(lora, lora._sf, lora._bw, lora._cr, ldro);
}

void setHeaderType(sx126x_t lora, uint8_t headerType)
{
    setLoRaPacket(lora, headerType, lora._preambleLength, lora._payloadLength, lora._crcType, lora._invertIq);
}

void setPreambleLength(sx126x_t lora, uint16_t preambleLength)
{
    setLoRaPacket(lora, lora._headerType, preambleLength, lora._payloadLength, lora._crcType, lora._invertIq);
}

void setPayloadLength(sx126x_t lora, uint8_t payloadLength)
{
    setLoRaPacket(lora, lora._headerType, lora._preambleLength, payloadLength, lora._crcType, lora._invertIq);
}

/**
 * @param[in] crcType: default true
*/
void setCrcEnable(sx126x_t lora, bool crcType)
{
    setLoRaPacket(lora, lora._headerType, lora._preambleLength, lora._payloadLength, crcType, lora._invertIq);
}

/**
 * @param[in] invertIq: default true
*/
void setInvertIq(sx126x_t lora, bool invertIq)
{
    setLoRaPacket(lora, lora._headerType, lora._preambleLength, lora._payloadLength, lora._crcType, invertIq);
}

void setSyncWord(sx126x_t lora, uint16_t syncWord)
{
    uint8_t buf[2];
    buf[0] = syncWord >> 8;
    buf[1] = syncWord & 0xFF;
    if (syncWord <= 0xFF) {
        buf[0] = (syncWord & 0xF0) | 0x04;
        buf[1] = (syncWord << 4) | 0x04;
    }
    sx126x_writeRegister(lora.sx126x_drv, SX126X_REG_LORA_SYNC_WORD_MSB, buf, 2);
}

void setFskModulation(sx126x_t lora, uint32_t br, uint8_t pulseShape, uint8_t bandwidth, uint32_t Fdev)
{
    sx126x_setModulationParamsFSK(lora.sx126x_drv, br, pulseShape, bandwidth, Fdev);
}

void setFskPacket(sx126x_t lora, uint16_t preambleLength, uint8_t preambleDetector, uint8_t syncWordLength, uint8_t addrComp, uint8_t packetType, uint8_t payloadLength, uint8_t crcType, uint8_t whitening)
{
    sx126x_setPacketParamsFSK(lora.sx126x_drv, preambleLength, preambleDetector, syncWordLength, addrComp, packetType, payloadLength, crcType, whitening);
}

void setFskSyncWord(sx126x_t lora, uint8_t* sw, uint8_t swLen)
{
    sx126x_writeRegister(lora.sx126x_drv, SX126X_REG_FSK_SYNC_WORD_0, sw, swLen);
}

void setFskAdress(sx126x_t lora, uint8_t nodeAddr, uint8_t broadcastAddr)
{
    uint8_t buf[2] = {nodeAddr, broadcastAddr};
    sx126x_writeRegister(lora.sx126x_drv, SX126X_REG_FSK_NODE_ADDRESS, buf, 2);
}

void setFskCrc(sx126x_t lora, uint16_t crcInit, uint16_t crcPolynom)
{
    uint8_t buf[4];
    buf[0] = crcInit >> 8;
    buf[1] = crcInit & 0xFF;
    buf[2] = crcPolynom >> 8;
    buf[3] = crcPolynom & 0xFF;
    sx126x_writeRegister(lora.sx126x_drv, SX126X_REG_FSK_CRC_INITIAL_MSB, buf, 4);
}

void setFskWhitening(sx126x_t lora, uint16_t whitening)
{
    uint8_t buf[2];
    buf[0] = whitening >> 8;
    buf[1] = whitening & 0xFF;
    sx126x_writeRegister(lora.sx126x_drv, SX126X_REG_FSK_WHITENING_INITIAL_MSB, buf, 2);
}

// Transmit related methods
void beginPacket(sx126x_t lora)
{
    // reset payload length and buffer index
    lora._payloadTxRx = 0;
    sx126x_setBufferBaseAddress(lora.sx126x_drv, lora._bufferIndex, lora._bufferIndex + 0xFF);
    
    // set txen pin to low and rxen pin to high
    if ((lora._rxen != NULL) && (lora._txen != NULL) && (sx126x_drv_null_ptr_check(lora.sx126x_drv) == SX126X_OK)) {        
        lora.sx126x_drv->gpio_reset_pin(lora._rxen->port, lora._rxen->pin);
        lora.sx126x_drv->gpio_set_pin(lora._txen->port, lora._txen->pin);
        lora._pinToLow = lora._txen;
    }
    
    sx126x_fixLoRaBw500(lora.sx126x_drv, lora._bw);
}

/**
 * @param timeout: default SX126X_TX_SINGLE
*/
bool endPacket(sx126x_t lora, uint32_t timeout)
{
    // skip to enter TX mode when previous TX operation incomplete
    if (getMode(lora) == SX126X_STATUS_MODE_TX) return false;

    // clear previous interrupt and set TX done, and TX timeout as interrupt source
    _irqSetup(lora, SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT);

    // set packet payload length
    setLoRaPacket(lora, lora._headerType, lora._preambleLength, lora._payloadTxRx, lora._crcType, lora._invertIq);

    // set status to TX wait
    lora._statusWait = SX126X_STATUS_TX_WAIT;
    lora._statusIrq = 0x0000;
    // calculate TX timeout config
    uint32_t txTimeout = timeout << 6;
    if (txTimeout > 0x00FFFFFF) txTimeout = SX126X_TX_SINGLE;

    // set device to transmit mode with configured timeout or single operation
    sx126x_setTx(lora.sx126x_drv, txTimeout);
    lora._transmitTime = millis();

    // set operation status to wait and attach TX interrupt handler
    // if (lora._irq != NULL) {
    //     attachInterrupt(_irqStatic, SX126x::_interruptTx, RISING);
    // }
    return true;
}

void write_byte(sx126x_t lora, uint8_t data)
{
    // write single byte of package to be transmitted
    sx126x_writeBuffer(lora.sx126x_drv, lora._bufferIndex, &data, 1);
    lora._bufferIndex++;
    lora._payloadTxRx++;
}

void write_ubuffer(sx126x_t lora, uint8_t* data, uint8_t length)
{
    // write multiple bytes of package to be transmitted
    sx126x_writeBuffer(lora.sx126x_drv, lora._bufferIndex, data, length);
    lora._bufferIndex += length;
    lora._payloadTxRx += length;
}

void write_sbuffer(sx126x_t lora, char* data, uint8_t length)
{
    // write multiple bytes of package to be transmitted for char type
    uint8_t* data_ = (uint8_t*) data;
    sx126x_writeBuffer(lora.sx126x_drv, lora._bufferIndex, data_, length);
    lora._bufferIndex += length;
    lora._payloadTxRx += length;
}

// Common Operational methods
void put(sx126x_t lora, void *data, const uint8_t length)
{
    union conv {
        void *Data;
        uint8_t Binary[length];
    };
    union conv u;
    u.Data = data;
    sx126x_writeBuffer(lora.sx126x_drv, lora._bufferIndex, u.Binary, length);
    lora._bufferIndex += length;
    lora._payloadTxRx += length;
}

// Receive related methods
/**
 * @param[in] timeout: default SX126X_RX_SINGLE
*/
bool request(sx126x_t lora, uint32_t timeout)
{
    // skip to enter RX mode when previous RX operation incomplete
    if (getMode(lora) == SX126X_STATUS_MODE_RX) return false;

    // clear previous interrupt and set RX done, RX timeout, header error, and CRC error as interrupt source
    _irqSetup(lora, SX126X_IRQ_RX_DONE | SX126X_IRQ_TIMEOUT | SX126X_IRQ_HEADER_ERR | SX126X_IRQ_CRC_ERR);

    // set status to RX wait or RX continuous wait
    lora._statusWait = SX126X_STATUS_RX_WAIT;
    lora._statusIrq = 0x0000;
    // calculate RX timeout config
    uint32_t rxTimeout = timeout << 6;
    if (rxTimeout > 0x00FFFFFF) rxTimeout = SX126X_RX_SINGLE;
    if (timeout == SX126X_RX_CONTINUOUS) {
        rxTimeout = SX126X_RX_CONTINUOUS;
        lora._statusWait = SX126X_STATUS_RX_CONTINUOUS;
    }

    // set txen pin to low and rxen pin to high
    if ((lora._rxen != NULL) && (lora._txen != NULL) && (sx126x_drv_null_ptr_check(lora.sx126x_drv) == SX126X_OK)) {        
        lora.sx126x_drv->gpio_reset_pin(lora._txen->port, lora._txen->pin);
        lora.sx126x_drv->gpio_set_pin(lora._rxen->port, lora._rxen->pin);
        lora._pinToLow = lora._rxen;
    }

    // set device to receive mode with configured timeout, single, or continuous operation
    sx126x_setRx(lora.sx126x_drv, rxTimeout);

    // set operation status to wait and attach RX interrupt handler
    // if (_irq != -1) {
    //     if (timeout == SX126X_RX_CONTINUOUS) {
    //         attachInterrupt(_irqStatic, SX126x::_interruptRxContinuous, RISING);
    //     } else {
    //         attachInterrupt(_irqStatic, SX126x::_interruptRx, RISING);
    //     }
    // }
    return true;
}

bool listen(sx126x_t lora, uint32_t rxPeriod, uint32_t sleepPeriod)
{
    // skip to enter RX mode when previous RX operation incomplete
    if (getMode(lora) == SX126X_STATUS_MODE_RX) return false;

    // clear previous interrupt and set RX done, RX timeout, header error, and CRC error as interrupt source
    _irqSetup(lora, SX126X_IRQ_RX_DONE | SX126X_IRQ_TIMEOUT | SX126X_IRQ_HEADER_ERR | SX126X_IRQ_CRC_ERR);

    // set status to RX wait
    lora._statusWait = SX126X_STATUS_RX_WAIT;
    lora._statusIrq = 0x0000;
    // calculate RX period and sleep period config
    rxPeriod = rxPeriod << 6;
    sleepPeriod = sleepPeriod << 6;
    if (rxPeriod > 0x00FFFFFF) rxPeriod = 0x00FFFFFF;
    if (sleepPeriod > 0x00FFFFFF) sleepPeriod = 0x00FFFFFF;

    // set txen pin to low and rxen pin to high
    if ((lora._rxen != NULL) && (lora._txen != NULL) && (sx126x_drv_null_ptr_check(lora.sx126x_drv) == SX126X_OK)) {        
        lora.sx126x_drv->gpio_reset_pin(lora._txen->port, lora._txen->pin);
        lora.sx126x_drv->gpio_set_pin(lora._rxen->port, lora._rxen->pin);
        lora._pinToLow = lora._rxen;
    }

    // set device to receive mode with configured receive and sleep period
    sx126x_setRxDutyCycle(lora.sx126x_drv, rxPeriod, sleepPeriod);

    // set operation status to wait and attach RX interrupt handler
    // if (_irq != -1) {
    //     attachInterrupt(_irqStatic, SX126x::_interruptRx, RISING);
    // }
    return true;
}

uint8_t available(sx126x_t lora)
{
    return lora._payloadTxRx;
}

uint8_t read_byte(sx126x_t lora)
{
    // read single byte of received package
    uint8_t buf;
    sx126x_readBuffer(lora.sx126x_drv, lora._bufferIndex, &buf, 1);
    lora._bufferIndex++;
    if (lora._payloadTxRx > 0) lora._payloadTxRx--;
    return buf;
}

uint8_t read_ubuffer(sx126x_t lora, uint8_t* data, uint8_t length)
{
    // read multiple bytes of received package
    sx126x_readBuffer(lora.sx126x_drv, lora._bufferIndex, data, length);
    lora._bufferIndex += length;
    lora._payloadTxRx = lora._payloadTxRx > length ? lora._payloadTxRx - length : 0;
    return lora._payloadTxRx > length ? length : lora._payloadTxRx;
}

uint8_t read_sbuffer(sx126x_t lora, char* data, uint8_t length)
{
    // write multiple bytes of package to be transmitted for char type
    uint8_t* data_ = (uint8_t*) data;
    sx126x_readBuffer(lora.sx126x_drv, lora._bufferIndex, data_, length);
    lora._bufferIndex += length;
    lora._payloadTxRx = lora._payloadTxRx > length ? lora._payloadTxRx - length : 0;
    return lora._payloadTxRx > length ? length : lora._payloadTxRx;
}

/**
 * @param length: default 0
*/
void purge(sx126x_t lora, uint8_t length)
{
    // subtract or reset received payload length
    lora._payloadTxRx = (lora._payloadTxRx > length) && length ? lora._payloadTxRx - length : 0;
    lora._bufferIndex += length;
}

uint8_t get(sx126x_t lora, void *data, const uint8_t length)
{
    union conv {
        void *Data;
        uint8_t Binary[length];
    };
    union conv u;
    sx126x_readBuffer(lora.sx126x_drv, lora._bufferIndex, u.Binary, length);
    data = u.Data;
    lora._bufferIndex += length;
    lora._payloadTxRx = lora._payloadTxRx > length ? lora._payloadTxRx - length : 0;
    return lora._payloadTxRx > length ? length : lora._payloadTxRx;
}

/**
 * @param[in] timeout: default 0
*/
bool wait(sx126x_t lora, uint32_t timeout, void (*yield)())
{
    // immediately return when currently not waiting transmit or receive process
    if (lora._statusIrq) return true;

    // wait transmit or receive process finish by checking interrupt status or IRQ status
    uint16_t irqStat = 0x0000;
    sx126x_driver_t *sx126x_drv = lora.sx126x_drv;
    uint8_t ret = sx126x_drv_null_ptr_check(sx126x_drv);
    if (ret != SX126X_OK) return ret;
    
    device_get_system_tick_count_fptr_t millis = sx126x_drv->sx126x_driver->get_system_tick_count;
    uint32_t t = millis();
    while (irqStat == 0x0000 && lora._statusIrq == 0x0000) {
        // only check IRQ status register for non interrupt operation
        if (lora._irq == -1) sx126x_getIrqStatus(sx126x_drv, &irqStat);
        // return when timeout reached
        if (millis() - t > timeout && timeout != 0) return false;
        if (yield != NULL) yield();
    }

    if (lora._statusIrq) {
        // immediately return when interrupt signal hit
        return true;
    } else if (lora._statusWait == SX126X_STATUS_TX_WAIT) {
        // for transmit, calculate transmit time and set back txen pin to low
        lora._transmitTime = millis() -lora. _transmitTime;
        if (lora._txen != NULL) sx126x_drv->gpio_reset_pin(lora._txen->port, lora._txen->pin);
    } else if (lora._statusWait == SX126X_STATUS_RX_WAIT) {
        // for receive, get received payload length and buffer index and set back rxen pin to low
        sx126x_getRxBufferStatus(sx126x_drv, &lora._payloadTxRx, &lora._bufferIndex);
        if (lora._rxen != NULL) lora.sx126x_drv->gpio_reset_pin(lora._rxen->port, lora._rxen->pin);
        sx126x_fixRxTimeout(sx126x_drv);
    } else if (lora._statusWait == SX126X_STATUS_RX_CONTINUOUS) {
        // for receive continuous, get received payload length and buffer index and clear IRQ status
        sx126x_getRxBufferStatus(sx126x_drv, &lora._payloadTxRx, &lora._bufferIndex);
        sx126x_clearIrqStatus(sx126x_drv, 0x03FF);
    }

    // store IRQ status
    lora._statusIrq = irqStat;
    return true;
}

uint8_t status(sx126x_t lora)
{
    // set back status IRQ for RX continuous operation
    uint16_t statusIrq = lora._statusIrq;
    if (lora._statusWait == SX126X_STATUS_RX_CONTINUOUS) {
        lora._statusIrq = 0x0000;
    }

    // get status for transmit and receive operation based on status IRQ
    if (statusIrq & SX126X_IRQ_TIMEOUT) {
        if (lora._statusWait == SX126X_STATUS_TX_WAIT) return SX126X_STATUS_TX_TIMEOUT;
        else return SX126X_STATUS_RX_TIMEOUT;
    }
    else if (statusIrq & SX126X_IRQ_HEADER_ERR) return SX126X_STATUS_HEADER_ERR;
    else if (statusIrq & SX126X_IRQ_CRC_ERR) return SX126X_STATUS_CRC_ERR;
    else if (statusIrq & SX126X_IRQ_TX_DONE) return SX126X_STATUS_TX_DONE;
    else if (statusIrq & SX126X_IRQ_RX_DONE) return SX126X_STATUS_RX_DONE;

    // return TX or RX wait status
    return lora._statusWait;
}

uint32_t transmitTime(sx126x_t lora)
{
    // get transmit time in millisecond (ms)
    return lora._transmitTime;
}

float dataRate(sx126x_t lora)
{
    // get data rate last transmitted package in kbps
    return 1000.0 * lora._payloadTxRx / lora._transmitTime;
}

int16_t packetRssi(sx126x_t lora)
{
    uint8_t rssiPkt, snrPkt, signalRssiPkt;
    sx126x_getPacketStatus(lora.sx126x_drv, &rssiPkt, &snrPkt, &signalRssiPkt);
    return (rssiPkt / -2);
}

float snr(sx126x_t lora)
{
    // get signal to noise ratio (SNR) of last incoming package
    uint8_t rssiPkt, snrPkt, signalRssiPkt;
    sx126x_getPacketStatus(lora.sx126x_drv, &rssiPkt, &snrPkt, &signalRssiPkt);
    return ((int8_t) snrPkt / 4.0);
}

int16_t signalRssi(sx126x_t lora)
{
    uint8_t rssiPkt, snrPkt, signalRssiPkt;
    sx126x_getPacketStatus(lora.sx126x_drv, &rssiPkt, &snrPkt, &signalRssiPkt);
    return (signalRssiPkt / -2);
}

int16_t rssiInst(sx126x_t lora)
{
    uint8_t rssiInst;
    sx126x_getRssiInst(lora.sx126x_drv, &rssiInst);
    return (rssiInst / -2);
}

uint16_t getError(sx126x_t lora)
{
    uint16_t error;
    sx126x_getDeviceErrors(lora.sx126x_drv, &error);
    sx126x_clearDeviceErrors(lora.sx126x_drv);
    return error;
}

uint32_t random(sx126x_t lora)
{
    // generate random number from register and previous random number
    uint8_t buf[4];
    uint8_t ret = sx126x_drv_null_ptr_check(lora.sx126x_drv);
    device_get_system_tick_count_fptr_t millis;
    if (ret != SX126X_OK) return 0;
    millis = lora.sx126x_drv->sx126x_driver->get_system_tick_count;
    
    sx126x_readRegister(lora.sx126x_drv, SX126X_REG_RANDOM_NUMBER_GEN, buf, 4);
    uint32_t number = ((uint32_t) buf[0] << 24) | ((uint32_t) buf[1] << 16) | ((uint32_t) buf[2] << 8) | ((uint32_t) buf[0]);
    uint32_t n = lora._random;
    number = number ^ ((~n << 16) | n);
    // combine random number with random number seeded by time
    srand(millis());
    number = number ^ (((uint32_t) rand() << 16) | rand());
    lora._random = number >> 8;
    return number;
}

void _irqSetup(sx126x_t lora, uint16_t irqMask)
{
    // clear IRQ status of previous transmit or receive operation
    sx126x_clearIrqStatus(lora.sx126x_drv, 0x03FF);

    // set selected interrupt source
    uint16_t dio1Mask = 0x0000;
    uint16_t dio2Mask = 0x0000;
    uint16_t dio3Mask = 0x0000;
    if (lora._dio == 2) dio2Mask = irqMask;
    else if (lora._dio == 3) dio3Mask = irqMask;
    else dio1Mask = irqMask;
    sx126x_setDioIrqParams(lora.sx126x_drv, irqMask, dio1Mask, dio2Mask, dio3Mask);
}

void _interruptTx(sx126x_t lora)
{
    sx126x_driver_t *sx126x_drv = lora.sx126x_drv;
    uint8_t ret = sx126x_drv_null_ptr_check(sx126x_drv);
    device_get_system_tick_count_fptr_t millis;
    if (ret != SX126X_OK) return;
    millis = sx126x_drv->sx126x_driver->get_system_tick_count;
    // calculate transmit time
    lora._transmitTime = millis() - lora._transmitTime;

    // set back txen pin to low and detach interrupt
    if (lora._pinToLow != NULL) sx126x_drv->gpio_reset_pin(lora._pinToLow->port, lora._pinToLow->pin);
    // detachInterrupt(_irqStatic);

    // store IRQ status
    uint16_t buf;
    sx126x_getIrqStatus(sx126x_drv, &buf);
    lora._statusIrq = buf;

    // call onTransmit function
    if (lora._onTransmit) {
        lora._onTransmit();
    }
}

void _interruptRx(sx126x_t lora)
{
    sx126x_driver_t *sx126x_drv = lora.sx126x_drv;
    uint8_t ret = sx126x_drv_null_ptr_check(sx126x_drv);
    if (ret != SX126X_OK) return;
    // set back rxen pin to low and detach interrupt
    if (lora._pinToLow != NULL) sx126x_drv->gpio_reset_pin(lora._pinToLow->port, lora._pinToLow->pin);
    // detachInterrupt(_irqStatic);
    sx126x_fixRxTimeout(sx126x_drv);

    // store IRQ status
    uint16_t buf;
    sx126x_getIrqStatus(lora.sx126x_drv, &buf);
    lora._statusIrq = buf;

    // get received payload length and buffer index
    sx126x_getRxBufferStatus(sx126x_drv, &lora._payloadTxRx, &lora._bufferIndex);

    // call onReceive function
    if (lora._onReceive) {
        lora._onReceive();
    }
}

void _interruptRxContinuous(sx126x_t lora)
{
    // store IRQ status
    uint16_t buf;
    sx126x_getIrqStatus(lora.sx126x_drv, &buf);
    lora._statusIrq = buf;

    // clear IRQ status
    sx126x_clearIrqStatus(lora.sx126x_drv, 0x03FF);

    // get received payload length and buffer index
    sx126x_getRxBufferStatus(lora.sx126x_drv, &lora._payloadTxRx, &lora._bufferIndex);

    // call onReceive function
    if (lora._onReceive) {
        lora._onReceive();
    }
}

void onTransmit(sx126x_t lora, void(*callback)())
{
    // register onTransmit function to call every transmit done
    lora._onTransmit = callback;
}

void onReceive(sx126x_t lora, void(*callback)())
{
    // register onReceive function to call every receive done
    lora._onReceive = callback;
}

SX126X_RET_TYPE sx126x_null_ptr_check(sx126x_t *lora)
{
    if (lora == NULL || lora->_reset == NULL || sx126x_drv_null_ptr_check(lora->sx126x_drv))
    {
        return SX126X_E_NULL_PTR;
    }
    return SX126X_OK;    
}
