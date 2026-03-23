/*!
 * @file QMI8658.cpp
 *
 * @brief Implementation of the QMI8658 6-axis IMU Arduino library.
 *
 * @section license License
 *
 * MIT license, all text above must be included in any redistribution.
 */

#include "QMI8658.h"

// ============================================================================
// Construction & Initialization
// ============================================================================

QMI8658::QMI8658()
    : _wire(&Wire), _address(QMI8658_I2C_ADDRESS_LOW), _error(0),
      _accelRange(AccelRange::RANGE_8G), _gyroRange(GyroRange::RANGE_512DPS),
      _accelScale(0.0f), _gyroScale(0.0f) {}

QMI8658::QMI8658(uint8_t address)
    : _wire(&Wire), _address(address), _error(0),
      _accelRange(AccelRange::RANGE_8G), _gyroRange(GyroRange::RANGE_512DPS),
      _accelScale(0.0f), _gyroScale(0.0f) {}

QMI8658::QMI8658(TwoWire &wire, uint8_t address)
    : _wire(&wire), _address(address), _error(0),
      _accelRange(AccelRange::RANGE_8G), _gyroRange(GyroRange::RANGE_512DPS),
      _accelScale(0.0f), _gyroScale(0.0f) {}

bool QMI8658::begin() {
    _wire->begin();
    return _init();
}

bool QMI8658::begin(int sda, int scl) {
#if defined(ESP32) || defined(ESP8266) || defined(ARDUINO_ARCH_RP2040) || \
    defined(ARDUINO_ARCH_STM32) || defined(ARDUINO_ARCH_NRF52) || \
    defined(ARDUINO_ARCH_SAMD)
    _wire->begin(sda, scl);
#else
    // Platforms with fixed I2C pins (e.g., AVR): ignore pin arguments.
    (void)sda;
    (void)scl;
    _wire->begin();
#endif
    return _init();
}

bool QMI8658::_init() {
    // Verify sensor identity
    uint8_t id = whoAmI();
    if (id != QMI8658_WHO_AM_I_DEFAULT) {
        _error = 1;
        return false;
    }

    // Soft reset to known state
    reset();

    // Enable address auto-increment (CTRL1 bit 6)
    // INT pins default: active-low, push-pull
    writeRegister(REG_CTRL1, 0x40);

    // Default configuration
    configAccel(AccelRange::RANGE_8G, AccelODR::ODR_500HZ);
    configGyro(GyroRange::RANGE_512DPS, GyroODR::ODR_500HZ);

    // Enable both accelerometer and gyroscope
    writeRegister(REG_CTRL7, CTRL7_ACC_EN | CTRL7_GYR_EN);

    delay(10); // Allow sensors to stabilize

    return true;
}

// ============================================================================
// Configuration
// ============================================================================

bool QMI8658::configAccel(AccelRange range, AccelODR odr) {
    uint8_t val = (static_cast<uint8_t>(range) << 4) | static_cast<uint8_t>(odr);
    writeRegister(REG_CTRL2, val);
    _accelRange = range;
    updateAccelScale();
    return true;
}

bool QMI8658::configGyro(GyroRange range, GyroODR odr) {
    uint8_t val = (static_cast<uint8_t>(range) << 4) | static_cast<uint8_t>(odr);
    writeRegister(REG_CTRL3, val);
    _gyroRange = range;
    updateGyroScale();
    return true;
}

bool QMI8658::configLowPassFilter(LowPassFilter accelLPF, LowPassFilter gyroLPF) {
    uint8_t val = 0;

    // Accelerometer LPF: bits 2:0 control bandwidth, bit 0 enables
    if (accelLPF != LowPassFilter::DISABLED) {
        val |= (static_cast<uint8_t>(accelLPF) << 1) | 0x01;
    }

    // Gyroscope LPF: bits 6:4 control bandwidth, bit 4 enables
    if (gyroLPF != LowPassFilter::DISABLED) {
        val |= (static_cast<uint8_t>(gyroLPF) << 5) | 0x10;
    }

    writeRegister(REG_CTRL5, val);
    return true;
}

void QMI8658::enableAccel(bool enable) {
    uint8_t ctrl7 = readRegister(REG_CTRL7);
    if (enable) {
        ctrl7 |= CTRL7_ACC_EN;
    } else {
        ctrl7 &= ~CTRL7_ACC_EN;
    }
    writeRegister(REG_CTRL7, ctrl7);
}

void QMI8658::enableGyro(bool enable) {
    uint8_t ctrl7 = readRegister(REG_CTRL7);
    if (enable) {
        ctrl7 |= CTRL7_GYR_EN;
        ctrl7 &= ~CTRL7_GYR_SNOOZE; // Full enable, not snooze
    } else {
        ctrl7 &= ~(CTRL7_GYR_EN | CTRL7_GYR_SNOOZE);
    }
    writeRegister(REG_CTRL7, ctrl7);
}

void QMI8658::enableSyncSample(bool enable) {
    uint8_t ctrl7 = readRegister(REG_CTRL7);
    if (enable) {
        ctrl7 |= CTRL7_SYNC_EN;
    } else {
        ctrl7 &= ~CTRL7_SYNC_EN;
    }
    writeRegister(REG_CTRL7, ctrl7);
}

void QMI8658::setClock(uint32_t clockHz) {
    _wire->setClock(clockHz);
}

// ============================================================================
// Basic Data Reading
// ============================================================================

bool QMI8658::dataReady() {
    uint8_t status = readRegister(REG_STATUS0);
    return (status & (STATUS0_ACC_RDY | STATUS0_GYR_RDY)) != 0;
}

AccelData QMI8658::readAccel() {
    AccelData data;
    uint8_t buf[6];
    readRegisters(REG_AX_L, buf, 6);

    data.rawX = (int16_t)(buf[0] | ((uint16_t)buf[1] << 8));
    data.rawY = (int16_t)(buf[2] | ((uint16_t)buf[3] << 8));
    data.rawZ = (int16_t)(buf[4] | ((uint16_t)buf[5] << 8));
    data.x = data.rawX * _accelScale;
    data.y = data.rawY * _accelScale;
    data.z = data.rawZ * _accelScale;

    return data;
}

GyroData QMI8658::readGyro() {
    GyroData data;
    uint8_t buf[6];
    readRegisters(REG_GX_L, buf, 6);

    data.rawX = (int16_t)(buf[0] | ((uint16_t)buf[1] << 8));
    data.rawY = (int16_t)(buf[2] | ((uint16_t)buf[3] << 8));
    data.rawZ = (int16_t)(buf[4] | ((uint16_t)buf[5] << 8));
    data.x = data.rawX * _gyroScale;
    data.y = data.rawY * _gyroScale;
    data.z = data.rawZ * _gyroScale;

    return data;
}

IMUData QMI8658::readBoth() {
    IMUData data;
    uint8_t buf[12];
    readRegisters(REG_AX_L, buf, 12);

    data.accel.rawX = (int16_t)(buf[0]  | ((uint16_t)buf[1] << 8));
    data.accel.rawY = (int16_t)(buf[2]  | ((uint16_t)buf[3] << 8));
    data.accel.rawZ = (int16_t)(buf[4]  | ((uint16_t)buf[5] << 8));
    data.gyro.rawX  = (int16_t)(buf[6]  | ((uint16_t)buf[7] << 8));
    data.gyro.rawY  = (int16_t)(buf[8]  | ((uint16_t)buf[9] << 8));
    data.gyro.rawZ  = (int16_t)(buf[10] | ((uint16_t)buf[11] << 8));

    data.accel.x = data.accel.rawX * _accelScale;
    data.accel.y = data.accel.rawY * _accelScale;
    data.accel.z = data.accel.rawZ * _accelScale;
    data.gyro.x  = data.gyro.rawX * _gyroScale;
    data.gyro.y  = data.gyro.rawY * _gyroScale;
    data.gyro.z  = data.gyro.rawZ * _gyroScale;

    return data;
}

float QMI8658::readTemperature() {
    uint8_t buf[2];
    readRegisters(REG_TEMP_L, buf, 2);
    int16_t raw = (int16_t)(buf[0] | ((uint16_t)buf[1] << 8));
    return raw / 256.0f;
}

uint32_t QMI8658::readTimestamp() {
    uint8_t buf[3];
    readRegisters(REG_TIMESTAMP_L, buf, 3);
    return (uint32_t)buf[0] | ((uint32_t)buf[1] << 8) | ((uint32_t)buf[2] << 16);
}

// ============================================================================
// Interrupt Configuration
// ============================================================================

void QMI8658::configInterrupt(InterruptPin pin, bool activeLow, bool pushPull) {
    uint8_t ctrl1 = readRegister(REG_CTRL1);

    if (pin == InterruptPin::INT1) {
        // CTRL1 bit 3: INT1 enable, bit 0: INT1 level (0=active-high, 1=active-low)
        ctrl1 |= 0x08; // Enable INT1 output
        if (activeLow) {
            ctrl1 |= 0x01;
        } else {
            ctrl1 &= ~0x01;
        }
        if (!pushPull) {
            ctrl1 |= 0x04; // Open-drain mode for INT1
        } else {
            ctrl1 &= ~0x04;
        }
    } else {
        // CTRL1 bit 4: INT2 enable, bit 1: INT2 level
        ctrl1 |= 0x10; // Enable INT2 output
        if (activeLow) {
            ctrl1 |= 0x02;
        } else {
            ctrl1 &= ~0x02;
        }
        if (!pushPull) {
            ctrl1 |= 0x04; // Open-drain shared bit
        } else {
            ctrl1 &= ~0x04;
        }
    }

    // Preserve address auto-increment (bit 6)
    ctrl1 |= 0x40;
    writeRegister(REG_CTRL1, ctrl1);
}

void QMI8658::enableAccelDataReadyInt(InterruptPin pin) {
    configInterrupt(pin);
    // Data-ready interrupts are routed based on CTRL1 configuration.
    // INT1 fires for accel/gyro data ready by default when enabled.
    // INT2 can be configured via CTRL1 to route data-ready.
    if (pin == InterruptPin::INT2) {
        uint8_t ctrl1 = readRegister(REG_CTRL1);
        ctrl1 |= 0x10; // Enable INT2
        writeRegister(REG_CTRL1, ctrl1);
    }
}

void QMI8658::enableGyroDataReadyInt(InterruptPin pin) {
    configInterrupt(pin);
    if (pin == InterruptPin::INT2) {
        uint8_t ctrl1 = readRegister(REG_CTRL1);
        ctrl1 |= 0x10;
        writeRegister(REG_CTRL1, ctrl1);
    }
}

void QMI8658::disableDataReadyInt() {
    uint8_t ctrl1 = readRegister(REG_CTRL1);
    ctrl1 &= ~0x18; // Disable both INT1 and INT2 outputs
    ctrl1 |= 0x40;  // Keep auto-increment
    writeRegister(REG_CTRL1, ctrl1);
}

// ============================================================================
// Tap Detection
// ============================================================================

bool QMI8658::enableTapDetection(bool singleTap, bool doubleTap,
                                  InterruptPin pin) {
    // Configure tap parameters via CAL registers + CTRL9
    // CAL1_L: Tap axis enable and priority
    //   Bits 2:0 = XYZ enable (0x07 = all axes)
    //   Bits 5:3 = axis priority (0x00 = X>Y>Z default)
    writeRegister(REG_CAL1_L, 0x07);

    // CAL1_H: Tap threshold for X axis (default: 0x40 = moderate sensitivity)
    writeRegister(REG_CAL1_H, 0x40);

    // CAL2_L: Tap threshold for Y axis
    writeRegister(REG_CAL2_L, 0x40);

    // CAL2_H: Tap threshold for Z axis
    writeRegister(REG_CAL2_H, 0x40);

    // CAL3_L: Tap timing parameters (tap duration)
    // Bits 7:0: PeakWindow - duration of tap pulse detection
    writeRegister(REG_CAL3_L, 0x1A); // ~26 samples

    // CAL3_H: Tap quiet period (between taps for double-tap)
    writeRegister(REG_CAL3_H, 0x0C); // ~12 samples

    // CAL4_L: Double-tap timing window
    if (doubleTap) {
        writeRegister(REG_CAL4_L, 0x30); // ~48 samples window for double-tap
    } else {
        writeRegister(REG_CAL4_L, 0x00);
    }

    // Send CTRL9 command to apply tap configuration
    if (!sendCtrl9Command(CTRL9_CMD_CONFIGURE_TAP)) {
        return false;
    }

    // Enable tap detection in CTRL8 (motion event control)
    uint8_t ctrl8 = readRegister(REG_CTRL8);
    ctrl8 |= 0x01; // Bit 0: tap enable

    // Route to selected interrupt pin (bit 6: 0=INT1, 1=INT2)
    if (pin == InterruptPin::INT2) {
        ctrl8 |= 0x40;
    } else {
        ctrl8 &= ~0x40;
    }

    writeRegister(REG_CTRL8, ctrl8);

    // Make sure the interrupt pin is enabled
    configInterrupt(pin);

    return true;
}

void QMI8658::disableTapDetection() {
    uint8_t ctrl8 = readRegister(REG_CTRL8);
    ctrl8 &= ~0x01; // Disable tap (bit 0)
    writeRegister(REG_CTRL8, ctrl8);
}

uint8_t QMI8658::readTapStatus() {
    return readRegister(REG_TAP_STATUS);
}

// ============================================================================
// Motion Detection
// ============================================================================

bool QMI8658::enableAnyMotion(InterruptPin pin) {
    // Configure any-motion parameters via CAL registers
    // CAL1_L: AnyMotion X threshold low byte
    writeRegister(REG_CAL1_L, 0x80); // Moderate threshold
    // CAL1_H: AnyMotion X threshold high byte
    writeRegister(REG_CAL1_H, 0x00);
    // CAL2_L: AnyMotion Y threshold low byte
    writeRegister(REG_CAL2_L, 0x80);
    // CAL2_H: AnyMotion Y threshold high byte
    writeRegister(REG_CAL2_H, 0x00);
    // CAL3_L: AnyMotion Z threshold low byte
    writeRegister(REG_CAL3_L, 0x80);
    // CAL3_H: AnyMotion Z threshold high byte
    writeRegister(REG_CAL3_H, 0x00);
    // CAL4_L: Logic mode (AND/OR) and blanking time
    // Bit 0: 0=OR logic (any axis), 1=AND logic (all axes)
    // Bits 7:2: blanking samples
    writeRegister(REG_CAL4_L, 0x04); // OR logic, small blanking
    // CAL4_H: Not used for any-motion
    writeRegister(REG_CAL4_H, 0x00);

    if (!sendCtrl9Command(CTRL9_CMD_CONFIGURE_MOTION)) {
        return false;
    }

    // Enable any-motion in CTRL8
    uint8_t ctrl8 = readRegister(REG_CTRL8);
    ctrl8 |= 0x02; // Bit 1: any-motion enable

    if (pin == InterruptPin::INT2) {
        ctrl8 |= 0x40;
    } else {
        ctrl8 &= ~0x40;
    }

    writeRegister(REG_CTRL8, ctrl8);
    configInterrupt(pin);

    return true;
}

bool QMI8658::enableNoMotion(InterruptPin pin) {
    // Configure no-motion parameters
    writeRegister(REG_CAL1_L, 0x80); // Threshold low
    writeRegister(REG_CAL1_H, 0x00); // Threshold high
    writeRegister(REG_CAL2_L, 0x80);
    writeRegister(REG_CAL2_H, 0x00);
    writeRegister(REG_CAL3_L, 0x80);
    writeRegister(REG_CAL3_H, 0x00);
    // CAL4_L: Logic mode and no-motion duration
    writeRegister(REG_CAL4_L, 0x60); // AND logic, longer blanking for no-motion
    writeRegister(REG_CAL4_H, 0x00);

    if (!sendCtrl9Command(CTRL9_CMD_CONFIGURE_MOTION)) {
        return false;
    }

    uint8_t ctrl8 = readRegister(REG_CTRL8);
    ctrl8 |= 0x04; // Bit 2: no-motion enable

    if (pin == InterruptPin::INT2) {
        ctrl8 |= 0x40;
    } else {
        ctrl8 &= ~0x40;
    }

    writeRegister(REG_CTRL8, ctrl8);
    configInterrupt(pin);

    return true;
}

bool QMI8658::enableSignificantMotion(InterruptPin pin) {
    // Significant motion uses the same CTRL9 motion config command
    // but is activated via CTRL8 bit 3.
    // First configure any-motion thresholds (significant motion builds on them)
    writeRegister(REG_CAL1_L, 0x80);
    writeRegister(REG_CAL1_H, 0x00);
    writeRegister(REG_CAL2_L, 0x80);
    writeRegister(REG_CAL2_H, 0x00);
    writeRegister(REG_CAL3_L, 0x80);
    writeRegister(REG_CAL3_H, 0x00);
    writeRegister(REG_CAL4_L, 0x04);
    writeRegister(REG_CAL4_H, 0x00);

    if (!sendCtrl9Command(CTRL9_CMD_CONFIGURE_MOTION)) {
        return false;
    }

    uint8_t ctrl8 = readRegister(REG_CTRL8);
    ctrl8 |= 0x08; // Bit 3: significant motion enable

    if (pin == InterruptPin::INT2) {
        ctrl8 |= 0x40;
    } else {
        ctrl8 &= ~0x40;
    }

    writeRegister(REG_CTRL8, ctrl8);
    configInterrupt(pin);

    return true;
}

void QMI8658::disableMotionDetection() {
    uint8_t ctrl8 = readRegister(REG_CTRL8);
    ctrl8 &= ~0x0E; // Clear any-motion (bit1), no-motion (bit2), sig-motion (bit3)
    writeRegister(REG_CTRL8, ctrl8);
}

uint8_t QMI8658::readMotionStatus() {
    return readRegister(REG_STATUS1);
}

// ============================================================================
// Pedometer
// ============================================================================

bool QMI8658::enablePedometer(InterruptPin pin) {
    // Configure pedometer parameters via CAL registers + CTRL9
    // CAL1_L/H: Sample rate configuration (match accel ODR)
    writeRegister(REG_CAL1_L, 0x02); // Low byte: sample rate divider
    writeRegister(REG_CAL1_H, 0x00); // High byte

    // CAL2_L/H: Step length (in cm, scaled)
    writeRegister(REG_CAL2_L, 0x50); // ~80 cm step length
    writeRegister(REG_CAL2_H, 0x00);

    // CAL3_L/H: Step detection threshold
    writeRegister(REG_CAL3_L, 0x14); // Moderate sensitivity
    writeRegister(REG_CAL3_H, 0x00);

    // CAL4_L/H: Step detection timing
    writeRegister(REG_CAL4_L, 0x0A);
    writeRegister(REG_CAL4_H, 0x00);

    if (!sendCtrl9Command(CTRL9_CMD_CONFIGURE_PEDOMETER)) {
        return false;
    }

    // Enable pedometer in CTRL8
    uint8_t ctrl8 = readRegister(REG_CTRL8);
    ctrl8 |= 0x10; // Bit 4: pedometer enable

    if (pin == InterruptPin::INT2) {
        ctrl8 |= 0x40;
    } else {
        ctrl8 &= ~0x40;
    }

    writeRegister(REG_CTRL8, ctrl8);
    configInterrupt(pin);

    return true;
}

void QMI8658::disablePedometer() {
    uint8_t ctrl8 = readRegister(REG_CTRL8);
    ctrl8 &= ~0x10; // Clear pedometer enable bit
    writeRegister(REG_CTRL8, ctrl8);
}

uint32_t QMI8658::readStepCount() {
    uint8_t buf[3];
    readRegisters(REG_STEP_CNT_L, buf, 3);
    return (uint32_t)buf[0] | ((uint32_t)buf[1] << 8) | ((uint32_t)buf[2] << 16);
}

void QMI8658::resetStepCount() {
    sendCtrl9Command(CTRL9_CMD_RESET_PEDOMETER);
}

// ============================================================================
// FIFO
// ============================================================================

bool QMI8658::configFIFO(FIFOMode mode, FIFOSize size) {
    if (mode == FIFOMode::BYPASS) {
        // Disable FIFO
        writeRegister(REG_FIFO_CTRL, 0x00);
        return true;
    }

    // FIFO_CTRL register: bits 1:0 = mode, bits 3:2 = size
    uint8_t ctrl = static_cast<uint8_t>(mode) |
                   (static_cast<uint8_t>(size) << 2);
    writeRegister(REG_FIFO_CTRL, ctrl);

    // Reset FIFO to start clean
    resetFIFO();

    return true;
}

void QMI8658::setFIFOWatermark(uint8_t threshold) {
    writeRegister(REG_FIFO_WTM_TH, threshold);
}

uint16_t QMI8658::readFIFOCount() {
    uint8_t buf[2];
    readRegisters(REG_FIFO_SMPL_CNT, buf, 2);
    // FIFO_SMPL_CNT is the count; FIFO_STATUS has overflow/watermark flags
    return (uint16_t)buf[0] | (((uint16_t)buf[1] & 0x03) << 8);
}

IMUData QMI8658::readFIFOSample() {
    IMUData data;

    // Enter FIFO read mode
    sendCtrl9Command(CTRL9_CMD_REQ_FIFO);

    // Read 12 bytes (6 accel + 6 gyro) from FIFO_DATA register
    uint8_t buf[12];
    for (uint8_t i = 0; i < 12; i++) {
        buf[i] = readRegister(REG_FIFO_DATA);
    }

    data.accel.rawX = (int16_t)(buf[0]  | ((uint16_t)buf[1] << 8));
    data.accel.rawY = (int16_t)(buf[2]  | ((uint16_t)buf[3] << 8));
    data.accel.rawZ = (int16_t)(buf[4]  | ((uint16_t)buf[5] << 8));
    data.gyro.rawX  = (int16_t)(buf[6]  | ((uint16_t)buf[7] << 8));
    data.gyro.rawY  = (int16_t)(buf[8]  | ((uint16_t)buf[9] << 8));
    data.gyro.rawZ  = (int16_t)(buf[10] | ((uint16_t)buf[11] << 8));

    data.accel.x = data.accel.rawX * _accelScale;
    data.accel.y = data.accel.rawY * _accelScale;
    data.accel.z = data.accel.rawZ * _accelScale;
    data.gyro.x  = data.gyro.rawX * _gyroScale;
    data.gyro.y  = data.gyro.rawY * _gyroScale;
    data.gyro.z  = data.gyro.rawZ * _gyroScale;

    return data;
}

uint16_t QMI8658::readFIFO(IMUData *buffer, uint16_t maxSamples) {
    uint16_t available = readFIFOCount();
    uint16_t toRead = (available < maxSamples) ? available : maxSamples;

    if (toRead == 0) return 0;

    // Enter FIFO read mode
    sendCtrl9Command(CTRL9_CMD_REQ_FIFO);

    for (uint16_t i = 0; i < toRead; i++) {
        uint8_t buf[12];
        for (uint8_t j = 0; j < 12; j++) {
            buf[j] = readRegister(REG_FIFO_DATA);
        }

        buffer[i].accel.rawX = (int16_t)(buf[0]  | ((uint16_t)buf[1] << 8));
        buffer[i].accel.rawY = (int16_t)(buf[2]  | ((uint16_t)buf[3] << 8));
        buffer[i].accel.rawZ = (int16_t)(buf[4]  | ((uint16_t)buf[5] << 8));
        buffer[i].gyro.rawX  = (int16_t)(buf[6]  | ((uint16_t)buf[7] << 8));
        buffer[i].gyro.rawY  = (int16_t)(buf[8]  | ((uint16_t)buf[9] << 8));
        buffer[i].gyro.rawZ  = (int16_t)(buf[10] | ((uint16_t)buf[11] << 8));

        buffer[i].accel.x = buffer[i].accel.rawX * _accelScale;
        buffer[i].accel.y = buffer[i].accel.rawY * _accelScale;
        buffer[i].accel.z = buffer[i].accel.rawZ * _accelScale;
        buffer[i].gyro.x  = buffer[i].gyro.rawX * _gyroScale;
        buffer[i].gyro.y  = buffer[i].gyro.rawY * _gyroScale;
        buffer[i].gyro.z  = buffer[i].gyro.rawZ * _gyroScale;
    }

    return toRead;
}

void QMI8658::resetFIFO() {
    sendCtrl9Command(CTRL9_CMD_RST_FIFO);
}

// ============================================================================
// Attitude Engine
// ============================================================================

bool QMI8658::enableAttitudeEngine() {
    // Set Attitude Engine mode via CTRL9
    // CAL1_L: AE mode (0x01 = 6-axis fusion, accel + gyro)
    writeRegister(REG_CAL1_L, 0x01);

    if (!sendCtrl9Command(CTRL9_CMD_AE_SET_MODE)) {
        return false;
    }

    // Enable AE in CTRL7 (bit 6)
    uint8_t ctrl7 = readRegister(REG_CTRL7);
    ctrl7 |= CTRL7_AE_EN;
    // Ensure accel and gyro are also enabled
    ctrl7 |= CTRL7_ACC_EN | CTRL7_GYR_EN;
    writeRegister(REG_CTRL7, ctrl7);

    delay(10);
    return true;
}

void QMI8658::disableAttitudeEngine() {
    uint8_t ctrl7 = readRegister(REG_CTRL7);
    ctrl7 &= ~CTRL7_AE_EN;
    writeRegister(REG_CTRL7, ctrl7);
}

AttitudeData QMI8658::readAttitude() {
    AttitudeData data;

    // Read quaternion: 8 bytes (4 x 16-bit) starting at DQW_L
    uint8_t qbuf[8];
    readRegisters(REG_DQW_L, qbuf, 8);

    int16_t rawW = (int16_t)(qbuf[0] | ((uint16_t)qbuf[1] << 8));
    int16_t rawX = (int16_t)(qbuf[2] | ((uint16_t)qbuf[3] << 8));
    int16_t rawY = (int16_t)(qbuf[4] | ((uint16_t)qbuf[5] << 8));
    int16_t rawZ = (int16_t)(qbuf[6] | ((uint16_t)qbuf[7] << 8));

    // Quaternion values are in Q14 fixed-point format
    data.quat.w = rawW / 16384.0f;
    data.quat.x = rawX / 16384.0f;
    data.quat.y = rawY / 16384.0f;
    data.quat.z = rawZ / 16384.0f;

    // Read delta velocity: 6 bytes (3 x 16-bit) starting at DVX_L
    uint8_t vbuf[6];
    readRegisters(REG_DVX_L, vbuf, 6);

    data.dVX = (int16_t)(vbuf[0] | ((uint16_t)vbuf[1] << 8)) / 16384.0f;
    data.dVY = (int16_t)(vbuf[2] | ((uint16_t)vbuf[3] << 8)) / 16384.0f;
    data.dVZ = (int16_t)(vbuf[4] | ((uint16_t)vbuf[5] << 8)) / 16384.0f;

    return data;
}

// ============================================================================
// Utility
// ============================================================================

void QMI8658::reset() {
    writeRegister(REG_RESET, QMI8658_RESET_COMMAND);
    delay(15); // Datasheet specifies ~15 ms for reset
}

bool QMI8658::selfTest() {
    // Save current CTRL7 state
    uint8_t ctrl7Save = readRegister(REG_CTRL7);

    // Disable all sensors first
    writeRegister(REG_CTRL7, 0x00);
    delay(10);

    // Enable accel only and wait for data
    writeRegister(REG_CTRL7, CTRL7_ACC_EN);
    delay(50);

    // Read baseline accel values
    AccelData baseline = readAccel();

    // Trigger accelerometer self-test via CTRL2 bit 7
    uint8_t ctrl2 = readRegister(REG_CTRL2);
    writeRegister(REG_CTRL2, ctrl2 | 0x80);
    delay(50);

    // Read self-test accel values
    AccelData selfTestData = readAccel();

    // Clear self-test bit
    writeRegister(REG_CTRL2, ctrl2);

    // Check that self-test produced a measurable deflection
    float diffX = (selfTestData.rawX - baseline.rawX) * _accelScale;
    float diffY = (selfTestData.rawY - baseline.rawY) * _accelScale;
    float diffZ = (selfTestData.rawZ - baseline.rawZ) * _accelScale;

    // Self-test should produce at least 0.1g deflection on at least one axis
    bool accelPass = (fabsf(diffX) > 0.1f) || (fabsf(diffY) > 0.1f) ||
                     (fabsf(diffZ) > 0.1f);

    // Restore original configuration
    writeRegister(REG_CTRL7, ctrl7Save);
    delay(10);

    if (!accelPass) {
        _error = 4;
        return false;
    }

    return true;
}

uint8_t QMI8658::whoAmI() {
    return readRegister(REG_WHO_AM_I);
}

uint8_t QMI8658::revisionID() {
    return readRegister(REG_REVISION_ID);
}

uint8_t QMI8658::getError() {
    return _error;
}

// ============================================================================
// Private: Register I/O
// ============================================================================

uint8_t QMI8658::readRegister(uint8_t reg) {
    _wire->beginTransmission(_address);
    _wire->write(reg);
    _wire->endTransmission(false);
    _wire->requestFrom(_address, (uint8_t)1);
    return _wire->read();
}

void QMI8658::readRegisters(uint8_t reg, uint8_t *buffer, uint8_t length) {
    _wire->beginTransmission(_address);
    _wire->write(reg);
    _wire->endTransmission(false);
    _wire->requestFrom(_address, length);
    for (uint8_t i = 0; i < length; i++) {
        buffer[i] = _wire->read();
    }
}

void QMI8658::writeRegister(uint8_t reg, uint8_t value) {
    _wire->beginTransmission(_address);
    _wire->write(reg);
    _wire->write(value);
    _wire->endTransmission();
}

// ============================================================================
// Private: CTRL9 Command Protocol
// ============================================================================

bool QMI8658::sendCtrl9Command(uint8_t cmd) {
    // Step 1: Write command to CTRL9
    writeRegister(REG_CTRL9, cmd);

    // Step 2: Poll STATUSINT bit 7 for command completion
    uint32_t start = millis();
    while (!(readRegister(REG_STATUSINT) & STATUSINT_CTRL9_DONE)) {
        if (millis() - start > 50) {
            _error = 2; // CTRL9 command timeout
            return false;
        }
        delayMicroseconds(200);
    }

    // Step 3: Acknowledge by writing 0x00 to CTRL9
    writeRegister(REG_CTRL9, CTRL9_CMD_ACK);

    // Step 4: Wait for CTRL9 done flag to clear
    start = millis();
    while (readRegister(REG_STATUSINT) & STATUSINT_CTRL9_DONE) {
        if (millis() - start > 50) {
            _error = 3; // CTRL9 ACK timeout
            return false;
        }
        delayMicroseconds(200);
    }

    return true;
}

// ============================================================================
// Private: Scale Conversion Helpers
// ============================================================================

void QMI8658::updateAccelScale() {
    _accelScale = rangeToG(_accelRange) / 32768.0f;
}

void QMI8658::updateGyroScale() {
    _gyroScale = rangeToDPS(_gyroRange) / 32768.0f;
}

float QMI8658::rangeToG(AccelRange range) {
    switch (range) {
        case AccelRange::RANGE_2G:  return 2.0f;
        case AccelRange::RANGE_4G:  return 4.0f;
        case AccelRange::RANGE_8G:  return 8.0f;
        case AccelRange::RANGE_16G: return 16.0f;
        default: return 8.0f;
    }
}

float QMI8658::rangeToDPS(GyroRange range) {
    // Enum values 0-7 correspond to 16, 32, 64, 128, 256, 512, 1024, 2048
    return 16.0f * (float)(1 << static_cast<uint8_t>(range));
}
