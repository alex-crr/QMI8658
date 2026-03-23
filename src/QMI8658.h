/*!
 * @file QMI8658.h
 *
 * @mainpage QMI8658 6-Axis IMU Arduino Library
 *
 * @section intro_sec Introduction
 *
 * Arduino library for the QMI8658 6-axis inertial measurement unit
 * (accelerometer + gyroscope) by QST Corporation. Communicates over I2C.
 *
 * Features:
 * - Configurable accelerometer (2G-16G) and gyroscope (16-2048 DPS)
 * - Output data rates from 3 Hz to 8000 Hz
 * - Low-pass filter configuration
 * - Hardware interrupt support (INT1/INT2)
 * - Tap detection (single and double)
 * - Motion detection (any-motion, no-motion, significant-motion)
 * - Pedometer (step counter)
 * - FIFO buffer (up to 128 samples, FIFO or stream mode)
 * - On-chip Attitude Engine (quaternion sensor fusion)
 * - Built-in temperature sensor
 * - Self-test capability
 *
 * @section author Author
 *
 * Written with reference to the QMI8658A datasheet Rev A.
 *
 * @section license License
 *
 * MIT license, all text above must be included in any redistribution.
 */

#ifndef QMI8658_H
#define QMI8658_H

#include <Arduino.h>
#include <Wire.h>

// ============================================================================
// I2C Addresses
// ============================================================================

/** I2C address when SA0/SDO pin is connected to GND. */
#define QMI8658_I2C_ADDRESS_LOW  0x6A

/** I2C address when SA0/SDO pin is connected to VCC. */
#define QMI8658_I2C_ADDRESS_HIGH 0x6B

/** Expected value of the WHO_AM_I register. */
#define QMI8658_WHO_AM_I_DEFAULT 0x05

/** Command byte to trigger soft reset. */
#define QMI8658_RESET_COMMAND    0xB0

// ============================================================================
// Enumerations
// ============================================================================

/** Accelerometer full-scale range selection. */
enum class AccelRange : uint8_t {
    RANGE_2G  = 0b000, ///< +/- 2g
    RANGE_4G  = 0b001, ///< +/- 4g
    RANGE_8G  = 0b010, ///< +/- 8g
    RANGE_16G = 0b011  ///< +/- 16g
};

/** Gyroscope full-scale range selection. */
enum class GyroRange : uint8_t {
    RANGE_16DPS   = 0b000, ///< +/- 16 degrees per second
    RANGE_32DPS   = 0b001, ///< +/- 32 degrees per second
    RANGE_64DPS   = 0b010, ///< +/- 64 degrees per second
    RANGE_128DPS  = 0b011, ///< +/- 128 degrees per second
    RANGE_256DPS  = 0b100, ///< +/- 256 degrees per second
    RANGE_512DPS  = 0b101, ///< +/- 512 degrees per second
    RANGE_1024DPS = 0b110, ///< +/- 1024 degrees per second
    RANGE_2048DPS = 0b111  ///< +/- 2048 degrees per second
};

/** Accelerometer output data rate selection. */
enum class AccelODR : uint8_t {
    ODR_8000HZ   = 0x00, ///< 8000 Hz (normal mode)
    ODR_4000HZ   = 0x01, ///< 4000 Hz (normal mode)
    ODR_2000HZ   = 0x02, ///< 2000 Hz (normal mode)
    ODR_1000HZ   = 0x03, ///< 1000 Hz (normal mode)
    ODR_500HZ    = 0x04, ///< 500 Hz (normal mode)
    ODR_250HZ    = 0x05, ///< 250 Hz (normal mode)
    ODR_125HZ    = 0x06, ///< 125 Hz (normal mode)
    ODR_62_5HZ   = 0x07, ///< 62.5 Hz (normal mode)
    ODR_31_25HZ  = 0x08, ///< 31.25 Hz (normal mode)
    ODR_LP_128HZ = 0x0C, ///< 128 Hz (low power mode)
    ODR_LP_21HZ  = 0x0D, ///< 21 Hz (low power mode)
    ODR_LP_11HZ  = 0x0E, ///< 11 Hz (low power mode)
    ODR_LP_3HZ   = 0x0F  ///< 3 Hz (low power mode)
};

/** Gyroscope output data rate selection. */
enum class GyroODR : uint8_t {
    ODR_8000HZ  = 0x00, ///< 8000 Hz
    ODR_4000HZ  = 0x01, ///< 4000 Hz
    ODR_2000HZ  = 0x02, ///< 2000 Hz
    ODR_1000HZ  = 0x03, ///< 1000 Hz
    ODR_500HZ   = 0x04, ///< 500 Hz
    ODR_250HZ   = 0x05, ///< 250 Hz
    ODR_125HZ   = 0x06, ///< 125 Hz
    ODR_62_5HZ  = 0x07, ///< 62.5 Hz
    ODR_31_25HZ = 0x08  ///< 31.25 Hz
};

/**
 * Low-pass filter bandwidth as a percentage of the output data rate.
 * Applies to both accelerometer and gyroscope independently.
 */
enum class LowPassFilter : uint8_t {
    DISABLED     = 0x00, ///< Low-pass filter disabled
    LPF_2_62PCT = 0x01, ///< 2.62% of ODR
    LPF_3_59PCT = 0x03, ///< 3.59% of ODR
    LPF_5_32PCT = 0x05, ///< 5.32% of ODR
    LPF_14PCT   = 0x07  ///< 14% of ODR
};

/** FIFO operating mode. */
enum class FIFOMode : uint8_t {
    BYPASS = 0x00, ///< FIFO disabled
    FIFO   = 0x01, ///< FIFO mode: stops collecting when full
    STREAM = 0x02  ///< Stream mode: overwrites oldest data when full
};

/** FIFO buffer size in samples. */
enum class FIFOSize : uint8_t {
    SAMPLES_16  = 0x00, ///< 16 samples
    SAMPLES_32  = 0x01, ///< 32 samples
    SAMPLES_64  = 0x02, ///< 64 samples
    SAMPLES_128 = 0x03  ///< 128 samples
};

/** Interrupt output pin selection. */
enum class InterruptPin : uint8_t {
    INT1 = 0, ///< Interrupt pin 1
    INT2 = 1  ///< Interrupt pin 2
};

// ============================================================================
// Data Structures
// ============================================================================

/** Accelerometer measurement data. */
struct AccelData {
    int16_t rawX; ///< Raw X-axis value (signed 16-bit)
    int16_t rawY; ///< Raw Y-axis value (signed 16-bit)
    int16_t rawZ; ///< Raw Z-axis value (signed 16-bit)
    float x;      ///< X-axis acceleration in g
    float y;      ///< Y-axis acceleration in g
    float z;      ///< Z-axis acceleration in g
};

/** Gyroscope measurement data. */
struct GyroData {
    int16_t rawX; ///< Raw X-axis value (signed 16-bit)
    int16_t rawY; ///< Raw Y-axis value (signed 16-bit)
    int16_t rawZ; ///< Raw Z-axis value (signed 16-bit)
    float x;      ///< X-axis angular rate in degrees per second
    float y;      ///< Y-axis angular rate in degrees per second
    float z;      ///< Z-axis angular rate in degrees per second
};

/** Combined accelerometer and gyroscope data from a single sample. */
struct IMUData {
    AccelData accel; ///< Accelerometer readings
    GyroData gyro;   ///< Gyroscope readings
};

/** Quaternion orientation representation. */
struct Quaternion {
    float w; ///< Scalar component
    float x; ///< X vector component
    float y; ///< Y vector component
    float z; ///< Z vector component
};

/** Attitude Engine output: quaternion orientation and delta velocity. */
struct AttitudeData {
    Quaternion quat; ///< Orientation as a unit quaternion
    float dVX;       ///< Delta velocity X (m/s)
    float dVY;       ///< Delta velocity Y (m/s)
    float dVZ;       ///< Delta velocity Z (m/s)
};

// ============================================================================
// QMI8658 Class
// ============================================================================

/**
 * @brief Driver for the QMI8658 6-axis IMU (accelerometer + gyroscope).
 *
 * Provides a high-level interface for configuring the sensor, reading
 * measurement data, and using advanced features such as tap detection,
 * motion detection, pedometer, FIFO, and the on-chip Attitude Engine.
 *
 * Basic usage:
 * @code
 * QMI8658 imu;
 *
 * void setup() {
 *     Serial.begin(115200);
 *     if (!imu.begin()) {
 *         Serial.println("QMI8658 not found!");
 *         while (1);
 *     }
 *     imu.configAccel(AccelRange::RANGE_8G, AccelODR::ODR_500HZ);
 *     imu.configGyro(GyroRange::RANGE_512DPS, GyroODR::ODR_500HZ);
 * }
 *
 * void loop() {
 *     if (imu.dataReady()) {
 *         IMUData data = imu.readBoth();
 *         Serial.printf("Accel: %.3f %.3f %.3f g\n",
 *                       data.accel.x, data.accel.y, data.accel.z);
 *     }
 * }
 * @endcode
 */
class QMI8658 {
public:
    // ---- Construction & Initialization ----

    /**
     * @brief Construct with default I2C address (0x6A) on the default Wire bus.
     */
    QMI8658();

    /**
     * @brief Construct with a specific I2C address.
     * @param address I2C address: QMI8658_I2C_ADDRESS_LOW (0x6A) or
     *                QMI8658_I2C_ADDRESS_HIGH (0x6B)
     */
    explicit QMI8658(uint8_t address);

    /**
     * @brief Construct with a specific Wire instance and I2C address.
     * @param wire Reference to a TwoWire instance (Wire, Wire1, etc.)
     * @param address I2C address (default: 0x6A)
     */
    QMI8658(TwoWire &wire, uint8_t address = QMI8658_I2C_ADDRESS_LOW);

    /**
     * @brief Initialize the sensor on the I2C bus.
     *
     * Calls Wire.begin(), verifies the WHO_AM_I register, performs a soft
     * reset, and applies default configuration (8G accel, 512 DPS gyro,
     * 500 Hz ODR, both sensors enabled).
     *
     * @return true if the sensor was found and initialized successfully
     */
    bool begin();

    /**
     * @brief Initialize with custom SDA and SCL pins.
     *
     * Useful for ESP32, RP2040, STM32, and other platforms that support
     * remappable I2C pins. On platforms that do not support custom pins
     * (e.g., AVR), the pin arguments are ignored and default pins are used.
     *
     * @param sda SDA pin number
     * @param scl SCL pin number
     * @return true if the sensor was found and initialized successfully
     */
    bool begin(int sda, int scl);

    // ---- Configuration ----

    /**
     * @brief Configure the accelerometer full-scale range and output data rate.
     * @param range Full-scale range (2G, 4G, 8G, or 16G)
     * @param odr Output data rate
     * @return true on success
     */
    bool configAccel(AccelRange range, AccelODR odr);

    /**
     * @brief Configure the gyroscope full-scale range and output data rate.
     * @param range Full-scale range (16 to 2048 DPS)
     * @param odr Output data rate
     * @return true on success
     */
    bool configGyro(GyroRange range, GyroODR odr);

    /**
     * @brief Configure low-pass filter bandwidth for accelerometer and gyroscope.
     * @param accelLPF Low-pass filter setting for accelerometer
     * @param gyroLPF Low-pass filter setting for gyroscope
     * @return true on success
     */
    bool configLowPassFilter(LowPassFilter accelLPF, LowPassFilter gyroLPF);

    /**
     * @brief Enable or disable the accelerometer.
     * @param enable true to enable, false to disable
     */
    void enableAccel(bool enable);

    /**
     * @brief Enable or disable the gyroscope.
     * @param enable true to enable, false to disable
     */
    void enableGyro(bool enable);

    /**
     * @brief Enable or disable synchronized sample mode.
     *
     * When enabled, accelerometer and gyroscope data are sampled
     * at the same time instant and locked together.
     *
     * @param enable true to enable sync sampling
     */
    void enableSyncSample(bool enable);

    /**
     * @brief Set the I2C clock frequency.
     * @param clockHz Clock frequency in Hz (e.g., 400000 for 400 kHz)
     */
    void setClock(uint32_t clockHz);

    // ---- Basic Data Reading ----

    /**
     * @brief Check if new sensor data is available.
     *
     * Reads STATUS0 register to check for accel and/or gyro data ready flags.
     *
     * @return true if new data is ready to read
     */
    bool dataReady();

    /**
     * @brief Read accelerometer data.
     *
     * Returns both raw 16-bit values and converted values in g.
     * Uses a 6-byte burst read for efficiency.
     *
     * @return AccelData struct with raw and converted values
     */
    AccelData readAccel();

    /**
     * @brief Read gyroscope data.
     *
     * Returns both raw 16-bit values and converted values in degrees/second.
     * Uses a 6-byte burst read for efficiency.
     *
     * @return GyroData struct with raw and converted values
     */
    GyroData readGyro();

    /**
     * @brief Read both accelerometer and gyroscope data in a single burst.
     *
     * This is the most efficient way to read both sensors, using a single
     * 12-byte I2C transaction. Recommended for high data rate applications.
     *
     * @return IMUData struct containing both accel and gyro readings
     */
    IMUData readBoth();

    /**
     * @brief Read the internal temperature sensor.
     * @return Temperature in degrees Celsius
     */
    float readTemperature();

    /**
     * @brief Read the 24-bit sample timestamp counter.
     *
     * The timestamp increments at a rate determined by the sensor ODR.
     * Useful for precise timing of samples.
     *
     * @return 24-bit timestamp value
     */
    uint32_t readTimestamp();

    // ---- Interrupt Configuration ----

    /**
     * @brief Configure an interrupt output pin.
     * @param pin Which interrupt pin to configure (INT1 or INT2)
     * @param activeLow true for active-low output (default), false for active-high
     * @param pushPull true for push-pull output (default), false for open-drain
     */
    void configInterrupt(InterruptPin pin, bool activeLow = true,
                         bool pushPull = true);

    /**
     * @brief Route accelerometer data-ready signal to an interrupt pin.
     * @param pin Interrupt pin to use (INT1 or INT2)
     */
    void enableAccelDataReadyInt(InterruptPin pin);

    /**
     * @brief Route gyroscope data-ready signal to an interrupt pin.
     * @param pin Interrupt pin to use (INT1 or INT2)
     */
    void enableGyroDataReadyInt(InterruptPin pin);

    /**
     * @brief Disable all data-ready interrupt outputs.
     */
    void disableDataReadyInt();

    // ---- Tap Detection ----

    /**
     * @brief Enable tap detection.
     *
     * Configures the sensor to detect single and/or double taps.
     * The tap event is signalled via the selected interrupt pin and
     * can be read with readTapStatus().
     *
     * @param singleTap Enable single-tap detection
     * @param doubleTap Enable double-tap detection
     * @param pin Interrupt pin for tap events
     * @return true on success
     */
    bool enableTapDetection(bool singleTap = true, bool doubleTap = true,
                            InterruptPin pin = InterruptPin::INT1);

    /**
     * @brief Disable tap detection.
     */
    void disableTapDetection();

    /**
     * @brief Read tap detection status.
     *
     * The returned byte indicates which tap events have occurred:
     * - Bit 0: Single tap detected on positive X
     * - Bit 1: Single tap detected on negative X
     * - Bit 2: Single tap detected on positive Y
     * - Bit 3: Single tap detected on negative Y
     * - Bit 4: Single tap detected on positive Z
     * - Bit 5: Single tap detected on negative Z
     * - Bit 6: Reserved
     * - Bit 7: Double tap detected
     *
     * @return Tap status byte
     */
    uint8_t readTapStatus();

    // ---- Motion Detection ----

    /**
     * @brief Enable any-motion detection.
     *
     * Triggers when the sensor detects motion exceeding a threshold.
     *
     * @param pin Interrupt pin for motion events
     * @return true on success
     */
    bool enableAnyMotion(InterruptPin pin = InterruptPin::INT1);

    /**
     * @brief Enable no-motion detection.
     *
     * Triggers when the sensor remains stationary for a configured duration.
     *
     * @param pin Interrupt pin for no-motion events
     * @return true on success
     */
    bool enableNoMotion(InterruptPin pin = InterruptPin::INT1);

    /**
     * @brief Enable significant-motion detection.
     *
     * Triggers on significant motion (e.g., walking, running) while
     * filtering out minor vibrations.
     *
     * @param pin Interrupt pin for significant-motion events
     * @return true on success
     */
    bool enableSignificantMotion(InterruptPin pin = InterruptPin::INT1);

    /**
     * @brief Disable all motion detection features.
     */
    void disableMotionDetection();

    /**
     * @brief Read motion event status from STATUS1 register.
     *
     * Bit meanings:
     * - Bit 0: Significant motion detected
     * - Bit 1: No-motion detected
     * - Bit 2: Any-motion detected
     * - Bit 6: Tap detected (see readTapStatus() for details)
     * - Bit 7: Pedometer step detected
     *
     * @return Motion event status byte
     */
    uint8_t readMotionStatus();

    // ---- Pedometer ----

    /**
     * @brief Enable the pedometer (step counter).
     * @param pin Interrupt pin for step events
     * @return true on success
     */
    bool enablePedometer(InterruptPin pin = InterruptPin::INT1);

    /**
     * @brief Disable the pedometer.
     */
    void disablePedometer();

    /**
     * @brief Read the current step count.
     * @return Number of steps counted since last reset
     */
    uint32_t readStepCount();

    /**
     * @brief Reset the step counter to zero.
     */
    void resetStepCount();

    // ---- FIFO ----

    /**
     * @brief Configure the FIFO buffer.
     * @param mode FIFO mode (BYPASS to disable, FIFO or STREAM)
     * @param size FIFO buffer size in samples
     * @return true on success
     */
    bool configFIFO(FIFOMode mode, FIFOSize size = FIFOSize::SAMPLES_128);

    /**
     * @brief Set the FIFO watermark threshold.
     *
     * An interrupt can be generated when the sample count reaches this level.
     *
     * @param threshold Number of samples for the watermark (1-128)
     */
    void setFIFOWatermark(uint8_t threshold);

    /**
     * @brief Get the number of samples currently stored in the FIFO.
     * @return Number of unread FIFO samples
     */
    uint16_t readFIFOCount();

    /**
     * @brief Read a single sample from the FIFO.
     *
     * Call readFIFOCount() first to verify data is available.
     *
     * @return IMUData for one FIFO sample
     */
    IMUData readFIFOSample();

    /**
     * @brief Read multiple samples from the FIFO into a buffer.
     * @param buffer Pointer to an array of IMUData to fill
     * @param maxSamples Maximum number of samples to read (buffer size)
     * @return Number of samples actually read
     */
    uint16_t readFIFO(IMUData *buffer, uint16_t maxSamples);

    /**
     * @brief Reset the FIFO, discarding all buffered data.
     */
    void resetFIFO();

    // ---- Attitude Engine ----

    /**
     * @brief Enable the on-chip Attitude Engine (sensor fusion).
     *
     * The Attitude Engine uses an Extended Kalman Filter (XKF3) to fuse
     * accelerometer and gyroscope data into a quaternion orientation.
     * Both accelerometer and gyroscope must be enabled before calling this.
     *
     * @return true on success
     */
    bool enableAttitudeEngine();

    /**
     * @brief Disable the Attitude Engine.
     */
    void disableAttitudeEngine();

    /**
     * @brief Read Attitude Engine output.
     *
     * Returns the current orientation as a quaternion and the delta
     * velocity vector.
     *
     * @return AttitudeData with quaternion and delta velocity
     */
    AttitudeData readAttitude();

    // ---- Utility ----

    /**
     * @brief Perform a soft reset of the sensor.
     *
     * All registers return to default values. The sensor takes
     * approximately 15 ms to restart after reset.
     */
    void reset();

    /**
     * @brief Run the built-in self-test.
     *
     * Exercises the accelerometer and gyroscope self-test mechanisms
     * and checks the results against expected thresholds.
     *
     * @return true if self-test passed
     */
    bool selfTest();

    /**
     * @brief Read the WHO_AM_I register.
     * @return Device ID (expected: 0x05)
     */
    uint8_t whoAmI();

    /**
     * @brief Read the revision ID register.
     * @return Revision ID byte
     */
    uint8_t revisionID();

    /**
     * @brief Get the last error code.
     *
     * Error codes:
     * - 0: No error
     * - 1: WHO_AM_I mismatch (sensor not found)
     * - 2: CTRL9 command timeout
     * - 3: CTRL9 acknowledge timeout
     * - 4: Self-test failure
     *
     * @return Error code byte
     */
    uint8_t getError();

private:
    // ---- Register Addresses ----
    static constexpr uint8_t REG_WHO_AM_I      = 0x00;
    static constexpr uint8_t REG_REVISION_ID   = 0x01;
    static constexpr uint8_t REG_CTRL1         = 0x02;
    static constexpr uint8_t REG_CTRL2         = 0x03;
    static constexpr uint8_t REG_CTRL3         = 0x04;
    static constexpr uint8_t REG_CTRL5         = 0x06;
    static constexpr uint8_t REG_CTRL6         = 0x07;
    static constexpr uint8_t REG_CTRL7         = 0x08;
    static constexpr uint8_t REG_CTRL8         = 0x09;
    static constexpr uint8_t REG_CTRL9         = 0x0A;
    static constexpr uint8_t REG_CAL1_L        = 0x0B;
    static constexpr uint8_t REG_CAL1_H        = 0x0C;
    static constexpr uint8_t REG_CAL2_L        = 0x0D;
    static constexpr uint8_t REG_CAL2_H        = 0x0E;
    static constexpr uint8_t REG_CAL3_L        = 0x0F;
    static constexpr uint8_t REG_CAL3_H        = 0x10;
    static constexpr uint8_t REG_CAL4_L        = 0x11;
    static constexpr uint8_t REG_CAL4_H        = 0x12;
    static constexpr uint8_t REG_FIFO_WTM_TH   = 0x13;
    static constexpr uint8_t REG_FIFO_CTRL     = 0x14;
    static constexpr uint8_t REG_FIFO_SMPL_CNT = 0x15;
    static constexpr uint8_t REG_FIFO_STATUS   = 0x16;
    static constexpr uint8_t REG_FIFO_DATA     = 0x17;
    static constexpr uint8_t REG_STATUSINT     = 0x2D;
    static constexpr uint8_t REG_STATUS0       = 0x2E;
    static constexpr uint8_t REG_STATUS1       = 0x2F;
    static constexpr uint8_t REG_TIMESTAMP_L   = 0x30;
    static constexpr uint8_t REG_TIMESTAMP_M   = 0x31;
    static constexpr uint8_t REG_TIMESTAMP_H   = 0x32;
    static constexpr uint8_t REG_TEMP_L        = 0x33;
    static constexpr uint8_t REG_TEMP_H        = 0x34;
    static constexpr uint8_t REG_AX_L          = 0x35;
    static constexpr uint8_t REG_GX_L          = 0x3B;
    static constexpr uint8_t REG_DQW_L         = 0x49;
    static constexpr uint8_t REG_DVX_L         = 0x51;
    static constexpr uint8_t REG_TAP_STATUS    = 0x59;
    static constexpr uint8_t REG_STEP_CNT_L    = 0x5A;
    static constexpr uint8_t REG_STEP_CNT_M    = 0x5B;
    static constexpr uint8_t REG_STEP_CNT_H    = 0x5C;
    static constexpr uint8_t REG_RESET         = 0x60;

    // ---- CTRL9 Command Codes ----
    static constexpr uint8_t CTRL9_CMD_ACK               = 0x00;
    static constexpr uint8_t CTRL9_CMD_RST_FIFO          = 0x04;
    static constexpr uint8_t CTRL9_CMD_REQ_FIFO          = 0x05;
    static constexpr uint8_t CTRL9_CMD_WRITE_WOM_SETTING = 0x08;
    static constexpr uint8_t CTRL9_CMD_ACCEL_HOST_DELTA_OFFSET = 0x09;
    static constexpr uint8_t CTRL9_CMD_GYRO_HOST_DELTA_OFFSET  = 0x0A;
    static constexpr uint8_t CTRL9_CMD_CONFIGURE_TAP     = 0x0C;
    static constexpr uint8_t CTRL9_CMD_CONFIGURE_PEDOMETER = 0x0D;
    static constexpr uint8_t CTRL9_CMD_CONFIGURE_MOTION  = 0x0E;
    static constexpr uint8_t CTRL9_CMD_RESET_PEDOMETER   = 0x0F;
    static constexpr uint8_t CTRL9_CMD_COPY_USID         = 0x10;
    static constexpr uint8_t CTRL9_CMD_SET_RPU           = 0x11;
    static constexpr uint8_t CTRL9_CMD_AE_SET_MODE       = 0x12;

    // ---- CTRL7 Bit Masks ----
    static constexpr uint8_t CTRL7_ACC_EN    = 0x01;
    static constexpr uint8_t CTRL7_GYR_EN    = 0x02;
    static constexpr uint8_t CTRL7_GYR_SNOOZE = 0x10;
    static constexpr uint8_t CTRL7_AE_EN     = 0x40;
    static constexpr uint8_t CTRL7_SYNC_EN   = 0x80;

    // ---- STATUS0 Bit Masks ----
    static constexpr uint8_t STATUS0_ACC_RDY = 0x01;
    static constexpr uint8_t STATUS0_GYR_RDY = 0x02;

    // ---- STATUSINT Bit Masks ----
    static constexpr uint8_t STATUSINT_CTRL9_DONE = 0x80;
    static constexpr uint8_t STATUSINT_AVAIL      = 0x01;
    static constexpr uint8_t STATUSINT_LOCKED     = 0x02;

    // ---- Instance Variables ----
    TwoWire *_wire;
    uint8_t _address;
    uint8_t _error;

    AccelRange _accelRange;
    GyroRange  _gyroRange;
    float      _accelScale; // Precomputed: range_in_g / 32768.0
    float      _gyroScale;  // Precomputed: range_in_dps / 32768.0

    // ---- Private Methods ----

    /** Shared initialization logic used by both begin() overloads. */
    bool _init();

    /** Read a single register. */
    uint8_t readRegister(uint8_t reg);

    /** Read multiple consecutive registers into a buffer. */
    void readRegisters(uint8_t reg, uint8_t *buffer, uint8_t length);

    /** Write a single register. */
    void writeRegister(uint8_t reg, uint8_t value);

    /**
     * Execute a CTRL9 host command and wait for acknowledgment.
     * @param cmd CTRL9 command byte
     * @return true if command completed successfully
     */
    bool sendCtrl9Command(uint8_t cmd);

    /** Recalculate _accelScale from _accelRange. */
    void updateAccelScale();

    /** Recalculate _gyroScale from _gyroRange. */
    void updateGyroScale();

    /** Convert AccelRange enum to full-scale value in g. */
    static float rangeToG(AccelRange range);

    /** Convert GyroRange enum to full-scale value in degrees/second. */
    static float rangeToDPS(GyroRange range);
};

#endif // QMI8658_H
