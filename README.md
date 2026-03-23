# QMI8658 Arduino Library

Arduino library for the **QMI8658** 6-axis inertial measurement unit (accelerometer + gyroscope) by QST Corporation.

## Features

- **Accelerometer**: configurable range (2G / 4G / 8G / 16G) and ODR (3 Hz to 8 kHz)
- **Gyroscope**: configurable range (16 to 2048 DPS) and ODR (31.25 Hz to 8 kHz)
- **Low-pass filter**: adjustable bandwidth for both sensors
- **Interrupt support**: configurable INT1/INT2 pins for data-ready and motion events
- **Tap detection**: single and double tap recognition
- **Motion detection**: any-motion, no-motion, and significant-motion
- **Pedometer**: hardware step counter
- **FIFO buffer**: up to 128 samples in FIFO or stream mode
- **Attitude Engine**: on-chip 6-axis sensor fusion with quaternion output (XKF3)
- **Temperature sensor**: built-in die temperature reading
- **Self-test**: built-in accelerometer self-test
- **Cross-platform**: works on AVR, ESP32, ESP8266, RP2040, STM32, nRF52, SAMD

## Quick Start

```cpp
#include <QMI8658.h>

QMI8658 imu;

void setup() {
    Serial.begin(115200);

    if (!imu.begin()) {
        Serial.println("QMI8658 not found!");
        while (1);
    }

    imu.configAccel(AccelRange::RANGE_8G, AccelODR::ODR_500HZ);
    imu.configGyro(GyroRange::RANGE_512DPS, GyroODR::ODR_500HZ);
}

void loop() {
    if (imu.dataReady()) {
        IMUData data = imu.readBoth();
        Serial.printf("Accel: %.3f %.3f %.3f g\n",
                      data.accel.x, data.accel.y, data.accel.z);
        Serial.printf("Gyro:  %.1f %.1f %.1f dps\n",
                      data.gyro.x, data.gyro.y, data.gyro.z);
    }
    delay(20);
}
```

## Wiring (I2C)

| QMI8658 Pin | Connection |
|-------------|------------|
| VCC | 3.3V |
| GND | GND |
| SDA | Board SDA |
| SCL | Board SCL |
| SA0 | GND (address 0x6A) or VCC (address 0x6B) |
| INT1 | GPIO (optional, for interrupts) |
| INT2 | GPIO (optional, for interrupts) |

### ESP32 / RP2040 Custom Pins

```cpp
imu.begin(5, 4); // SDA=GPIO5, SCL=GPIO4
```

### Alternate I2C Address

```cpp
QMI8658 imu(QMI8658_I2C_ADDRESS_HIGH); // 0x6B when SA0 is high
```

### Using Wire1

```cpp
QMI8658 imu(Wire1, QMI8658_I2C_ADDRESS_LOW);
```

## Examples

| Example | Description |
|---------|-------------|
| **BasicAccelGyro** | Read accelerometer and gyroscope data |
| **Temperature** | Read the built-in temperature sensor |
| **TapDetection** | Detect single and double taps |
| **Pedometer** | Count steps using the hardware pedometer |
| **MotionDetection** | Detect any-motion, no-motion, significant-motion |
| **AttitudeEngine** | On-chip sensor fusion with quaternion output |
| **FIFO_Reading** | Batch-read samples from the hardware FIFO |
| **InterruptDataReady** | Interrupt-driven data reading |
| **MadgwickFusion** | Software sensor fusion with Euler angle output |

## API Reference

### Initialization

```cpp
QMI8658 imu;                              // Default: Wire, address 0x6A
QMI8658 imu(0x6B);                        // Custom address
QMI8658 imu(Wire1, 0x6A);                 // Custom Wire bus

bool begin();                              // Initialize (default pins)
bool begin(int sda, int scl);             // Initialize with custom pins
```

### Configuration

```cpp
bool configAccel(AccelRange range, AccelODR odr);
bool configGyro(GyroRange range, GyroODR odr);
bool configLowPassFilter(LowPassFilter accelLPF, LowPassFilter gyroLPF);
void enableAccel(bool enable);
void enableGyro(bool enable);
void setClock(uint32_t clockHz);
```

### Data Reading

```cpp
bool      dataReady();          // Check for new data
AccelData readAccel();          // Read accelerometer (6 bytes)
GyroData  readGyro();           // Read gyroscope (6 bytes)
IMUData   readBoth();           // Read both in one burst (12 bytes)
float     readTemperature();    // Read temperature in Celsius
uint32_t  readTimestamp();      // Read 24-bit timestamp
```

### Advanced Features

```cpp
// Tap detection
bool    enableTapDetection(bool single, bool doubleTap, InterruptPin pin);
uint8_t readTapStatus();

// Motion detection
bool    enableAnyMotion(InterruptPin pin);
bool    enableNoMotion(InterruptPin pin);
bool    enableSignificantMotion(InterruptPin pin);
uint8_t readMotionStatus();

// Pedometer
bool     enablePedometer(InterruptPin pin);
uint32_t readStepCount();
void     resetStepCount();

// FIFO
bool     configFIFO(FIFOMode mode, FIFOSize size);
uint16_t readFIFOCount();
uint16_t readFIFO(IMUData *buffer, uint16_t maxSamples);

// Attitude Engine (on-chip sensor fusion)
bool         enableAttitudeEngine();
AttitudeData readAttitude();    // Returns quaternion + delta velocity
```

### Utility

```cpp
void    reset();         // Soft reset
bool    selfTest();      // Run built-in self-test
uint8_t whoAmI();        // Read device ID (expected: 0x05)
uint8_t getError();      // Get last error code
```

## Data Structures

```cpp
struct AccelData { int16_t rawX, rawY, rawZ; float x, y, z; };  // in g
struct GyroData  { int16_t rawX, rawY, rawZ; float x, y, z; };  // in dps
struct IMUData   { AccelData accel; GyroData gyro; };
struct Quaternion    { float w, x, y, z; };
struct AttitudeData  { Quaternion quat; float dVX, dVY, dVZ; };
```

## License

MIT License. See [LICENSE](LICENSE) for details.
