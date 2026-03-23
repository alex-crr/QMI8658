/**
 * BasicAccelGyro - QMI8658 Library Example
 *
 * Reads accelerometer and gyroscope data from the QMI8658 and prints
 * the values to the Serial Monitor.
 *
 * This is the simplest example to verify that your sensor is working.
 * At rest, you should see approximately:
 *   - Accel: ~0 g on X and Y, ~1 g (or -1 g) on Z (gravity)
 *   - Gyro:  ~0 dps on all axes
 *
 * Wiring (I2C):
 *   QMI8658 SDA -> Board SDA (or custom pin)
 *   QMI8658 SCL -> Board SCL (or custom pin)
 *   QMI8658 VCC -> 3.3V
 *   QMI8658 GND -> GND
 *
 * Default I2C address: 0x6A (SA0 to GND) or 0x6B (SA0 to VCC)
 */

#include <QMI8658.h>

// Create sensor instance with default address (0x6A)
QMI8658 imu;

// Uncomment for alternate address:
// QMI8658 imu(QMI8658_I2C_ADDRESS_HIGH);

// Uncomment to use a different Wire bus:
// QMI8658 imu(Wire1, QMI8658_I2C_ADDRESS_LOW);

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10); // Wait for Serial on USB-native boards

    Serial.println("QMI8658 Basic Accel + Gyro Example");
    Serial.println("===================================");

    // Initialize the sensor
    // For ESP32 or RP2040 with custom pins, use: imu.begin(SDA_PIN, SCL_PIN);
    if (!imu.begin()) {
        Serial.print("QMI8658 not found! WHO_AM_I = 0x");
        Serial.println(imu.whoAmI(), HEX);
        while (1) delay(100);
    }

    Serial.print("WHO_AM_I: 0x");
    Serial.println(imu.whoAmI(), HEX);
    Serial.print("Revision: 0x");
    Serial.println(imu.revisionID(), HEX);

    // Configure accelerometer: +/- 8g range, 500 Hz output rate
    imu.configAccel(AccelRange::RANGE_8G, AccelODR::ODR_500HZ);

    // Configure gyroscope: +/- 512 dps range, 500 Hz output rate
    imu.configGyro(GyroRange::RANGE_512DPS, GyroODR::ODR_500HZ);

    // Optional: set I2C to Fast Mode (400 kHz) for higher throughput
    imu.setClock(400000);

    Serial.println("Sensor initialized. Reading data...\n");
}

void loop() {
    // Check if new data is available
    if (imu.dataReady()) {
        // Read both sensors in a single efficient 12-byte burst
        IMUData data = imu.readBoth();

        // Print accelerometer values in g
        Serial.print("Accel [g]:  X=");
        Serial.print(data.accel.x, 3);
        Serial.print("  Y=");
        Serial.print(data.accel.y, 3);
        Serial.print("  Z=");
        Serial.println(data.accel.z, 3);

        // Print gyroscope values in degrees per second
        Serial.print("Gyro [dps]: X=");
        Serial.print(data.gyro.x, 1);
        Serial.print("  Y=");
        Serial.print(data.gyro.y, 1);
        Serial.print("  Z=");
        Serial.println(data.gyro.z, 1);

        Serial.println();
    }

    delay(20); // ~50 Hz print rate
}
