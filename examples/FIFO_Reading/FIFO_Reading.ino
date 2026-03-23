/**
 * FIFO_Reading - QMI8658 Library Example
 *
 * Demonstrates the QMI8658's hardware FIFO buffer for efficient batch
 * data reading. Instead of reading one sample at a time, the FIFO
 * collects multiple samples that can be read in a burst.
 *
 * This is useful for:
 *   - Reducing I2C bus traffic (read many samples at once)
 *   - Allowing the MCU to sleep between reads
 *   - Ensuring no data is lost at high sample rates
 *
 * FIFO modes:
 *   - FIFO: Stops collecting when full (no data loss, but may miss new data)
 *   - Stream: Overwrites oldest data when full (always has newest data)
 *
 * Wiring: Same as BasicAccelGyro example.
 */

#include <QMI8658.h>

QMI8658 imu;

// Buffer to hold FIFO samples (max 128 for SAMPLES_128)
IMUData fifoBuffer[64];

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    Serial.println("QMI8658 FIFO Reading Example");
    Serial.println("============================");

    if (!imu.begin()) {
        Serial.println("QMI8658 not found!");
        while (1) delay(100);
    }

    // Configure sensors
    imu.configAccel(AccelRange::RANGE_8G, AccelODR::ODR_500HZ);
    imu.configGyro(GyroRange::RANGE_512DPS, GyroODR::ODR_500HZ);

    // Configure FIFO in stream mode with 64-sample buffer
    if (!imu.configFIFO(FIFOMode::STREAM, FIFOSize::SAMPLES_64)) {
        Serial.println("Failed to configure FIFO!");
        while (1) delay(100);
    }

    // Set watermark to trigger at 32 samples (half full)
    imu.setFIFOWatermark(32);

    Serial.println("FIFO configured: Stream mode, 64 samples, watermark=32");
    Serial.println("Collecting data...\n");
}

void loop() {
    // Check how many samples are in the FIFO
    uint16_t count = imu.readFIFOCount();

    if (count >= 32) { // Wait for watermark level
        Serial.print("FIFO has ");
        Serial.print(count);
        Serial.println(" samples. Reading...");

        // Burst-read all available samples
        uint16_t read = imu.readFIFO(fifoBuffer, 64);

        Serial.print("Read ");
        Serial.print(read);
        Serial.println(" samples:");

        // Print first and last sample to show the range
        if (read > 0) {
            Serial.print("  First: Accel=(");
            Serial.print(fifoBuffer[0].accel.x, 3);
            Serial.print(", ");
            Serial.print(fifoBuffer[0].accel.y, 3);
            Serial.print(", ");
            Serial.print(fifoBuffer[0].accel.z, 3);
            Serial.print(") g  Gyro=(");
            Serial.print(fifoBuffer[0].gyro.x, 1);
            Serial.print(", ");
            Serial.print(fifoBuffer[0].gyro.y, 1);
            Serial.print(", ");
            Serial.print(fifoBuffer[0].gyro.z, 1);
            Serial.println(") dps");

            Serial.print("  Last:  Accel=(");
            Serial.print(fifoBuffer[read - 1].accel.x, 3);
            Serial.print(", ");
            Serial.print(fifoBuffer[read - 1].accel.y, 3);
            Serial.print(", ");
            Serial.print(fifoBuffer[read - 1].accel.z, 3);
            Serial.print(") g  Gyro=(");
            Serial.print(fifoBuffer[read - 1].gyro.x, 1);
            Serial.print(", ");
            Serial.print(fifoBuffer[read - 1].gyro.y, 1);
            Serial.print(", ");
            Serial.print(fifoBuffer[read - 1].gyro.z, 1);
            Serial.println(") dps");
        }

        Serial.println();
    }

    delay(50); // Check FIFO level periodically
}
