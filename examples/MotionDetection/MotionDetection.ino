/**
 * MotionDetection - QMI8658 Library Example
 *
 * Demonstrates the QMI8658's any-motion, no-motion, and significant-motion
 * detection features.
 *
 * - Any-motion: triggers when acceleration exceeds a threshold on any axis
 * - No-motion: triggers when the sensor stays still for a configured time
 * - Significant-motion: triggers on sustained motion (walking, driving)
 *   while ignoring brief vibrations
 *
 * How to test:
 *   - Place the sensor on a table and wait for "No motion" detection
 *   - Pick it up and move it to trigger "Any motion"
 *   - Walk with it attached to trigger "Significant motion"
 *
 * Wiring: Same as BasicAccelGyro example.
 */

#include <QMI8658.h>

QMI8658 imu;

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    Serial.println("QMI8658 Motion Detection Example");
    Serial.println("=================================");

    if (!imu.begin()) {
        Serial.println("QMI8658 not found!");
        while (1) delay(100);
    }

    // Configure accelerometer
    imu.configAccel(AccelRange::RANGE_8G, AccelODR::ODR_500HZ);

    // Enable any-motion detection
    if (!imu.enableAnyMotion(InterruptPin::INT1)) {
        Serial.println("Failed to enable any-motion detection!");
        while (1) delay(100);
    }
    Serial.println("Any-motion detection: ENABLED");

    // Enable no-motion detection
    if (!imu.enableNoMotion(InterruptPin::INT1)) {
        Serial.println("Failed to enable no-motion detection!");
        while (1) delay(100);
    }
    Serial.println("No-motion detection: ENABLED");

    // Enable significant-motion detection
    if (!imu.enableSignificantMotion(InterruptPin::INT1)) {
        Serial.println("Failed to enable significant-motion detection!");
        while (1) delay(100);
    }
    Serial.println("Significant-motion detection: ENABLED");

    Serial.println("\nMonitoring motion events...\n");
}

void loop() {
    uint8_t status = imu.readMotionStatus();

    // Check each motion event flag
    if (status & 0x04) { // Bit 2: any-motion
        Serial.println(">> ANY MOTION detected!");
    }

    if (status & 0x02) { // Bit 1: no-motion
        Serial.println(">> NO MOTION detected (sensor is still)");
    }

    if (status & 0x01) { // Bit 0: significant motion
        Serial.println(">> SIGNIFICANT MOTION detected!");
    }

    if (status & 0x40) { // Bit 6: tap (if enabled)
        Serial.println(">> TAP detected (as part of motion events)");
    }

    delay(50); // Check 20 times per second
}
