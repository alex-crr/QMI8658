/**
 * Pedometer - QMI8658 Library Example
 *
 * Uses the QMI8658's built-in pedometer engine to count steps.
 *
 * How to test:
 *   - Attach the sensor to your body (e.g., in a pocket, on your wrist)
 *   - Walk normally; the step count should increment
 *   - The pedometer uses the accelerometer to detect the periodic
 *     motion pattern of walking/running
 *
 * The step counter persists until explicitly reset with resetStepCount().
 *
 * Wiring: Same as BasicAccelGyro example.
 */

#include <QMI8658.h>

QMI8658 imu;

uint32_t lastStepCount = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    Serial.println("QMI8658 Pedometer Example");
    Serial.println("=========================");

    if (!imu.begin()) {
        Serial.println("QMI8658 not found!");
        while (1) delay(100);
    }

    // Configure accelerometer for pedometer
    // 500 Hz with moderate range works well for step detection
    imu.configAccel(AccelRange::RANGE_8G, AccelODR::ODR_500HZ);

    // Enable the pedometer engine
    if (!imu.enablePedometer(InterruptPin::INT1)) {
        Serial.println("Failed to enable pedometer!");
        Serial.print("Error code: ");
        Serial.println(imu.getError());
        while (1) delay(100);
    }

    // Start with a fresh count
    imu.resetStepCount();

    Serial.println("Pedometer enabled. Start walking!\n");
}

void loop() {
    uint32_t steps = imu.readStepCount();

    // Only print when the count changes
    if (steps != lastStepCount) {
        lastStepCount = steps;
        Serial.print("Steps: ");
        Serial.println(steps);
    }

    // Also show the motion status to detect step events
    uint8_t status = imu.readMotionStatus();
    if (status & 0x80) { // Bit 7: pedometer event
        Serial.println("  (step detected)");
    }

    delay(100); // Check 10 times per second
}
