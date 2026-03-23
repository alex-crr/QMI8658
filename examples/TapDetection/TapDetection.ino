/**
 * TapDetection - QMI8658 Library Example
 *
 * Demonstrates single-tap and double-tap detection using the QMI8658's
 * built-in tap recognition engine.
 *
 * How to test:
 *   - Place the sensor on a solid surface
 *   - Tap the surface near the sensor with your finger
 *   - Single taps and double taps will be reported on Serial
 *
 * The tap detection runs entirely in hardware, so it works even
 * when the main loop is doing other work.
 *
 * Wiring: Same as BasicAccelGyro example.
 *         Optionally connect INT1 pin for interrupt-driven detection.
 */

#include <QMI8658.h>

QMI8658 imu;

// Uncomment and set this to your interrupt pin for interrupt-driven mode:
// #define TAP_INT_PIN 2

#ifdef TAP_INT_PIN
volatile bool tapEvent = false;

void IRAM_ATTR onTapInterrupt() {
    tapEvent = true;
}
#endif

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    Serial.println("QMI8658 Tap Detection Example");
    Serial.println("==============================");

    if (!imu.begin()) {
        Serial.println("QMI8658 not found!");
        while (1) delay(100);
    }

    // Configure accelerometer for tap detection
    // Lower ODR and moderate range work well for tap detection
    imu.configAccel(AccelRange::RANGE_4G, AccelODR::ODR_500HZ);

    // Enable both single-tap and double-tap detection on INT1
    if (!imu.enableTapDetection(true, true, InterruptPin::INT1)) {
        Serial.println("Failed to enable tap detection!");
        Serial.print("Error code: ");
        Serial.println(imu.getError());
        while (1) delay(100);
    }

#ifdef TAP_INT_PIN
    // Set up hardware interrupt for faster response
    pinMode(TAP_INT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(TAP_INT_PIN), onTapInterrupt, FALLING);
    Serial.println("Interrupt-driven tap detection enabled.");
#else
    Serial.println("Polling-based tap detection enabled.");
#endif

    Serial.println("Tap the sensor to detect events!\n");
}

void loop() {
    bool shouldCheck = false;

#ifdef TAP_INT_PIN
    // Interrupt-driven: only check when interrupt fires
    if (tapEvent) {
        tapEvent = false;
        shouldCheck = true;
    }
#else
    // Polling mode: check motion status periodically
    uint8_t motionStatus = imu.readMotionStatus();
    if (motionStatus & 0x40) { // Bit 6: tap event flag
        shouldCheck = true;
    }
#endif

    if (shouldCheck) {
        uint8_t tapStatus = imu.readTapStatus();

        if (tapStatus & 0x80) {
            Serial.println(">> DOUBLE TAP detected!");
        }

        // Check individual axes for single tap
        if (tapStatus & 0x01) Serial.println("   Single tap: +X axis");
        if (tapStatus & 0x02) Serial.println("   Single tap: -X axis");
        if (tapStatus & 0x04) Serial.println("   Single tap: +Y axis");
        if (tapStatus & 0x08) Serial.println("   Single tap: -Y axis");
        if (tapStatus & 0x10) Serial.println("   Single tap: +Z axis");
        if (tapStatus & 0x20) Serial.println("   Single tap: -Z axis");

        if (tapStatus == 0) {
            Serial.println("   (tap event cleared)");
        }

        Serial.println();
    }

    delay(10);
}
