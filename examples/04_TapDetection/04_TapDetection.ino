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
    // 500 Hz ODR recommended for reliable tap detection
    imu.configAccel(AccelRange::RANGE_4G, AccelODR::ODR_500HZ);

    // Enable tap detection on INT1
    if (!imu.enableTapDetection(InterruptPin::INT1)) {
        Serial.println("Failed to enable tap detection!");
        Serial.print("Error code: ");
        Serial.println(imu.getError());
        while (1) delay(100);
    }

#ifdef TAP_INT_PIN
    // Set up hardware interrupt for faster response
    pinMode(TAP_INT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(TAP_INT_PIN), onTapInterrupt, RISING);
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
    // Polling mode: check STATUS1 for tap event flag
    uint8_t motionStatus = imu.readMotionStatus();
    // STATUS1 bit 1: Tap detected (per datasheet Table 24)
    if (motionStatus & 0x02) {
        shouldCheck = true;
    }
#endif

    if (shouldCheck) {
        // TAP_STATUS register (0x59) layout per datasheet Table 26:
        //   Bit 7:   TAP_POLARITY (0=positive, 1=negative direction)
        //   Bits 5:4: TAP_AXIS (0=none, 1=X, 2=Y, 3=Z)
        //   Bits 1:0: TAP_NUM (0=none, 1=single, 2=double)
        uint8_t tapStatus = imu.readTapStatus();

        uint8_t tapNum  = tapStatus & 0x03;
        uint8_t tapAxis = (tapStatus >> 4) & 0x03;
        bool    tapNeg  = (tapStatus >> 7) & 0x01;

        if (tapNum == 1) {
            Serial.print(">> SINGLE TAP on ");
        } else if (tapNum == 2) {
            Serial.print(">> DOUBLE TAP on ");
        }

        if (tapNum > 0) {
            switch (tapAxis) {
                case 1: Serial.print("X axis"); break;
                case 2: Serial.print("Y axis"); break;
                case 3: Serial.print("Z axis"); break;
                default: Serial.print("unknown axis"); break;
            }
            Serial.println(tapNeg ? " (negative direction)" : " (positive direction)");
        }

        Serial.println();
    }

    delay(10);
}
