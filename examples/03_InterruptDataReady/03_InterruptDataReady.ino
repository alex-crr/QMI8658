/**
 * InterruptDataReady - QMI8658 Library Example
 *
 * Demonstrates interrupt-driven data reading using the QMI8658's
 * data-ready (DRDY) signal on INT2.
 *
 * Per the datasheet, DRDY is always routed to INT2 (edge-triggered,
 * rising edge indicates new data). FIFO mode must be set to BYPASS
 * (the default) for DRDY to function.
 *
 * Wiring:
 *   Same as BasicAccelGyro, plus:
 *   QMI8658 INT2 -> Board GPIO (set INT_PIN below)
 */

#include <QMI8658.h>

QMI8658 imu;

// Set this to the GPIO pin connected to the QMI8658's INT2 output
#define INT_PIN 8

// Volatile flag set by the interrupt service routine
volatile bool dataReadyFlag = false;

// Interrupt service routine - keep it minimal!
#if defined(ESP32) || defined(ESP8266)
void IRAM_ATTR onDataReady() {
#else
void onDataReady() {
#endif
    dataReadyFlag = true;
}

// Counter for measuring sample rate
unsigned long sampleCount = 0;
unsigned long lastReportTime = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    Serial.println("QMI8658 Interrupt Data Ready Example");
    Serial.println("=====================================");

    if (!imu.begin()) {
        Serial.println("QMI8658 not found!");
        while (1) delay(100);
    }

    // Configure sensors
    imu.configAccel(AccelRange::RANGE_8G, AccelODR::ODR_500HZ);
    imu.configGyro(GyroRange::RANGE_512DPS, GyroODR::ODR_500HZ);

    // Enable the DRDY signal on INT2
    // (DRDY is always on INT2 per the datasheet)
    imu.enableDataReadyInt();

    // Set up hardware interrupt on the MCU side
    // DRDY is a rising-edge pulse
    pinMode(INT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(INT_PIN), onDataReady, RISING);

    imu.setClock(400000); // Fast I2C for high-rate reading

    lastReportTime = millis();
    Serial.println("Interrupt-driven reading active. Showing rate every 2s.\n");
}

void loop() {
    // Only read when the interrupt fires
    if (dataReadyFlag) {
        dataReadyFlag = false;

        // Read both accel + gyro in a single burst
        IMUData data = imu.readBoth();
        sampleCount++;

        // Print every 100th sample to avoid flooding Serial
        if (sampleCount % 100 == 0) {
            Serial.print("Sample #");
            Serial.print(sampleCount);
            Serial.print("  Accel: (");
            Serial.print(data.accel.x, 3);
            Serial.print(", ");
            Serial.print(data.accel.y, 3);
            Serial.print(", ");
            Serial.print(data.accel.z, 3);
            Serial.print(") g  Gyro: (");
            Serial.print(data.gyro.x, 1);
            Serial.print(", ");
            Serial.print(data.gyro.y, 1);
            Serial.print(", ");
            Serial.print(data.gyro.z, 1);
            Serial.println(") dps");
        }
    }

    // Report effective sample rate every 2 seconds
    if (millis() - lastReportTime >= 2000) {
        float rate = sampleCount * 1000.0f / (millis() - lastReportTime);
        Serial.print("Effective sample rate: ");
        Serial.print(rate, 1);
        Serial.println(" Hz");
        sampleCount = 0;
        lastReportTime = millis();
    }
}
