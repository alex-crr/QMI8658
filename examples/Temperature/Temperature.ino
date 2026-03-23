/**
 * Temperature - QMI8658 Library Example
 *
 * Reads the built-in temperature sensor of the QMI8658 and prints
 * the value to the Serial Monitor.
 *
 * The on-chip temperature sensor measures die temperature, which is
 * typically a few degrees above ambient due to self-heating. It is
 * useful for monitoring thermal drift and relative temperature changes,
 * but should not be used as a precision ambient temperature sensor.
 *
 * Wiring: Same as BasicAccelGyro example.
 */

#include <QMI8658.h>

QMI8658 imu;

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    Serial.println("QMI8658 Temperature Sensor Example");
    Serial.println("===================================");

    if (!imu.begin()) {
        Serial.println("QMI8658 not found!");
        while (1) delay(100);
    }

    Serial.println("Sensor initialized. Reading temperature...\n");
}

void loop() {
    float temp = imu.readTemperature();

    Serial.print("Temperature: ");
    Serial.print(temp, 2);
    Serial.println(" C");

    // Also read timestamp to demonstrate the sample counter
    uint32_t timestamp = imu.readTimestamp();
    Serial.print("Timestamp:   ");
    Serial.println(timestamp);

    Serial.println();
    delay(1000); // Read once per second
}
