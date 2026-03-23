/**
 * AttitudeEngine - QMI8658 Library Example
 *
 * Demonstrates the QMI8658's on-chip Attitude Engine, which performs
 * 6-axis sensor fusion (accelerometer + gyroscope) using a built-in
 * Extended Kalman Filter (XKF3) to output orientation as a quaternion.
 *
 * The Attitude Engine runs entirely in the sensor's hardware, providing:
 *   - Quaternion orientation (w, x, y, z)
 *   - Delta velocity increments
 *   - Automatic gyroscope bias calibration
 *
 * This example also converts the quaternion to Euler angles (roll, pitch, yaw)
 * for easier visualization.
 *
 * Wiring: Same as BasicAccelGyro example.
 */

#include <QMI8658.h>

QMI8658 imu;

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    Serial.println("QMI8658 Attitude Engine Example");
    Serial.println("================================");

    if (!imu.begin()) {
        Serial.println("QMI8658 not found!");
        while (1) delay(100);
    }

    // Configure sensors for attitude estimation
    // The Attitude Engine works best with moderate ODR
    imu.configAccel(AccelRange::RANGE_8G, AccelODR::ODR_500HZ);
    imu.configGyro(GyroRange::RANGE_512DPS, GyroODR::ODR_500HZ);

    // Enable the on-chip Attitude Engine
    if (!imu.enableAttitudeEngine()) {
        Serial.println("Failed to enable Attitude Engine!");
        Serial.print("Error code: ");
        Serial.println(imu.getError());
        while (1) delay(100);
    }

    Serial.println("Attitude Engine enabled.");
    Serial.println("Rotate the sensor to see orientation changes.\n");
}

void loop() {
    if (imu.dataReady()) {
        // Read the Attitude Engine output
        AttitudeData attitude = imu.readAttitude();

        // Print quaternion
        Serial.print("Quat: w=");
        Serial.print(attitude.quat.w, 4);
        Serial.print("  x=");
        Serial.print(attitude.quat.x, 4);
        Serial.print("  y=");
        Serial.print(attitude.quat.y, 4);
        Serial.print("  z=");
        Serial.println(attitude.quat.z, 4);

        // Convert quaternion to Euler angles (in degrees)
        float w = attitude.quat.w;
        float x = attitude.quat.x;
        float y = attitude.quat.y;
        float z = attitude.quat.z;

        // Roll (rotation around X axis)
        float sinr = 2.0f * (w * x + y * z);
        float cosr = 1.0f - 2.0f * (x * x + y * y);
        float roll = atan2f(sinr, cosr) * 180.0f / PI;

        // Pitch (rotation around Y axis)
        float sinp = 2.0f * (w * y - z * x);
        float pitch;
        if (fabsf(sinp) >= 1.0f) {
            pitch = copysignf(90.0f, sinp); // Clamp to +/-90
        } else {
            pitch = asinf(sinp) * 180.0f / PI;
        }

        // Yaw (rotation around Z axis)
        float siny = 2.0f * (w * z + x * y);
        float cosy = 1.0f - 2.0f * (y * y + z * z);
        float yaw = atan2f(siny, cosy) * 180.0f / PI;

        Serial.print("Euler: Roll=");
        Serial.print(roll, 1);
        Serial.print("  Pitch=");
        Serial.print(pitch, 1);
        Serial.print("  Yaw=");
        Serial.println(yaw, 1);

        // Print delta velocity
        Serial.print("dV:   X=");
        Serial.print(attitude.dVX, 4);
        Serial.print("  Y=");
        Serial.print(attitude.dVY, 4);
        Serial.print("  Z=");
        Serial.println(attitude.dVZ, 4);

        Serial.println();
    }

    delay(20); // ~50 Hz
}
