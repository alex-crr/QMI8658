/**
 * MadgwickFusion - QMI8658 Library Example
 *
 * Demonstrates software-based sensor fusion using the Madgwick algorithm
 * to compute orientation from QMI8658 accelerometer and gyroscope data.
 *
 * The Madgwick filter fuses accelerometer (gravity reference) with
 * gyroscope (angular rate) to produce a stable orientation quaternion
 * that is then converted to Euler angles.
 *
 * This example uses interrupt-driven data reading for precise timing.
 * DRDY is always on INT2 per the QMI8658A datasheet.
 *
 * Wiring:
 *   Same as BasicAccelGyro, plus:
 *   QMI8658 INT2 -> Board GPIO (set INT_PIN below)
 */

#include <QMI8658.h>

QMI8658 imu;

#define INT_PIN 8

volatile bool dataReadyFlag = false;

#if defined(ESP32) || defined(ESP8266)
void IRAM_ATTR onDataReady() {
#else
void onDataReady() {
#endif
    dataReadyFlag = true;
}

// ---- Madgwick Filter State ----
// Quaternion representing orientation (initialized to identity)
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

// Filter gain: higher = more aggressive accel correction (less gyro drift
// but more noise). Typical range: 0.01 (slow correction) to 0.5 (fast).
float beta = 0.1f;

// Timing
unsigned long lastMicros = 0;

/**
 * Madgwick AHRS filter update.
 *
 * Fuses accelerometer and gyroscope readings into a quaternion.
 *
 * @param ax, ay, az  Accelerometer in g
 * @param gx, gy, gz  Gyroscope in degrees/second
 * @param dt           Time step in seconds
 */
void madgwickUpdate(float ax, float ay, float az,
                    float gx, float gy, float gz, float dt) {
    // Convert gyro from degrees/s to radians/s
    gx *= 0.0174533f;
    gy *= 0.0174533f;
    gz *= 0.0174533f;

    // Quaternion rate of change from gyroscope
    float qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    float qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    float qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    float qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

    // Apply accelerometer correction (gravity direction)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        // Normalize accelerometer
        float norm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
        ax *= norm; ay *= norm; az *= norm;

        // Gradient descent corrective step
        float f1 = 2.0f * (q1 * q3 - q0 * q2) - ax;
        float f2 = 2.0f * (q0 * q1 + q2 * q3) - ay;
        float f3 = 1.0f - 2.0f * (q1 * q1 + q2 * q2) - az;

        float s0 =  -2.0f * q2 * f1 + 2.0f * q1 * f2;
        float s1 =   2.0f * q3 * f1 + 2.0f * q0 * f2 - 4.0f * q1 * f3;
        float s2 =  -2.0f * q0 * f1 + 2.0f * q3 * f2 - 4.0f * q2 * f3;
        float s3 =   2.0f * q1 * f1 + 2.0f * q2 * f2;

        // Normalize step
        norm = 1.0f / sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= norm; s1 *= norm; s2 *= norm; s3 *= norm;

        // Apply correction
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    // Integrate to get new quaternion
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    // Normalize quaternion
    float norm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= norm; q1 *= norm; q2 *= norm; q3 *= norm;
}

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    Serial.println("QMI8658 Madgwick Fusion Example");
    Serial.println("================================");

    if (!imu.begin()) {
        Serial.println("QMI8658 not found!");
        while (1) delay(100);
    }

    // Configure for fusion: moderate range and ODR
    imu.configAccel(AccelRange::RANGE_8G, AccelODR::ODR_500HZ);
    imu.configGyro(GyroRange::RANGE_512DPS, GyroODR::ODR_500HZ);

    // Enable DRDY interrupt on INT2 (the only pin DRDY can use)
    imu.enableDataReadyInt();

    // DRDY is a rising-edge pulse on INT2
    pinMode(INT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(INT_PIN), onDataReady, RISING);

    imu.setClock(400000);
    lastMicros = micros();

    Serial.println("Fusion running. Rotate the sensor to see angles.\n");
}

void loop() {
    if (dataReadyFlag) {
        dataReadyFlag = false;

        // Calculate time step
        unsigned long now = micros();
        float dt = (now - lastMicros) / 1000000.0f;
        if (dt > 0.1f) dt = 0.002f; // Clamp on first iteration or overflow
        lastMicros = now;

        // Read sensor data
        IMUData data = imu.readBoth();

        // Run Madgwick filter
        madgwickUpdate(data.accel.x, data.accel.y, data.accel.z,
                       data.gyro.x, data.gyro.y, data.gyro.z, dt);

        // Convert quaternion to Euler angles (degrees)
        float roll  = atan2f(2.0f * (q0 * q1 + q2 * q3),
                             1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 180.0f / PI;
        float pitch = asinf(2.0f * (q0 * q2 - q3 * q1)) * 180.0f / PI;
        float yaw   = atan2f(2.0f * (q0 * q3 + q1 * q2),
                             1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 180.0f / PI;

        // Print at ~25 Hz (every 20th sample at 500 Hz)
        static uint16_t printCounter = 0;
        if (++printCounter >= 20) {
            printCounter = 0;

            Serial.print("Roll=");
            Serial.print(roll, 1);
            Serial.print("  Pitch=");
            Serial.print(pitch, 1);
            Serial.print("  Yaw=");
            Serial.print(yaw, 1);
            Serial.print("  |  Quat: ");
            Serial.print(q0, 4);
            Serial.print(" ");
            Serial.print(q1, 4);
            Serial.print(" ");
            Serial.print(q2, 4);
            Serial.print(" ");
            Serial.println(q3, 4);
        }
    }
}
