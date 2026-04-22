#include <Arduino_BMI270_BMM150.h>
#include "mbed.h"
#include <AutoPID.h>

bool debugging = false; // Set to true to run tiltControlledSpeed()

float comp_angle = 0;
float gyro_angle = 0;
float alpha = 0.95;
unsigned long lastTime = 0;

// Offsets
float ax0 = 0;
float ay0 = 0;
float az0 = 0;

// FLAGS
bool stopIssued = false;

// Motor pins
#define MOTOR_A_FWD 9  
#define MOTOR_A_BACK 8  
#define MOTOR_B_FWD 10 
#define MOTOR_B_BACK 7  

#define ON 255
#define OFF 0

#define FORWARD 1 
#define BACKWARD -1
#define RIGHT 2
#define LEFT 3
#define STOP 0

const int speedLevels[] = {0, 25, 50, 75, 100};

// PWM pin setup
mbed::PwmOut pwmPinA1(digitalPinToPinName(MOTOR_A_FWD));
mbed::PwmOut pwmPinA2(digitalPinToPinName(MOTOR_A_BACK));
mbed::PwmOut pwmPinB1(digitalPinToPinName(MOTOR_B_FWD));
mbed::PwmOut pwmPinB2(digitalPinToPinName(MOTOR_B_BACK));

// === AutoPID Setup ===
double input, output;
double setpoint = 0.0; // Balanced = 0 degrees

double KP = 2.6;
double KI = 0.0;
double KD = 0.0;

#define OUTPUT_MIN -100
#define OUTPUT_MAX 100

AutoPID pid(&input, &setpoint, &output, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);

// === Frequencies ===
const int frequency = 20000;
const int frequencyB = 20000;

void setup() 
{
    Serial.begin(9600);

    if (!IMU.begin()) {
        Serial.println("Failed to initialize IMU!");
        while (1);
    }

    pid.setGains(KP, KI, KD);
    pid.setBangBang(5);      // Optional: switch to bang-bang when close
    pid.setTimeStep(5);     // Optional: update interval (ms)
}

void loop() {
    checkSerialCommands();

    // Sanity check debugging mode
    if (debugging) 
    {
        tiltControlledSpeed();
        delay(10); // prevent spamming serial
        return;
    }

    // === PID Mode ===
    pwmPinA1.period(1.0 / frequency);
    pwmPinA2.period(1.0 / frequency);
    pwmPinB1.period(1.0 / frequencyB);
    pwmPinB2.period(1.0 / frequencyB);

    float ax, ay, az, gx, gy, gz;

    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) 
    {
        IMU.readAcceleration(ax, ay, az);
        ax -= ax0;
        ay -= ay0;
        az -= az0;
        IMU.readGyroscope(gx, gy, gz);

        float accel_angle = atan2(ay, az) * 180.0 / PI;

        unsigned long currentTime = micros();
        float dt = (currentTime - lastTime) / 1000000.0;
        lastTime = currentTime;

        // Complementary filter
        gyro_angle = comp_angle + (-1) * gx * dt;
        comp_angle = alpha * gyro_angle + (1 - alpha) * accel_angle;

        input = comp_angle;
        pid.run();
        float pid_output = output;
        Serial.print("pid_output: ");
        Serial.print(pid_output);

        int MAX_PWM = 100;
        float scale = 1.0;
        int pwm = constrain(abs(pid_output), 0, MAX_PWM);
        int direction = (pid_output > 0) ? FORWARD : BACKWARD;
        float percentage = pwm * scale;

        if (abs(comp_angle) <= 25) {
            stopIssued = false;
        }

        Serial.print("Comp Angle: ");
        Serial.print(comp_angle);

        if (abs(comp_angle) > 55 && !stopIssued) {
            stopIssued = true;
            return;
        } else if (direction == FORWARD) {
            driveMotors(FORWARD, percentage);
            Serial.print(" | Percentage: ");
            Serial.println(percentage);
        } else {
            driveMotors(BACKWARD, percentage);
            Serial.print(" | Percentage: ");
            Serial.println(percentage);
        }
    }
}

void driveMotors(int direction, int percentage) {
    float pwmValue = percentage / 100.0;

    if (direction == FORWARD) {
        Serial.print("Forwards ");
        pwmPinA1.write(pwmValue);
        pwmPinB1.write(pwmValue);
        pwmPinA2.write(OFF);
        pwmPinB2.write(OFF);
    } else if (direction == BACKWARD) {
        Serial.print("Backwards ");
        pwmPinA1.write(OFF);
        pwmPinB1.write(OFF);
        pwmPinA2.write(pwmValue);
        pwmPinB2.write(pwmValue);
    } else if (direction == RIGHT) {
        Serial.print("Right ");
        pwmPinA1.write(OFF);
        pwmPinB1.write(pwmValue);
        pwmPinA2.write(pwmValue);
        pwmPinB2.write(OFF);
    } else if (direction == LEFT) {
        Serial.print("Left ");
        pwmPinA1.write(pwmValue);
        pwmPinB1.write(OFF);
        pwmPinA2.write(OFF);
        pwmPinB2.write(pwmValue);
    } else {
        Serial.print("Stop ");
        pwmPinA1.write(OFF);
        pwmPinB1.write(OFF);
        pwmPinA2.write(OFF);
        pwmPinB2.write(OFF);
    }
}

void tiltControlledSpeed() 
{
    float ax, ay, az, gx, gy, gz;
    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) 
    {
        IMU.readAcceleration(ax, ay, az); 
        ax = ax - ax0;
        ay = ay - ay0;
        az = az - az0;
        IMU.readGyroscope(gx, gy, gz);

        float accel_angle = atan2(ay, az) * 180 / PI;

        unsigned long currentTime = micros();
        float dt = (currentTime - lastTime) / 1000000.0;
        lastTime = currentTime;

        gyro_angle = comp_angle + (-1) * gx * dt;
        comp_angle = alpha * (gyro_angle) + (1 - alpha) * accel_angle;

        float angle = comp_angle;
        Serial.println("Tilt Controlled Speed :"); 
        Serial.println(angle); 

        float percentage = map(abs(angle), 0, 40, 0, 100); 

        if (angle > 0) {
            driveMotors(FORWARD, percentage);
        } else {
            driveMotors(BACKWARD, percentage);
        }
        Serial.print("Speed %: ");
        Serial.println(percentage); 
    }
}

// === Serial PID Tuning Commands ===
void checkSerialCommands() 
{
    if (Serial.available()) 
    {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();

        if (cmd.startsWith("KP=")) {
            KP = cmd.substring(3).toFloat();
            Serial.print("New KP: "); Serial.println(KP);
        } else if (cmd.startsWith("KI=")) {
            KI = cmd.substring(3).toFloat();
            Serial.print("New KI: "); Serial.println(KI);
        } else if (cmd.startsWith("KD=")) {
            KD = cmd.substring(3).toFloat();
            Serial.print("New KD: "); Serial.println(KD);
        } else if (cmd == "SHOW") {
            Serial.print("KP="); Serial.println(KP);
            Serial.print("KI="); Serial.println(KI);
            Serial.print("KD="); Serial.println(KD);
        } else if (cmd == "DBG=1") {
            debugging = true;
            Serial.println("Debugging mode: ON");
        } else if (cmd == "DBG=0") {
            debugging = false;
            Serial.println("Debugging mode: OFF");
        }

        // Apply new gains
        pid.setGains(KP, KI, KD);
    }
}
