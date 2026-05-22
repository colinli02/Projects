#include <Wire.h>
#include <Adafruit_AS5600.h>
#include <Arduino_BMI270_BMM150.h>
#include "mbed.h"
#include <ArduinoBLE.h>
#include <string.h>

#define TCA_ADDR 0x70
Adafruit_AS5600 encoder1;
Adafruit_AS5600 encoder2;

#define BUFFER_SIZE 20

// IMU variables
float comp_angle = 0;
float gyro_angle = 0;
unsigned long lastTime = 0;

// Motor control
#define MOTOR_A_FWD 9  
#define MOTOR_A_BACK 8  
#define MOTOR_B_FWD 10 
#define MOTOR_B_BACK 7  
#define OFF 0
#define FORWARD 1 
#define BACKWARD -1
#define RIGHT 2
#define LEFT 3
#define STOP 0


// PWM setup
mbed::PwmOut pwmPinA1(digitalPinToPinName(MOTOR_A_FWD));
mbed::PwmOut pwmPinA2(digitalPinToPinName(MOTOR_A_BACK));
mbed::PwmOut pwmPinB1(digitalPinToPinName(MOTOR_B_FWD));
mbed::PwmOut pwmPinB2(digitalPinToPinName(MOTOR_B_BACK));

// PID
float KP = 8.5, KI = 40.0, KD = 0.525;
#define ZERO_ANGLE  -2.4
float targetAngle = ZERO_ANGLE; 
float currentError = 0.0, lastError = 0.0, lastAngle = 0.0;
float propTerm = 0.0, intTerm = 0.0, derivTerm = 0.0;
float pidControlSignal = 0.0;
float maxIntegralLimit = 0.0, minIntegralLimit = 0.0;

float positionIntegral = 0.0;
float lastPositionError = 0.0;
const float POSITION_KP = 0.002;  // tune these
const float POSITION_KI = 0.0001; 
const float POSITION_KD = 0.0005; 


// === Position Hold Additions ===
float wheel1Pos = 0.0, wheel2Pos = 0.0;
float lastAngle1 = 0.0, lastAngle2 = 0.0;
// ================================

// BLE setup
BLEService customService("00000000-5EC4-4083-81CD-A10B8D5CF6EC");
BLECharacteristic customCharacteristic("00000001-5EC4-4083-81CD-A10B8D5CF6EC", BLERead | BLEWrite | BLENotify, BUFFER_SIZE, false);

// TCA helper
void tcaSelect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

// Init mode
static char mode[5];
void updateMode(const char* receivedString) {
  strncpy(mode, receivedString, sizeof(mode) - 1);
  mode[sizeof(mode) - 1] = '\0';
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(100);

  // Init TCA + Encoders
  tcaSelect(5);
  delay(10);
  if (!encoder1.begin()) {
    Serial.println("AS5600 not detected on channel 1!");
    while (1);
  } else {
    Serial.println("AS5600 detected on channel 1.");
  }

  tcaSelect(2);
  delay(10);
  if (!encoder2.begin()) {
    Serial.println("AS5600 not detected on channel 2!");
    while (1);
  } else {
    Serial.println("AS5600 detected on channel 2.");
  }

  // BLE
  pinMode(LED_BUILTIN, OUTPUT);
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }

  BLE.setLocalName("BLE-GROUP9");
  BLE.setDeviceName("BLE-GROUP9");
  customService.addCharacteristic(customCharacteristic);
  BLE.addService(customService);
  customCharacteristic.writeValue("Waiting for data");
  BLE.advertise();
  Serial.println("Bluetooth® device active, waiting for connections...");

  // IMU
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  // PWM
  const int frequency = 200;
  const int frequencyB = 200;
  pwmPinA1.period(1.0 / frequency);
  pwmPinA2.period(1.0 / frequency);
  pwmPinB1.period(1.0 / frequencyB);
  pwmPinB2.period(1.0 / frequencyB);

  lastTime = micros();
  
  // Initialize encoder angles tracking variables:
  tcaSelect(5);
  lastAngle1 = encoder1.getAngle();

  tcaSelect(2);
  lastAngle2 = encoder2.getAngle();

  // Initialize wheel positions
  wheel1Pos = 0.0;
  wheel2Pos = 0.0;
}


static char prevMode[5] = "";

void loop() {
  BLEDevice central = BLE.central(); 
  char receivedString[5];

  if (central) {
    if (customCharacteristic.written()) {
      const unsigned char* receivedData = customCharacteristic.value();
      memcpy(receivedString, receivedData, 5);
      receivedString[4] = '\0';
      Serial.print(" Received data: ");
      Serial.print(receivedString);
      updateMode(receivedString);
      Serial.print(" Mode: ");
      Serial.print(mode);
      customCharacteristic.writeValue("Data received");
    }
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }

  checkSerialCommands();

  float ax, ay, az, gx, gy, gz;

  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);

    float accelerometerAngle = atan2(ay, az) * 180.0 / PI;
    unsigned long currentTime = micros();
    float dt = (currentTime - lastTime) / 1000000.0;
    lastTime = currentTime;

    // Kalman Filter
    static float currentAngle = 0.0f;
    static float errorCovariance = 4.0f;
    const float GYRO_NOISE = 0.003f, ACC_NOISE = 0.03f;
    const float KALMAN_UPDATE_FACTOR = 1.0f;

    currentAngle = currentAngle - gx * dt;
    errorCovariance += dt * dt * GYRO_NOISE * GYRO_NOISE;
    float adaptiveWeight = errorCovariance / (errorCovariance + ACC_NOISE * ACC_NOISE);
    currentAngle += adaptiveWeight * (accelerometerAngle - currentAngle);
    errorCovariance *= (KALMAN_UPDATE_FACTOR - adaptiveWeight);

    comp_angle = currentAngle;

    // PID
    const float UPPER_BOUND = 1000.0, LOWER_BOUND = -1000.0;
    currentError = targetAngle - currentAngle;

    Serial.print(" | targetAngle: ");
    Serial.print(targetAngle);

    propTerm = KP * currentError;
    derivTerm = -1 * KD * (currentAngle - lastAngle) / dt;
    float pdTerm  = propTerm + derivTerm;

    if (KI != 0.0) {
      intTerm += (KI * dt) * (currentError + lastError) / 2.0;
      maxIntegralLimit = UPPER_BOUND - pdTerm;
      minIntegralLimit = LOWER_BOUND - pdTerm;
      intTerm = constrain(intTerm, minIntegralLimit, maxIntegralLimit);
    } else intTerm = 0.0;

    pidControlSignal  = constrain(propTerm + derivTerm + intTerm, LOWER_BOUND, UPPER_BOUND);
    lastAngle = currentAngle;
    lastError = currentError;


    int direction = (pidControlSignal  > 0) ? FORWARD : BACKWARD;
    float pwm = constrain(abs(pidControlSignal) / 300.0, 0.0, 1.0);

// === AS5600 angle reads ===
// Seems to go from 0 to 4096 per wheel raw TCA values
  const float FULL_SCALE = 4096.0;
  const float HALF_SCALE = FULL_SCALE / 2.0;
  
  tcaSelect(5);
  float angle1 = encoder1.getAngle();
  tcaSelect(2);
  float angle2 = encoder2.getAngle();
  
  // Compute delta and handle wrap-around
  float delta1 = angle1 - lastAngle1;
  float delta2 = angle2 - lastAngle2;
  
  if (delta1 > HALF_SCALE) delta1 -= FULL_SCALE;
  if (delta1 < -HALF_SCALE) delta1 += FULL_SCALE;
  
  if (delta2 > HALF_SCALE) delta2 -= FULL_SCALE;
  if (delta2 < -HALF_SCALE) delta2 += FULL_SCALE;
  
  wheel1Pos += delta1;
  wheel2Pos += delta2;
  
  lastAngle1 = angle1;
  lastAngle2 = angle2;
  
  float avgWheelPos = (wheel1Pos + wheel2Pos) / 2.0;

  Serial.print(" | wheel1: ");
  Serial.print(wheel1Pos);
  Serial.print(" | wheel2: ");
  Serial.print(wheel2Pos);


  static float leftMultiplier = 1.0, rightMultiplier = 1.0;

// Target Angle in different modes
    if (strcmp(mode, "FRWD") == 0) targetAngle = ZERO_ANGLE + 1.25;
    else if (strcmp(mode, "BACK") == 0) targetAngle = ZERO_ANGLE - 2.0;
    else if (strcmp(mode, "RGHT") == 0 || strcmp(mode, "LEFT") == 0) targetAngle = ZERO_ANGLE;
    // STOP MODE
    // Inside your loop STOP mode:
    else {
        // STOP mode PID Loop
        
        float positionError = avgWheelPos;  // positive error for positive correction
        
        // Calculate dt safely:
        unsigned long currentTime = micros();
        float dt = (lastTime == 0) ? 0 : (currentTime - lastTime) / 1000000.0;
        lastTime = currentTime;
        if (dt <= 0) dt = 0.001;  // fallback small dt
    
        // Integral term update with clamping
        positionIntegral += positionError * dt;

        // Anti-windup clamping
        const float MAX_POSITION_INTEGRAL = 1000.0;  // adjust as needed
        positionIntegral = constrain(positionIntegral, -MAX_POSITION_INTEGRAL, MAX_POSITION_INTEGRAL);
    
        // Derivative term
        float positionDerivative = (positionError - lastPositionError) / dt;
        lastPositionError = positionError;
    
        // PID control
        float positionCorrection = POSITION_KP * positionError 
                                 + POSITION_KI * positionIntegral 
                                 + POSITION_KD * positionDerivative;
    
        targetAngle = ZERO_ANGLE + positionCorrection;
    
        // Clamp targetAngle to safe limits [-1.5, 1.5]
        targetAngle = constrain(targetAngle, ZERO_ANGLE - 1.5, ZERO_ANGLE + 1.5);
		
		
		// PID Loop to control wheel sync differential 
		static float syncErrorIntegral = 0.0;
		static float lastSyncError = 0.0;

		float syncError = wheel1Pos - wheel2Pos;
		
		Serial.print(" | syncError: ");
		Serial.print(syncError);
		
		syncErrorIntegral += syncError * dt; // dt: time since last loop
		float syncErrorDerivative = (syncError - lastSyncError) / dt;
		lastSyncError = syncError;

		// PID gains — tweak these based on testing
		const float SYNC_KP = 0.0005;
		const float SYNC_KI = 0.0;
		const float SYNC_KD = 0.0001;

		float syncCorrection = SYNC_KP * syncError 
							 + SYNC_KI * syncErrorIntegral 
							 + SYNC_KD * syncErrorDerivative;

		// Apply correction symmetrically
		leftMultiplier  -= syncCorrection;
		rightMultiplier += syncCorrection;
		}

    // Detect mode change
    bool modeChanged = strcmp(mode, prevMode) != 0;
    if (modeChanged) {
      Serial.print("RESETTING POSITION PID ");
  
		if (strcmp(mode, "STOP") == 0) 
		{
		// Reset position PID
		positionIntegral = 0.0;
		lastPositionError = 0.0;

		// Reset sync PID too 👇
		syncErrorIntegral = 0.0;
		lastSyncError = 0.0;

		// Wheel positions reference
		wheel1Pos = 0.0;
		wheel2Pos = 0.0;
		}
      strcpy(prevMode, mode);
    }

  float turnRateR = 1.0;
  float turnRateL = 1.0;
  // Turning PWM scale
    if (strcmp(mode, "RGHT") == 0) {
      turnRateL = 0.8;
      turnRateR  = 1.0;
    } else if (strcmp(mode, "LEFT") == 0) {
      turnRateL = 1.0;
      turnRateR  = 0.8;
    } else if (
      strcmp(mode, "FRWD") == 0 ||
      strcmp(mode, "BACK") == 0 || strcmp(mode, "STOP") == 0 
    ) {
      turnRateL = 1.0;
      turnRateR  = 1.0;
    }

  // Balancing Logic
    if (abs(comp_angle) < 30.0) 
    {
      if (direction == FORWARD) {
          float pwmLeft = constrain(pwm * leftMultiplier * turnRateL, 0.0, 1.0);
          float pwmRight = constrain(pwm * rightMultiplier * turnRateR, 0.0, 1.0);
          pwmPinA1.write(pwmLeft);
          pwmPinB1.write(pwmRight);
          pwmPinA2.write(0.0);
          pwmPinB2.write(0.0);
      } 
      else 
      {
          float pwmLeft = constrain(pwm * leftMultiplier*turnRateL, 0.0, 1.0);
          float pwmRight = constrain(pwm * rightMultiplier*turnRateR, 0.0, 1.0);
          pwmPinA1.write(0.0);
          pwmPinB1.write(0.0);
          pwmPinA2.write(pwmLeft);
          pwmPinB2.write(pwmRight);
      }
    } 
    else 
    {
      pwmPinA1.write(OFF); pwmPinB1.write(OFF);
      pwmPinA2.write(OFF); pwmPinB2.write(OFF);
    }

    Serial.print(" | Angle1: ");
    Serial.print(angle1);
    Serial.print(" | Angle2: ");
    Serial.println(angle2);  // Final println
  }
}

// === Serial PID Tuning Commands ===
void checkSerialCommands() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.startsWith("KP=")) KP = cmd.substring(3).toFloat();
    else if (cmd.startsWith("KI=")) KI = cmd.substring(3).toFloat();
    else if (cmd.startsWith("KD=")) KD = cmd.substring(3).toFloat();
    else if (cmd == "SHOW") {
      Serial.print("KP="); Serial.println(KP);
      Serial.print("KI="); Serial.println(KI);
      Serial.print("KD="); Serial.println(KD);
      Serial.println("====================");
    }
  }
}