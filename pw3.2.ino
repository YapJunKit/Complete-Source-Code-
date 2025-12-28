#include <Wire.h>
#include <LiquidCrystal.h>

// Motor Control Pins
#define MOTOR_A_EN 11   // Enable pin for Motor A (must be PWM)
#define MOTOR_A_IN1 12 // Direction pin 1 for Motor A
#define MOTOR_A_IN2 13 // Direction pin 2 for Motor A
#define MOTOR_B_EN 3  // Enable pin for Motor B (must be PWM)
#define MOTOR_B_IN1 2  // Direction pin 1 for Motor B
#define MOTOR_B_IN2 1  // Direction pin 2 for Motor B

// LCD Pin Setup
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// PID Control Variables
double Kp = 50;   // Proportional gain
double Ki = 0;    // Integral gain
double Kd = 0.8;  // Derivative gain

double yaw_setpoint = 0;  // We want to go straight, so the target yaw is 0
double pid_error = 0;
double pid_integral = 0;
double pid_derivative = 0;
double pid_previous_error = 0;
double pid_output = 0;
int pitchState = 0;

// Motor Speed Variables
int baseSpeed = 180;  // Base speed for both motors (0-255). Adjust this!
int motorSpeedA = 0;
int motorSpeedB = 0;

// IMU Variables
const int MPU = 0x68;  // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw = 0.0;  // Initialize yaw to 0
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;

// State and Timing Variables
unsigned long lastLCDUpdate = 0;
bool stopOperation = false;
unsigned long lastMillis = 0;
int c = 0;

// --- ADDED: Variables for tracking maximum pitch ---
double maxPitchReached = 0.0;  // Track maximum pitch angle
// --- END ADDED ---

void setup() {
  Wire.begin();                 // Initialize comunication
  Wire.beginTransmission(MPU);  // Start communication with MPU6050
  Wire.write(0x6B);             // Talk to the register 6B
  Wire.write(0x00);             // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);   // End the transmission

  // LCD Setup
  lcd.begin(16, 2);  // Set up the LCD's number of columns and rows
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Initializing...");
  delay(1000);

  // Motor Pin Setup
  pinMode(MOTOR_A_EN, OUTPUT);
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_EN, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);

  stopAllMotors();

  // Calculate IMU error values
  calculate_IMU_error();
  delay(20);

  yaw = 0;
  
  // Clear LCD for main display
  lcd.clear();

  pitchState = 0;
  stopOperation = false;
  stopPermanently = false;

  currentTime = millis();
  previousTime = currentTime;
}

void loop() {
  if (stopPermanently) {
    stopAllMotors();
    return;  // Stop all processing if permanently stopped
  }
  previousTime = currentTime;                           // Previous time is stored before the actual time read
  currentTime = millis();                               // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000.0;  // Divide by 1000 to get seconds

  // Read gyro data
  Wire.beginTransmission(MPU);
  Wire.write(0x43);  // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);                    // Read 6 registers total
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0;  // deg/s
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;  // deg/s
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;  // deg/s

  // Correct the gyro outputs with calculated error values
  GyroZ = GyroZ - GyroErrorZ;
  GyroY = GyroY - GyroErrorY;
  yaw = yaw + GyroZ * elapsedTime;
  pitch = pitch + GyroY * elapsedTime;

  // --- ADDED: Track maximum pitch value ---
  if (pitch > maxPitchReached) {
    maxPitchReached = pitch;
  }
  // --- END ADDED ---

  // PID Calculation
  pid_error = yaw_setpoint - yaw;  // Calculate the error

  pid_integral = pid_integral + (pid_error * elapsedTime);
  pid_integral = constrain(pid_integral, -100, 100);

  pid_derivative = (pid_error - pid_previous_error) / elapsedTime;

  pid_output = (Kp * pid_error) + (Ki * pid_integral) + (Kd * pid_derivative);

  pid_previous_error = pid_error;

  // Motor Control Logic
  if (!stopOperation) {
    motorSpeedB = baseSpeed - pid_output;  // Right Motor
    motorSpeedA = baseSpeed + pid_output;  // Left Motor

    // Constrain the motor speeds
    motorSpeedA = constrain(motorSpeedA, -255, 255);
    motorSpeedB = constrain(motorSpeedB, -255, 255);

    setMotorSpeed('A', motorSpeedA);
    setMotorSpeed('B', motorSpeedB);
  } else {
    setMotorSpeed('A', 0);
    setMotorSpeed('B', 0);
  }

  // State Machine for ramp climbing sequence
  if (pitchState == 0) {
    if (pitch >= 20) {
      pitchState = 1;
      delay(100);
      stopOperation = true;
      lastMillis = millis();
    }
  } else if (pitchState == 1) {
    if (millis() - lastMillis >= 5000) {
      stopOperation = false;
      pitchState = 2;
    }
  } else if (pitchState == 2) {
    if (pitch <= 5) {
      pitchState = 3;
      delay(200);
      stopOperation = true;
      lastMillis = millis();
    }
  } else if (pitchState == 3) {
    if (millis() - lastMillis >= 4000) {
      stopOperation = false;
      pitchState = 4;
      yaw_setpoint = 360;
    }
  } else if (pitchState == 4) {
    if (abs(pid_error) <= 1) {
      setMotorSpeed('A', 0);
      setMotorSpeed('B', 0);
      stopOperation = true;
      pitchState = 5;
      lastMillis = millis();
      pitch = 0;
    }
  } else if (pitchState == 5) {
    if (millis() - lastMillis >= 1000) {
      stopOperation = false;
      pitchState = 6;
    }
  } else if (pitchState == 6) {
    if (pitch <= -20) {
      pitchState = 7;
    }
  } else if (pitchState == 7) {
    if (pitch >= -5) {
      pitchState = 8;
      stopOperation = true;
    }
  }

  // --- MODIFIED: LCD Display with maximum pitch ---
  if (currentTime - lastLCDUpdate > 100) {  // Update LCD every 100ms
    lcd.clear();
    
    // First line: Current Yaw and Pitch
    lcd.setCursor(0, 0);
    lcd.print("Y:");
    lcd.print(yaw, 0);  // Display yaw with 0 decimal places
    
    lcd.setCursor(8, 0);
    lcd.print("Angle:");
    lcd.print(pitch, 0);  // Display current pitch with 0 decimal places
    
    // Second line: Maximum Pitch (Highest Ramp Angle)
    lcd.setCursor(0, 1);
    lcd.print("Max_Angle:");
    lcd.print(maxPitchReached, 0);  // Display maximum pitch with 0 decimal places
    lcd.print((char)223);  // Degree symbol
    
    // Optional: Display state if you want to see what's happening
    // lcd.setCursor(12, 1);
    // lcd.print("S:");
    // lcd.print(pitchState);
    
    lastLCDUpdate = currentTime;
  }
  // --- END MODIFIED ---
}

// Motor Control Function
void setMotorSpeed(char motor, int speed) {
  

//{  if (motor == 'A') {
   // enPin = MOTOR_A_EN;
   // in1Pin = MOTOR_A_IN1;
   // in2Pin = MOTOR_A_IN2;
 // } else if (motor == 'B') {
   // enPin = MOTOR_B_EN;
   // in1Pin = MOTOR_B_IN1;
   // in2Pin = MOTOR_B_IN2;
  //} else {
  //  return;  // Invalid motor
//  }

  if (speed < 0) {
    // Go forward
    digitalWrite(MOTOR_A_IN1, LOW);
    digitalWrite(MOTOR_A_IN2, HIGH);
    digitalWrite(MOTOR_B_IN1, LOW);
    digitalWrite(MOTOR_B_IN2, HIGH);
    analogWrite(MOTOR_A_EN, constrain(abs(speed), 100, 255));
    analogWrite(MOTOR_B_EN, constrain(abs(speed), 100, 255));
  } else if (speed > 0) {
    // Go backward
    digitalWrite(MOTOR_A_IN1, HIGH);
    digitalWrite(MOTOR_A_IN2, LOW);
    digitalWrite(MOTOR_B_IN1, HIGH);
    digitalWrite(MOTOR_B_IN2, LOW);
    analogWrite(MOTOR_A_EN, constrain(abs(speed), 100, 255));
    analogWrite(MOTOR_B_EN, constrain(abs(speed), 100, 255));
  } else {
    // Stop
    digitalWrite(MOTOR_A_IN1, LOW);
    digitalWrite(MOTOR_A_IN2, LOW);
    digitalWrite(MOTOR_B_IN1, LOW);
    digitalWrite(MOTOR_B_IN2, LOW);
    analogWrite(MOTOR_A_EN, 0);
    analogWrite(MOTOR_B_EN, 0);
  }
}

void calculate_IMU_error() {
  // This function calculates the gyro offsets
  c = 0;
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0;
    
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }
  
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  
  c = 0;
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    c++;
  }

  // Calculate the average gyro error
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;

  // Print the errors to Serial Monitor
  Serial.begin(9600);
  Serial.print("AccErrorX: ");
  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY);
  Serial.print("GyroErrorX: ");
  Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(GyroErrorY);
  Serial.print("GyroErrorZ: ");
  Serial.println(GyroErrorZ);
}