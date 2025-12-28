#include <Wire.h>
#include <LiquidCrystal.h>
#include <PinChangeInterrupt.h>

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// Encoders
const uint8_t encoderRightPin = A2;
const uint8_t encoderLeftPin  = A3;

volatile long rightCount = 0;
volatile long leftCount  = 0;

float pulsesPerRev = 20.0;
float wheelDiameter = 6.7;
float wheelCircumference = wheelDiameter * 3.1416;

// Motor pins
int enA = 11, in1 = 12, in2 = 13;
int enB = 3,  in3 = 1,  in4 = 2;

// Line sensors
int L_S = A4;
int R_S = A5;

// Timer
bool timerRunning = false;
unsigned long startTime = 0;
unsigned long stopTime  = 0;

// Stop counter (new)
int stopCounter = 0;

// Forward declarations
void onRightPulse();
void onLeftPulse();

  
void setup() {
  lcd.begin(16, 2);

  // Sensor pins  
  pinMode(L_S, INPUT);
  pinMode(R_S, INPUT);

  // Motor pins
  pinMode(enA, OUTPUT); pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);

  // Encoders
  pinMode(encoderRightPin, INPUT_PULLUP);
  pinMode(encoderLeftPin,  INPUT_PULLUP);

  attachPCINT(digitalPinToPCINT(encoderRightPin), onRightPulse, RISING);
  attachPCINT(digitalPinToPCINT(encoderLeftPin),  onLeftPulse,  RISING);
}



void loop() {
  static unsigned long blackStart = 0;

  int LS = digitalRead(L_S);   // 1 = white, 0 = black
  int RS = digitalRead(R_S);

  // -----------------------------
  // T–JUNCTION STOP LOGIC + stopCounter
  // -----------------------------
  if (LS == 0 || RS == 0) {      // at least one sees black

    if (blackStart == 0)
      blackStart = millis();

    // increment stopCounter whenever black detected
    stopCounter++;

    // Permanent stop after 5 consecutive detections
    if (stopCounter > 5) {
      while (1) {
        Stop();
      }
    }

    // BOTH black for >120ms = STOP
    if (LS == 0 && RS == 0 && (millis() - blackStart > 120)) {
      Stop();

      if (timerRunning) {
        timerRunning = false;
        stopTime = millis();
      }
      return;   // fully stop, skip rest of loop
    }
  }
  else {
    // reset when back on white
    blackStart = 0;
    stopCounter = 0;
  }



  // -----------------------------
  // NORMAL LINE FOLLOWING
  // -----------------------------
  if (RS == 1 && LS == 1) {
    forward();

    if (!timerRunning) {
      timerRunning = true;
      startTime = millis();
    }
  }

  else if (RS == 1 && LS == 0) {
    turnRight();
    delay(100);
  }

  else if (RS == 0 && LS == 1) {
    turnLeft();
    delay(100);
  }



  // -----------------------------
  // DISTANCE CALC
  // -----------------------------
  float rightDistance = (rightCount / pulsesPerRev) * wheelCircumference;
  float leftDistance  = (leftCount  / pulsesPerRev) * wheelCircumference;
  float avgDistance   = (rightDistance + leftDistance) / 2.0;



  // -----------------------------
  // TIME CALC
  // -----------------------------
  float elapsedSec;

  if (timerRunning)
    elapsedSec = (millis() - startTime) / 1000.0;
  else
    elapsedSec = (stopTime - startTime) / 1000.0;



  // -----------------------------
  // LCD DISPLAY
  // -----------------------------
  lcd.setCursor(0, 0);
  lcd.print("Dist:");
  lcd.print(avgDistance, 1);
  lcd.print("cm   ");

  lcd.setCursor(0, 1);
  lcd.print("Time:");
  lcd.print(elapsedSec, 1);
  lcd.print("s   ");
}



// ======================================
// INTERRUPTS
// ======================================
void onRightPulse() { rightCount++; }
void onLeftPulse()  { leftCount++;  }



// ======================================
// MOTOR CONTROL
// ======================================
void forward() {
  digitalWrite(in1, HIGH); 
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, 90);
  analogWrite(enB, 90);
}


void turnRight() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(enA, 230);

  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enB, 120);
}


void turnLeft() {
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enB, 230);

  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(enA, 120);
}


void Stop() {
  digitalWrite(in1, LOW);  
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);  
  digitalWrite(in4, LOW);
  analogWrite(enA, LOW);
  analogWrite(enB, LOW);
}
