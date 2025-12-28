#include <LiquidCrystal.h>

// LCD pins
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// === ULTRASONIC SENSOR PINS ===
const int trigPin = A4; 
const int echoPin = A5; 

// === MOTOR DRIVER PINS ===
const int ENA = 11;
const int IN1 = 12;
const int IN2 = 13;
const int ENB = 3;
const int IN3 = 1;
const int IN4 = 2;

// Obstacle Distance Threshold (in cm)
const int OBSTACLE_LIMIT = 20; 

void setup() {
  lcd.begin(16, 2);
  lcd.print("Obstacle Avoider");
  delay(1500);
  lcd.clear();

  // Motor pins
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Ultrasonic Pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {
  // 1. Get Distance
  int distance = getDistance();

  // Update LCD
  lcd.setCursor(0, 0);
  lcd.print("Dist: ");
  lcd.print(distance);
  lcd.print("cm   "); // Extra spaces to clear old numbers

  // 2. Logic: If obstacle is close, Turn. Else, Go Straight.
  if (distance > 0 && distance < OBSTACLE_LIMIT) {
    // === OBSTACLE DETECTED ===
    lcd.setCursor(0, 1);
    lcd.print("Avoiding...     ");
    
    avoidObstacle();
  } 
  else {
    // === PATH CLEAR ===
    lcd.setCursor(0, 1);
    lcd.print("Moving Forward  ");
    
    moveForward();
  }
  
  // Small delay for sensor stability
  delay(50); 
}

// === ULTRASONIC FUNCTION ===
int getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  int distanceCm = duration * 0.034 / 2;
  
  // Filter out zero readings (sometimes sensors give 0 on error)
  if (distanceCm == 0) return 999; 
  return distanceCm;
}

// === AVOIDANCE MANEUVER ===
void avoidObstacle() {
  // 1. Stop immediately
  stop();
  delay(200);

  // 2. Move Backward slightly (prevents getting stuck)
  moveBackward();
  delay(300);

  // 3. Turn Right (or Left) to find a new path
  turnRight();
  delay(600); // Adjust this delay to change how much it turns (e.g., 90 degrees)
  
  // 4. Stop before resuming loop
  stop();
  delay(200);
}

// === MOTOR FUNCTIONS ===
void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 200); // Speed 0-255
  analogWrite(ENB, 200);
}

void moveBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 150);
  analogWrite(ENB, 150);
}

void turnRight() {
  // Left Motor Forward, Right Motor Backward (Spin Turn)
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 200); // Slightly higher speed for turning
  analogWrite(ENB, 200);
}

void stop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}