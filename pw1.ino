#include <LiquidCrystal.h>

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
// Motor A connections
int enA = 11;
int in1 = 12;
int in2 = 13;
// Motor B connections
int enB = 3;
int in3 = 2;
int in4 = 1;

void setup() {
  lcd.begin(16,2);
  lcd.print("Time (s):");
  // Set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void loop() {
  for (int t = 0; t < 11; t++) {
    lcd.setCursor(0, 1);        // Set cursor to second line of LCD
    lcd.print("        ");      // Clear previous value
    lcd.setCursor(0, 1);        // Set cursor again after clearing
    lcd.print(t);               // Print the seconds value
    if (t<10){
      directionControl();
    }
    else{
      stopMotors();
    }
    delay(1000);
}
    while (true){}
}

// This function lets you control spinning direction of motors
void directionControl() {
  // Set motors to maximum speed
  analogWrite(enA, 253); // Full speed for Motor A
  analogWrite(enB, 255); // Slightly slower for Motor B


  // Turn on motor A & B
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void stopMotors(){
  // Turn off motors
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}