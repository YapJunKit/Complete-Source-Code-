  /* * FULL BLUETOOTH CAR CODE 
  * Optimized for Restricted Pins (1, 2, 3, 11, 12, 13)
  */

  #include <SoftwareSerial.h>

  // ---------- 1. BLUETOOTH CONNECTION ----------
  // Arduino A5 (RX) -> Connect to HC-05 TX
  // Arduino A4 (TX) -> Connect to HC-05 RX
  SoftwareSerial myBT(A5, A4); 

  // ---------- 2. MOTOR PINS ----------
  // Left Motor
  #define enA 11  // PWM Speed Control
  #define in1 12   // WARNING: You MUST unplug this wire when uploading code!
  #define in2 13

  // Right Motor
  #define enB 3   // PWM Speed Control
  #define in3 2
  #define in4 1  // WARNING: Wheel will spin briefly on startup (LED pin)

  // ---------- 3. SETTINGS ----------
  int carSpeed = 150; // Speed range is 0 to 255. 150 is good for testing.
  char command;       // Variable to store the letter coming from the App

  void setup() {
    // NOTE: We do NOT use Serial.begin() because Pin 1 is used for the motor.
    
    myBT.begin(9600); // Start Bluetooth communication

    // Set all motor pins to Output
    pinMode(enA, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);

    pinMode(enB, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);

    // SAFETY: Stop motors immediately to handle Pin 13 startup twitch
    stopCar();
    
    // Wait 2 seconds before accepting commands 
    // This gives you time to put the car on the floor after turning it on
    delay(2000); 
  }

  // ------------ MAIN LOOP ------------
  void loop() {
    // Check if phone sent a command
    if (myBT.available()) {
      command = myBT.read(); // Read the letter
      
      // Check which letter was sent and move accordingly
      if (command == 'F') {
        forward();
      }
      else if (command == 'B') {
        backward();
      }
      else if (command == 'L') {
        left();
      }
      else if (command == 'R') {
        right();
      }
      else if (command == 'S' || command == '0') {
        stopCar();
      }
    }
  }

  // ------------ MOTOR FUNCTIONS ------------

  void forward() {
    // Left Motor Forward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enA, carSpeed);

    // Right Motor Forward
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enB, carSpeed);
  }

  void backward() {
    // Left Motor Backward
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enA, carSpeed);

    // Right Motor Backward
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enB, carSpeed);
  }

  void left() {
    // TANK TURN LEFT: Left wheels back, Right wheels forward
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enA, carSpeed);

    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enB, carSpeed);
  }

  void right() {
    // TANK TURN RIGHT: Left wheels forward, Right wheels back
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enA, carSpeed);

    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enB, carSpeed);
  }

  void stopCar() {
    // Turn everything OFF
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(enA, 0);

    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    analogWrite(enB, 0);
  }