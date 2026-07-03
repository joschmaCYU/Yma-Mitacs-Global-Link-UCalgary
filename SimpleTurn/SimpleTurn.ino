// ==========================================
// PIN DEFINITIONS
// ==========================================

// Motor 1 (Driver 1 - Channel A)
const int M1_PWM = 7;
const int M1_IN1 = 4;
const int M1_IN2 = 5;

// Motor 2 (Driver 1 - Channel B)
const int M2_PWM = 8;
const int M2_IN1 = A5;
const int M2_IN2 = A4;

// Motor 3 (Driver 2 - Channel A)
const int M3_PWM = 11;
const int M3_IN1 = 10;
const int M3_IN2 = 9;

void setup() {
  Serial.begin(115200);
  Serial.println("Starting Motor Test...");

  // Set all motor control pins as OUTPUT
  pinMode(M1_PWM, OUTPUT);
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);

  pinMode(M2_PWM, OUTPUT);
  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);

  pinMode(M3_PWM, OUTPUT);
  pinMode(M3_IN1, OUTPUT);
  pinMode(M3_IN2, OUTPUT);

  // Ensure all motors are stopped initially
  setMotor(1, 0);
  setMotor(2, 0);
  setMotor(3, 0);
}

void loop() {
  // --- TEST SEQUENCE ---
  
  Serial.println("All motors forward at 50% speed");
  setMotor(1, 128); // 128 is approx 50% of 255
  setMotor(2, 128);
  setMotor(3, 128);
  delay(2000);

  Serial.println("Stopping");
  setMotor(1, 0);
  setMotor(2, 0);
  setMotor(3, 0);
  delay(1000);

  Serial.println("All motors reverse at 100% speed");
  setMotor(1, -255);
  setMotor(2, -255);
  setMotor(3, -255);
  delay(2000);

  Serial.println("Stopping");
  setMotor(1, 0);
  setMotor(2, 0);
  setMotor(3, 0);
  delay(3000);
}

// ==========================================
// MOTOR CONTROL HELPER FUNCTION
// ==========================================
// motorNumber: 1, 2, or 3
// speed: -255 to 255 (negative = reverse)
void setMotor(int motorNumber, int speed) {
  int pwmPin, in1Pin, in2Pin;

  // Assign correct pins based on the motor selected
  if (motorNumber == 1) {
    pwmPin = M1_PWM; in1Pin = M1_IN1; in2Pin = M1_IN2;
  } else if (motorNumber == 2) {
    pwmPin = M2_PWM; in1Pin = M2_IN1; in2Pin = M2_IN2;
  } else if (motorNumber == 3) {
    pwmPin = M3_PWM; in1Pin = M3_IN1; in2Pin = M3_IN2;
  } else {
    return; // Invalid motor number
  }

  // Determine direction based on the sign of the speed
  bool in1State, in2State;
  
  if (speed > 0) {
    // Forward
    in1State = HIGH;
    in2State = LOW;
  } else if (speed < 0) {
    // Reverse
    in1State = LOW;
    in2State = HIGH;
    speed = -speed; // Make speed positive for analogWrite
  } else {
    // Stop (Brake)
    in1State = LOW;
    in2State = LOW;
  }

  // Ensure speed does not exceed max PWM value
  if (speed > 255) speed = 255;

  // Apply signals to the driver
  digitalWrite(in1Pin, in1State);
  digitalWrite(in2Pin, in2State);
  analogWrite(pwmPin, speed);
}