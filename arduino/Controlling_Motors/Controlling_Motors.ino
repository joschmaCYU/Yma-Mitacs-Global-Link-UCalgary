// ==========================================
// PIN DEFINITIONS (MOTEURS)
// ==========================================
const int M1_PWM = PA8;  const int M1_IN1 = PB5;  const int M1_IN2 = PB4;
const int M2_PWM = PA9;  const int M2_IN1 = PC0; const int M2_IN2 = PC1;
const int M3_PWM = PA11; const int M3_IN1 = PB12; const int M3_IN2 = PB11;

// ==========================================
// PIN DEFINITIONS (ENCODEURS)
// ==========================================
const int E1_A = PA6;   const int E1_B = PA1;   
const int E2_A = PC2;   const int E2_B = PB10;
const int E3_A = PC11;  const int E3_B = PC10;

const int STYB1 = PC6;  // Ajout des pins manquantes si nécessaire pour compilation
const int STYB2 = PB8;

volatile long pos1 = 0;
volatile long pos2 = 0;
volatile long pos3 = 0;

const float TICKS_PER_REV = 341.2; 

// Variables pour la gestion du temps de mouvement et d'affichage
unsigned long previousMotorMillis = 0;
unsigned long previousLogMillis = 0;
bool movingForward = true;
const int TEST_SPEED = 50; // Vitesse de test (0-255)

void isr_enc1() {
  if (digitalRead(E1_B) == HIGH) pos1++; else pos1--;
}
void isr_enc2() { 
  if (digitalRead(E2_A) == HIGH) pos2++; else pos2--;
}
void isr_enc3() {
  if (digitalRead(E3_B) == HIGH) pos3++; else pos3--;
}

void setup() {
  Serial.begin(115200);

  pinMode(M1_PWM, OUTPUT); pinMode(M1_IN1, OUTPUT); pinMode(M1_IN2, OUTPUT);
  pinMode(M2_PWM, OUTPUT); pinMode(M2_IN1, OUTPUT); pinMode(M2_IN2, OUTPUT);
  pinMode(M3_PWM, OUTPUT); pinMode(M3_IN1, OUTPUT); pinMode(M3_IN2, OUTPUT);

  pinMode(STYB1, OUTPUT); pinMode(STYB2, OUTPUT);
  digitalWrite(STYB1, HIGH);
  digitalWrite(STYB2, HIGH);

  pinMode(E1_A, INPUT_PULLUP); pinMode(E1_B, INPUT_PULLUP);
  pinMode(E2_A, INPUT_PULLUP); pinMode(E2_B, INPUT_PULLUP);
  pinMode(E3_A, INPUT_PULLUP); pinMode(E3_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(E1_A), isr_enc1, RISING);
  attachInterrupt(digitalPinToInterrupt(E2_B), isr_enc2, RISING);
  attachInterrupt(digitalPinToInterrupt(E3_A), isr_enc3, RISING);

  // Initialisation du premier mouvement
  setMotor(1, TEST_SPEED);
  setMotor(2, TEST_SPEED);
  setMotor(3, TEST_SPEED);
  previousMotorMillis = millis();
}

void loop() {
  unsigned long currentMillis = millis();

  // 1. Gestion de l'alternance avant/arrière toutes les 1000 ms (1s)
  if (currentMillis - previousMotorMillis >= 500) {
    previousMotorMillis = currentMillis;
    movingForward = !movingForward; // Inverse la direction

    int speedToApply = movingForward ? TEST_SPEED : -TEST_SPEED;
    
    setMotor(1, speedToApply);
    setMotor(2, speedToApply);
    setMotor(3, speedToApply);
  }

  // 2. Publication des données toutes les 20 ms (50 Hz) sans bloquer les moteurs
  if (currentMillis - previousLogMillis >= 20) {
    previousLogMillis = currentMillis;

    noInterrupts();
    long p1 = pos1;
    long p2 = pos2;
    long p3 = pos3;
    interrupts();   

    float angle1 = (p1 / TICKS_PER_REV) * 2.0 * PI;
    float angle2 = (p2 / TICKS_PER_REV) * 2.0 * PI;
    float angle3 = (p3 / TICKS_PER_REV) * 2.0 * PI;

    Serial.print(angle1, 4);
    Serial.print(",");
    Serial.print(angle2, 4);
    Serial.print(",");
    Serial.println(angle3, 4);
  }
}

void setMotor(int motorNumber, int speed) {
  int pwmPin, in1Pin, in2Pin;

  if (motorNumber == 1) {
    pwmPin = M1_PWM; in1Pin = M1_IN1; in2Pin = M1_IN2;
  } else if (motorNumber == 2) {
    pwmPin = M2_PWM; in1Pin = M2_IN1; in2Pin = M2_IN2;
  } else if (motorNumber == 3) {
    pwmPin = M3_PWM; in1Pin = M3_IN1; in2Pin = M3_IN2;
  } else {
    return;
  }

  bool in1State, in2State;
  
  if (speed > 0) {
    in1State = HIGH;
    in2State = LOW;
  } else if (speed < 0) {
    in1State = LOW;
    in2State = HIGH;
    speed = -speed;
  } else {
    in1State = LOW;
    in2State = LOW;
  }

  if (speed > 255) speed = 255;

  digitalWrite(in1Pin, in1State);
  digitalWrite(in2Pin, in2State);
  analogWrite(pwmPin, speed);
}