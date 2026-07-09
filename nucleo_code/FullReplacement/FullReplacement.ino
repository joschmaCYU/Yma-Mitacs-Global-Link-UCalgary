#include <SPI.h>
#include <math.h>
#include <complex>  

// ==========================================
// CONFIGURATION MATHÉMATIQUE & MÉCANIQUE
// ==========================================
#define ENCODER_MAX_VALUE 4095.0f
#define GEAR_RATIO (3224.0f / 24.0f)
#define DEGTOSTEP ((ENCODER_MAX_VALUE + 1.0f) * GEAR_RATIO / 360.0f)

// ==========================================
// SPI BUS (MOSI, MISO, SCK)
// ==========================================
SPIClass SPI_1(PA7, PA6, PA1);
SPIClass SPI_2(PC3, PC2, PB10);
SPIClass SPI_3(PC12, PC11, PC10);

// Les paramètres SPI déduits de ton ancien code CubeMX :
// Polarity HIGH + Phase 1 Edge = SPI_MODE2
// Fréquence à 5 MHz (largement suffisant pour 32 bits)
SPISettings spiSettings(5000000, MSBFIRST, SPI_MODE2);

float Ts = 0.0009f; // Temps d'échantillonnage de la boucle PID (900 microsecondes)

// ==========================================
// PIN DEFINITIONS
// ==========================================
const int M1_PWM = PA8;  const int M1_IN1 = PB5;  const int M1_IN2 = PB4;
const int M3_PWM = PA9;  const int M3_IN1 = PC0;  const int M3_IN2 = PC1;
const int M2_PWM = PA11; const int M2_IN1 = PB12; const int M2_IN2 = PB11;

const int E1_A = PA6;    const int E1_B = PA1;   
const int E2_A = PC2;    const int E2_B = PB10;
const int E3_A = PC11;   const int E3_B = PC10;

const int STYB1 = PC6; // À adapter selon ton câblage exact
const int STYB2 = PB8;

// ==========================================
// STRUCTURES
// ==========================================
typedef struct {
    double yaw;
    double pitch;
    double roll;
} Orientation;

typedef struct {
    double theta1;
    double theta2;
    double theta3;
} DiskAngles;

struct Quaternion {
    double q0, q1, q2, q3;
};

typedef struct {
    float Kp, Ki, Kd;
    float tau;
    float limMin, limMax;
    float limMinInt, limMaxInt;
    float T;
    
    float proportional;
    float integrator;
    float prevError;
    float differentiator;
    float prevMeasurement;
    float out;
} PIDController;

typedef struct {
    volatile long position;
    long old_position;
    float speed;
    
    long target_position;
    float target_speed;
    int duty_cycle;

    int16_t pos_enc;
    int16_t old_pos_enc;
    int number_turn;
    int16_t initial_pos_enc;
    
    PIDController pid_position;
    PIDController pid_speed;
} Motor;

// ==========================================
// VARIABLES GLOBALES
// ==========================================
Motor mot1, mot2, mot3;
bool calibrationMot = false;
unsigned long previousLoopMicros = 0;
unsigned long previousPrintMillis = 0;

Orientation setpoint_orientation = {0, 0, 0};
Orientation old_orientation = {0, 0, 0};
DiskAngles disk_angles = {0, 0, 0};

// ==========================================
// FONCTIONS DE LECTURE SSI
// ==========================================
int16_t read_encoder_spi(SPIClass &spi, uint8_t *buf) {
    spi.beginTransaction(spiSettings);
    for(int i=0; i<4; i++) buf[i] = spi.transfer(0x00);
    spi.endTransaction();

    unsigned char bitBuffer = 0;
    int16_t encoderposition = 0;
    int bitCount = 0;
    bool foundStart = false;

    for (int i = 0; i < 32; i++) {
        int bit = (buf[i / 8] >> (7 - (i % 8))) & 0x01;
        bitBuffer = ((bitBuffer << 1) | bit) & 0x07;
        if (!foundStart && bitBuffer == 0b010) { foundStart = true; continue; }
        if (foundStart && bitCount < 12) { encoderposition = (encoderposition << 1) | bit; bitCount++; }
    }
    return foundStart ? encoderposition : -1;
}

void update_motor_turns(Motor *m) {
    if (m->pos_enc == -1) return; // Sécurité : on ignore si le SPI bugge sur un tour

    if (m->pos_enc < 1000 && m->old_pos_enc > 3000) m->number_turn++;
    else if (m->pos_enc > 3000 && m->old_pos_enc < 1000) m->number_turn--;
    
    // On soustrait la position de départ pour démarrer à un vrai Zéro
    m->position = (m->number_turn * 4096) + m->pos_enc - m->initial_pos_enc;
    m->old_pos_enc = m->pos_enc;
}

// ==========================================
// ROUTINES D'INTERRUPTION (ENCODEURS)
// ==========================================
void isr_enc1() { if (digitalRead(E1_B) == HIGH) mot1.position++; else mot1.position--; }
void isr_enc2() { if (digitalRead(E2_A) == HIGH) mot2.position++; else mot2.position--; }
void isr_enc3() { if (digitalRead(E3_B) == HIGH) mot3.position++; else mot3.position--; }

// ==========================================
// FONCTIONS PID
// ==========================================
void PIDController_Init(PIDController *pid) {
    pid->integrator = 0.0f;
    pid->prevError  = 0.0f;
    pid->differentiator  = 0.0f;
    pid->prevMeasurement = 0.0f;
    pid->out = 0.0f;
}

float PIDController_Update(PIDController *pid, float setpoint, float measurement) {
    float error = setpoint - measurement;
    pid->proportional = pid->Kp * error;
    pid->integrator = pid->integrator + pid->Ki * pid->T * pid->prevError;

    if (pid->integrator > pid->limMaxInt) pid->integrator = pid->limMaxInt;
    else if (pid->integrator < pid->limMinInt) pid->integrator = pid->limMinInt;

    pid->differentiator = pid->Kd * (error - pid->prevError);
    pid->out = pid->proportional + pid->integrator + pid->differentiator;

    if (pid->out > pid->limMax) pid->out = pid->limMax;
    else if (pid->out < pid->limMin) pid->out = pid->limMin;

    pid->prevError = error;
    pid->prevMeasurement = measurement;
    return pid->out;
}

void init_motor_pid(Motor *m) {
    // --- PID Vitesse ---
    m->pid_speed.Kp = 0.05f;
    m->pid_speed.Ki = 5.0f;
    m->pid_speed.Kd = 0.01f;
    m->pid_speed.tau = 0.01f;
    m->pid_speed.limMin = -250.0f; // Adapté pour Arduino analogWrite (0-255)
    m->pid_speed.limMax = 250.0f;
    m->pid_speed.limMinInt = -200.0f; // Anti-windup
    m->pid_speed.limMaxInt = 200.0f;
    m->pid_speed.T = Ts;
    PIDController_Init(&(m->pid_speed));

    // --- PID Position ---
    m->pid_position.Kp = 0.5f;
    m->pid_position.Ki = 2.2f;
    m->pid_position.Kd = 0.001f;
    m->pid_position.tau = 0.01f;
    m->pid_position.limMin = -10000.0f;
    m->pid_position.limMax = 10000.0f;
    m->pid_position.limMinInt = -5000.0f;
    m->pid_position.limMaxInt = 5000.0f;
    m->pid_position.T = Ts;
    PIDController_Init(&(m->pid_position));
}

// ==========================================
// CINÉMATIQUE INVERSE
// ==========================================
struct Quaternion ToQuaternion(double roll, double pitch, double yaw) {
    double cr = cos(roll * 0.5);  double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5); double sp = sin(pitch * 0.5);
    double cy = cos(yaw * 0.5);   double sy = sin(yaw * 0.5);
    struct Quaternion q;
    q.q0 = cr * cp * cy + sr * sp * sy;
    q.q1 = sr * cp * cy - cr * sp * sy;
    q.q2 = cr * sp * cy + sr * cp * sy;
    q.q3 = cr * cp * sy - sr * sp * cy;
    return q;
}

DiskAngles computeInverseKinematics(Orientation tool_orientation) {
    DiskAngles da;
    struct Quaternion q = ToQuaternion(tool_orientation.roll * PI / 180.0, 
                                       tool_orientation.pitch * PI / 180.0, 
                                       tool_orientation.yaw * PI / 180.0);
    double e0 = q.q0, e1 = q.q1, e2 = q.q2, e3 = q.q3;
    
    // Traduction en C++ des tableaux et de l'unité imaginaire
    std::complex<float> u[3];
    float angle_val[3] = {0};
    const std::complex<float> i_comp(0.0f, 1.0f); // Unité imaginaire (i)

    for (int n = 0; n < 3; n++) {
        float ap = 5.0 * PI / 18.0;
        float ni = 2.0 * PI / 3.0 * (n - 1);

        float Mj = sin(ap) * (-2*e0*e3 + 2*e1*e2*cos(2*ni) + (-e1*e1 + e2*e2)*sin(2*ni));
        float Nj = sin(ap) * ((e0*e0) - e3*e3 + (-e1*e1 + e2*e2)*cos(2*ni) - 2*e1*e2*sin(2*ni));
        float Pj = -2*cos(ap) * ((e0*e1 + e2*e3)*cos(ni) + (e0*e2 - e1*e3)*sin(ni));

        // Mathématiques complexes en C++
        std::complex<float> innerTerm(Pj * Pj - Mj * Mj - Nj * Nj, 0.0f);
        std::complex<float> squareRoot = std::sqrt(innerTerm);
        std::complex<float> numerator = (-Pj - squareRoot) * (Mj + i_comp * Nj);
        float denominator = Mj * Mj + Nj * Nj;
        
        u[n] = numerator / denominator;
        angle_val[n] = std::arg(u[n]) * 180.0 / PI; 
    }
    
    da.theta1 = angle_val[1];
    da.theta2 = angle_val[2];
    da.theta3 = angle_val[0];
    return da;
}

// ==========================================
// FONCTION MOTEUR ARDUINO
// ==========================================
void setMotor(int motorNumber, int speed) {
    int pwmPin, in1Pin, in2Pin;
    if (motorNumber == 1) { pwmPin = M1_PWM; in1Pin = M1_IN1; in2Pin = M1_IN2; } 
    else if (motorNumber == 2) { pwmPin = M2_PWM; in1Pin = M2_IN1; in2Pin = M2_IN2; } 
    else if (motorNumber == 3) { pwmPin = M3_PWM; in1Pin = M3_IN1; in2Pin = M3_IN2; } 
    else return;

    bool in1State, in2State;
    if (speed > 0) { in1State = LOW; in2State = HIGH; } 
    else if (speed < 0) { in1State = HIGH; in2State = LOW; speed = -speed; } 
    else { in1State = LOW; in2State = LOW; }

    if (speed > 255) speed = 255;
    digitalWrite(in1Pin, in1State);
    digitalWrite(in2Pin, in2State);
    analogWrite(pwmPin, speed);
}

// ==========================================
// SETUP
// ==========================================
void setup() {
    Serial.begin(115200);
    SPI_1.begin(); SPI_2.begin(); SPI_3.begin();
    
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

    init_motor_pid(&mot1);
    init_motor_pid(&mot2);
    init_motor_pid(&mot3);

    Serial.println("Arduino STM32duino ready. Send 'init' to start.");
}

// ==========================================
// LOOP
// ==========================================
void loop() {
    unsigned long currentMicros = micros();
    unsigned long currentMillis = millis();

    // ----------------------------------------------------
    // 1. PARSEUR SÉRIE
    // ----------------------------------------------------
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();

        if (input.equals("init")) {
            Serial.println("Initialisation en cours...");
            noInterrupts();
            
            // On lit la position absolue de chaque encodeur pour créer le Zéro
            uint8_t b1[4], b2[4], b3[4];
            mot1.initial_pos_enc = read_encoder_spi(SPI_1, b1);
            mot2.initial_pos_enc = read_encoder_spi(SPI_2, b2);
            mot3.initial_pos_enc = read_encoder_spi(SPI_3, b3);

            // Si la lecture rate au démarrage, on évite le crash et on met 0
            if (mot1.initial_pos_enc == -1) mot1.initial_pos_enc = 0;
            if (mot2.initial_pos_enc == -1) mot2.initial_pos_enc = 0;
            if (mot3.initial_pos_enc == -1) mot3.initial_pos_enc = 0;

            mot1.old_pos_enc = mot1.initial_pos_enc;
            mot2.old_pos_enc = mot2.initial_pos_enc;
            mot3.old_pos_enc = mot3.initial_pos_enc;

            mot1.number_turn = 0; mot2.number_turn = 0; mot3.number_turn = 0;
            mot1.position = 0; mot2.position = 0; mot3.position = 0;
            mot1.old_position = 0; mot2.old_position = 0; mot3.old_position = 0;
            mot1.target_position = 0; mot2.target_position = 0; mot3.target_position = 0;
            
            PIDController_Init(&(mot1.pid_position)); PIDController_Init(&(mot1.pid_speed));
            PIDController_Init(&(mot2.pid_position)); PIDController_Init(&(mot2.pid_speed));
            PIDController_Init(&(mot3.pid_position)); PIDController_Init(&(mot3.pid_speed));
            
            interrupts();
            calibrationMot = true;
            Serial.println("Calibration terminee ! Moteurs prets.");
        } else if (calibrationMot) {
            int comma1 = input.indexOf(',');
            int comma2 = input.lastIndexOf(',');
            if (comma1 > 0 && comma2 > comma1) {
                setpoint_orientation.yaw = input.substring(0, comma1).toFloat();
                setpoint_orientation.pitch = input.substring(comma1 + 1, comma2).toFloat();
                setpoint_orientation.roll = input.substring(comma2 + 1).toFloat();
                
                // Erreur de parenthèses corrigée ici :
                Serial.print("Recu : Yaw="); Serial.print(setpoint_orientation.yaw);
                Serial.print(", Pitch="); Serial.print(setpoint_orientation.pitch);
                Serial.print(", Roll="); Serial.println(setpoint_orientation.roll);

                disk_angles = computeInverseKinematics(setpoint_orientation);
                mot1.target_position = -disk_angles.theta1 * DEGTOSTEP;
                mot2.target_position = -disk_angles.theta2 * DEGTOSTEP;
                mot3.target_position = -disk_angles.theta3 * DEGTOSTEP;
            } else {
                Serial.println("Format invalide ! Utilisez: Yaw,Pitch,Roll");
            }
        } else {
            Serial.println("Erreur: Envoyez 'init' d'abord.");
        }
    }

    // ----------------------------------------------------
    // 2. BOUCLE DE CONTRÔLE PID (Exécutée toutes les 900 µs)
    // ----------------------------------------------------
    if (currentMicros - previousLoopMicros >= 900) {
        previousLoopMicros = currentMicros;

        if (calibrationMot) {
            // Lecture des encodeurs
            uint8_t p1[4], p2[4], p3[4];
            mot1.pos_enc = read_encoder_spi(SPI_1, p1);
            mot2.pos_enc = read_encoder_spi(SPI_2, p2);
            mot3.pos_enc = read_encoder_spi(SPI_3, p3);

            update_motor_turns(&mot1);
            update_motor_turns(&mot2);
            update_motor_turns(&mot3);

            long pos1_val = mot1.position;
            long pos2_val = mot2.position;
            long pos3_val = mot3.position;

            // Moteur 1
            mot1.speed = (float)(pos1_val - mot1.old_position) * 60.0f / Ts / (ENCODER_MAX_VALUE + 1.0f);
            mot1.old_position = pos1_val;
            mot1.target_speed = PIDController_Update(&(mot1.pid_position), mot1.target_position, mot1.position);
            mot1.duty_cycle = PIDController_Update(&(mot1.pid_speed), mot1.target_speed, mot1.speed);
            
            // Moteur 2
            mot2.speed = (float)(pos2_val - mot2.old_position) * 60.0f / Ts / (ENCODER_MAX_VALUE + 1.0f);
            mot2.old_position = pos2_val;
            mot2.target_speed = PIDController_Update(&(mot2.pid_position), mot2.target_position, mot2.position);
            mot2.duty_cycle = PIDController_Update(&(mot2.pid_speed), mot2.target_speed, mot2.speed);

            // Moteur 3
            mot3.speed = (float)(pos3_val - mot3.old_position) * 60.0f / Ts / (ENCODER_MAX_VALUE + 1.0f);
            mot3.old_position = pos3_val;
            mot3.target_speed = PIDController_Update(&(mot3.pid_position), mot3.target_position, mot3.position);
            mot3.duty_cycle = PIDController_Update(&(mot3.pid_speed), mot3.target_speed, mot3.speed);

            // Application de la puissance
            setMotor(1, mot1.duty_cycle);
            setMotor(2, mot2.duty_cycle);
            setMotor(3, mot3.duty_cycle);
        } else {
            setMotor(1, 0); setMotor(2, 0); setMotor(3, 0);
        }
    }

    // ----------------------------------------------------
    // 3. ESPION (Exécuté toutes les 100 ms)
    // ----------------------------------------------------
    if (calibrationMot && (currentMillis - previousPrintMillis >= 100)) {
        previousPrintMillis = currentMillis;

        Serial.print("Cible1:");     Serial.print(mot1.target_position); Serial.print(",");
        Serial.print("Cible2:");     Serial.print(mot2.target_position); Serial.print(",");
        Serial.print("Cible3:");     Serial.print(mot3.target_position); Serial.print(",");

        
        Serial.print("Pos_MOT1:");  Serial.print(mot1.position);  Serial.print(",");
        Serial.print("Pos_MOT2:");  Serial.print(mot2.position);  Serial.print(",");
        Serial.print("Pos_MOT3:");  Serial.print(mot3.position);  Serial.print(",");

        Serial.print("Vit_MOT1:");  Serial.print((int)mot1.speed);  Serial.print(",");
        Serial.print("Vit_MOT2:");  Serial.print((int)mot2.speed);  Serial.print(",");
        Serial.print("Vit_MOT3:");  Serial.print((int)mot3.speed);  Serial.print(",");

        Serial.print("PWM_MOT1:");  Serial.print(mot1.duty_cycle);  Serial.print(",");
        Serial.print("PWM_MOT2:");  Serial.print(mot2.duty_cycle);  Serial.print(",");
        Serial.print("PWM_MOT3:");  Serial.println(mot3.duty_cycle);
        
        // char buffer[100];
        // sprintf(buffer, "MOT1 | Cible:%ld | Pos:%ld | Vit:%d | PWM:%d", , , , );
        // Serial.println(buffer);
        // sprintf(buffer, "MOT2 | Cible:%ld | Pos:%ld | Vit:%d | PWM:%d", , , , mot2.duty_cycle);
        // Serial.println(buffer);
        // sprintf(buffer, "MOT3 | Cible:%ld | Pos:%ld | Vit:%d | PWM:%d", mot3.target_position, mot3.position, (int)mot3.speed, mot3.duty_cycle);
        // Serial.println(buffer);
    }
}