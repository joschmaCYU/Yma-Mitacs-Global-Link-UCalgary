#include <ros.h>
#include <mitacs/Motor.h>
#include <geometry_msgs/Vector3.h> // Pour recevoir les consignes Yaw/Pitch/Roll

#include <SPI.h>
#include <math.h>
#include <complex>  

// ==========================================
// CONFIGURATION ROS
// ==========================================
ros::NodeHandle nh;

mitacs::Motor msg_mot;
ros::Publisher pub_motor("motor", &msg_mot);

// ==========================================
// CONFIGURATION MATHÉMATIQUE & MÉCANIQUE
// ==========================================
#define ENCODER_MAX_VALUE 4095.0f
#define GEAR_RATIO (3224.0f / 24.0f)
#define DEGTOSTEP ((ENCODER_MAX_VALUE + 1.0f) * GEAR_RATIO / 360.0f)

SPIClass SPI_1(PA7, PA6, PA5);
SPIClass SPI_2(PB15, PB14, PB13);
SPIClass SPI_3(PC12, PC11, PC10);

SPISettings spiSettings(5000000, MSBFIRST, SPI_MODE2);

float Ts = 0.0009f;

// ==========================================
// PIN DEFINITIONS
// ==========================================
const int M1_PWM = PA8;  const int M1_IN1 = PB5;  const int M1_IN2 = PB4;
const int M3_PWM = PA9;  const int M3_IN1 = PC0;  const int M3_IN2 = PC1;
const int M2_PWM = PA11; const int M2_IN1 = PB12; const int M2_IN2 = PB11;

const int STYB1 = PC6; 
const int STYB2 = PB8;

// ==========================================
// STRUCTURES
// ==========================================
typedef struct { double yaw; double pitch; double roll; } Orientation;
typedef struct { double theta1; double theta2; double theta3; } DiskAngles;
struct Quaternion { double q0, q1, q2, q3; };

typedef struct {
    float Kp, Ki, Kd, tau, limMin, limMax, limMinInt, limMaxInt, T, proportional, integrator, prevError, differentiator, prevMeasurement, out;
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

Motor mot1, mot2, mot3;
bool calibrationMot = false;
unsigned long previousLoopMicros = 0;
unsigned long previousPubMillis = 0;

Orientation setpoint_orientation = {0, 0, 0};
DiskAngles disk_angles = {0, 0, 0};

// ==========================================
// FONCTIONS MÉCANIQUES & MATHÉMATIQUES
// ==========================================
// (Garde tes fonctions intactes ici)
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
    if (m->pos_enc == -1) return; 
    if (m->pos_enc < 1000 && m->old_pos_enc > 3000) m->number_turn++;
    else if (m->pos_enc > 3000 && m->old_pos_enc < 1000) m->number_turn--;
    m->position = (m->number_turn * 4096) + m->pos_enc - m->initial_pos_enc;
    m->old_pos_enc = m->pos_enc;
}

void PIDController_Init(PIDController *pid) {
    pid->integrator = 0.0f; pid->prevError  = 0.0f; pid->differentiator  = 0.0f;
    pid->prevMeasurement = 0.0f; pid->out = 0.0f;
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
    m->pid_speed.Kp = 0.05f; m->pid_speed.Ki = 5.0f; m->pid_speed.Kd = 0.01f; m->pid_speed.tau = 0.01f;
    m->pid_speed.limMin = -250.0f; m->pid_speed.limMax = 250.0f; m->pid_speed.limMinInt = -200.0f; m->pid_speed.limMaxInt = 200.0f;
    m->pid_speed.T = Ts; PIDController_Init(&(m->pid_speed));

    m->pid_position.Kp = 0.5f; m->pid_position.Ki = 2.2f; m->pid_position.Kd = 0.001f; m->pid_position.tau = 0.01f;
    m->pid_position.limMin = -10000.0f; m->pid_position.limMax = 10000.0f; m->pid_position.limMinInt = -5000.0f; m->pid_position.limMaxInt = 5000.0f;
    m->pid_position.T = Ts; PIDController_Init(&(m->pid_position));
}

struct Quaternion ToQuaternion(double roll, double pitch, double yaw) {
    double cr = cos(roll * 0.5);  double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5); double sp = sin(pitch * 0.5);
    double cy = cos(yaw * 0.5);   double sy = sin(yaw * 0.5);
    struct Quaternion q;
    q.q0 = cr * cp * cy + sr * sp * sy; q.q1 = sr * cp * cy - cr * sp * sy;
    q.q2 = cr * sp * cy + sr * cp * sy; q.q3 = cr * cp * sy - sr * sp * cy;
    return q;
}

DiskAngles computeInverseKinematics(Orientation tool_orientation) {
    DiskAngles da;
    struct Quaternion q = ToQuaternion(tool_orientation.roll * PI / 180.0, tool_orientation.pitch * PI / 180.0, tool_orientation.yaw * PI / 180.0);
    double e0 = q.q0, e1 = q.q1, e2 = q.q2, e3 = q.q3;
    std::complex<float> u[3]; float angle_val[3] = {0};
    const std::complex<float> i_comp(0.0f, 1.0f);
    for (int n = 0; n < 3; n++) {
        float ap = 5.0 * PI / 18.0; float ni = 2.0 * PI / 3.0 * (n - 1);
        float Mj = sin(ap) * (-2*e0*e3 + 2*e1*e2*cos(2*ni) + (-e1*e1 + e2*e2)*sin(2*ni));
        float Nj = sin(ap) * ((e0*e0) - e3*e3 + (-e1*e1 + e2*e2)*cos(2*ni) - 2*e1*e2*sin(2*ni));
        float Pj = -2*cos(ap) * ((e0*e1 + e2*e3)*cos(ni) + (e0*e2 - e1*e3)*sin(ni));
        std::complex<float> innerTerm(Pj * Pj - Mj * Mj - Nj * Nj, 0.0f);
        std::complex<float> squareRoot = std::sqrt(innerTerm);
        std::complex<float> numerator = (-Pj - squareRoot) * (Mj + i_comp * Nj);
        float denominator = Mj * Mj + Nj * Nj;
        u[n] = numerator / denominator; angle_val[n] = std::arg(u[n]) * 180.0 / PI; 
    }
    da.theta1 = angle_val[1]; da.theta2 = angle_val[2]; da.theta3 = angle_val[0];
    return da;
}

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
    digitalWrite(in1Pin, in1State); digitalWrite(in2Pin, in2State); analogWrite(pwmPin, speed);
}

// ==========================================
// CALLBACK CIBLES ROS
// ==========================================
void targetCb(const geometry_msgs::Vector3& msg) {
    // msg.x = Yaw, msg.y = Pitch, msg.z = Roll
    setpoint_orientation.yaw = msg.x;
    setpoint_orientation.pitch = msg.y;
    setpoint_orientation.roll = msg.z;
    
    disk_angles = computeInverseKinematics(setpoint_orientation);
    mot1.target_position = -disk_angles.theta1 * DEGTOSTEP;
    mot2.target_position = -disk_angles.theta2 * DEGTOSTEP;
    mot3.target_position = -disk_angles.theta3 * DEGTOSTEP;
}
ros::Subscriber<geometry_msgs::Vector3> sub_target("target_orientation", &targetCb);

// ==========================================
// SETUP
// ==========================================
void setup() {
    SPI_1.begin(); 
    SPI_1.beginTransaction(spiSettings);
    SPI_2.begin(); 
    SPI_2.beginTransaction(spiSettings);
    SPI_3.begin();
    SPI_3.beginTransaction(spiSettings);
    
    pinMode(M1_PWM, OUTPUT); pinMode(M1_IN1, OUTPUT); pinMode(M1_IN2, OUTPUT);
    pinMode(M2_PWM, OUTPUT); pinMode(M2_IN1, OUTPUT); pinMode(M2_IN2, OUTPUT);
    pinMode(M3_PWM, OUTPUT); pinMode(M3_IN1, OUTPUT); pinMode(M3_IN2, OUTPUT);
    pinMode(STYB1, OUTPUT); pinMode(STYB2, OUTPUT);
    digitalWrite(STYB1, HIGH); digitalWrite(STYB2, HIGH);

    init_motor_pid(&mot1); init_motor_pid(&mot2); init_motor_pid(&mot3);

    // AUTO-CALIBRATION AU DÉMARRAGE
    delay(1000); // Laisse le temps aux capteurs de s'allumer
    uint8_t b1[4], b2[4], b3[4];
    mot1.initial_pos_enc = read_encoder_spi(SPI_1, b1);
    mot2.initial_pos_enc = read_encoder_spi(SPI_2, b2);
    mot3.initial_pos_enc = read_encoder_spi(SPI_3, b3);

    if (mot1.initial_pos_enc == -1) mot1.initial_pos_enc = 0;
    if (mot2.initial_pos_enc == -1) mot2.initial_pos_enc = 0;
    if (mot3.initial_pos_enc == -1) mot3.initial_pos_enc = 0;

    mot1.old_pos_enc = mot1.initial_pos_enc; mot2.old_pos_enc = mot2.initial_pos_enc; mot3.old_pos_enc = mot3.initial_pos_enc;
    mot1.position = 0; mot2.position = 0; mot3.position = 0;
    mot1.old_position = 0; mot2.old_position = 0; mot3.old_position = 0;
    
    calibrationMot = true;

    // ROS INIT (STRICTEMENT AUCUN SERIAL)
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.advertise(pub_motor);
    nh.subscribe(sub_target);
}

// ==========================================
// LOOP
// ==========================================
void loop() {
    unsigned long currentMicros = micros();
    unsigned long currentMillis = millis();

    // 1. BOUCLE PID
    if (currentMicros - previousLoopMicros >= 900) {
        previousLoopMicros = currentMicros;

        if (calibrationMot) {
            uint8_t p1[4], p2[4], p3[4];
            mot1.pos_enc = read_encoder_spi(SPI_1, p1);
            mot2.pos_enc = read_encoder_spi(SPI_2, p2);
            mot3.pos_enc = read_encoder_spi(SPI_3, p3);

            update_motor_turns(&mot1); update_motor_turns(&mot2); update_motor_turns(&mot3);

            long pos1_val = mot1.position; long pos2_val = mot2.position; long pos3_val = mot3.position;

            mot1.speed = (float)(pos1_val - mot1.old_position) * 60.0f / Ts / (ENCODER_MAX_VALUE + 1.0f);
            mot1.old_position = pos1_val;
            mot1.target_speed = PIDController_Update(&(mot1.pid_position), mot1.target_position, mot1.position);
            mot1.duty_cycle = PIDController_Update(&(mot1.pid_speed), mot1.target_speed, mot1.speed);
            
            mot2.speed = (float)(pos2_val - mot2.old_position) * 60.0f / Ts / (ENCODER_MAX_VALUE + 1.0f);
            mot2.old_position = pos2_val;
            mot2.target_speed = PIDController_Update(&(mot2.pid_position), mot2.target_position, mot2.position);
            mot2.duty_cycle = PIDController_Update(&(mot2.pid_speed), mot2.target_speed, mot2.speed);

            mot3.speed = (float)(pos3_val - mot3.old_position) * 60.0f / Ts / (ENCODER_MAX_VALUE + 1.0f);
            mot3.old_position = pos3_val;
            mot3.target_speed = PIDController_Update(&(mot3.pid_position), mot3.target_position, mot3.position);
            mot3.duty_cycle = PIDController_Update(&(mot3.pid_speed), mot3.target_speed, mot3.speed);

            setMotor(1, mot1.duty_cycle); setMotor(2, mot2.duty_cycle); setMotor(3, mot3.duty_cycle);
        }
    }

    // 2. PUBLICATION ROS (50 Hz)
    if ((currentMillis - previousPubMillis >= 20)) {
        previousPubMillis = currentMillis;
        
        msg_mot.name = "MOT1"; msg_mot.position = mot1.position; msg_mot.speed = mot1.speed; msg_mot.duty_cycle = mot1.duty_cycle;
        pub_motor.publish(&msg_mot);

        msg_mot.name = "MOT2"; msg_mot.position = mot2.position; msg_mot.speed = mot2.speed; msg_mot.duty_cycle = mot2.duty_cycle;
        pub_motor.publish(&msg_mot);

        msg_mot.name = "MOT3"; msg_mot.position = mot3.position; msg_mot.speed = mot3.speed; msg_mot.duty_cycle = mot3.duty_cycle;
        pub_motor.publish(&msg_mot);
    }
    
    // Indispensable pour traiter les réceptions et l'état de la connexion
    nh.spinOnce();
}