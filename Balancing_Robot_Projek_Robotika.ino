#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "BluetoothSerial.h"
#include <ESP32Servo.h>

#define INT_PIN_IMU 23              
#define PID_MIN -255             
#define PID_MAX 255                
#define PID_SAMPLE 10     
#define SPEED_MIN 0  
          
#define PITCH_KP 20              
#define PITCH_KI 70              
#define PITCH_KD 0.8

Servo Hitech;
Servo SG;

MPU6050 mpu;
byte Activated;

int YAW_KP = 0;
int YAW_KI = 0;
int YAW_KD = 0;

BluetoothSerial SerialBT;

unsigned long lastTime = 0;
unsigned long currentTime;
float lastPitchError = 0, lastYawError = 0;
float pitchIntegral = 0, yawIntegral = 0;

const unsigned long interval = 10; 

const int Buz = 26;
const int melody[] = { 262, 294, 330, 349, 392, 440, 494, 523 };
const int noteDurations[] = { 4, 4, 4, 4, 4, 4, 4, 4 };

const int IN1 = 2, IN2 = 4, ENA = 15, ENB = 5, IN3 = 16, IN4 = 17;

volatile bool mpuInterrupt = false;

float Setpoint = 0;
float pitchGyroAngle = 0;
float pitchPIDOutput = 0;
float SetpointYaw = 0;
float yawGyroRate = 0;
float yawPIDOutput = 0;
float output;
float derivative;
float error;

const int pinHitech = 18;
const int pinSG = 19;
int setServo = 0;

bool dmpReady = false;  
uint8_t mpuIntStatus;   
uint8_t devStatus;      
uint16_t packetSize;    
uint16_t fifoCount;     
uint8_t fifoBuffer[64]; 

Quaternion q;           
VectorFloat gravity;    
float ypr[3];           
VectorInt16 gy;         

void dmpDataReady() {
    mpuInterrupt = true;
}

void setupServo() {
    ESP32PWM::allocateTimer(0);  // Allocate timer 0 for PWM.
    ESP32PWM::allocateTimer(1);  // Allocate timer 1 for PWM.
    ESP32PWM::allocateTimer(2);  // Allocate timer 2 for PWM.
    ESP32PWM::allocateTimer(3);  // Allocate timer 3 for PWM.
    Hitech.setPeriodHertz(50);  // Set PWM frequency to 50 Hz (standard for servos).
    Hitech.attach(pinHitech, 500, 2400);
    SG.setPeriodHertz(50);  // Set PWM frequency to 50 Hz (standard for servos).
    SG.attach(pinSG, 500, 2400);
}


void setupMotors()
{
  pinMode(ENA,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  
  pinMode(ENB,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);

  rotateMotor(0,0);
}

void setupMPU()
{
  Wire.begin();
  Wire.setClock(400000);
  mpu.initialize();
  pinMode(INT_PIN_IMU, INPUT);
  devStatus = mpu.dmpInitialize();
//-1426.00000,	3229.00000,	1464.00000,	-126.00000,	345.00000,	232.00000
  mpu.setXAccelOffset(-1458); 
  mpu.setYAccelOffset(3263); 
  mpu.setZAccelOffset(1472);   
  mpu.setXGyroOffset(-123);
  mpu.setYGyroOffset(344);
  mpu.setZGyroOffset(232);
//-1034.00000,	-381.00000,	1148.00000,	95.00000,	24.00000,	-4.00000
  // mpu.setXAccelOffset(-1034); 
  // mpu.setYAccelOffset(-381); 
  // mpu.setZAccelOffset(1148);   
  // mpu.setXGyroOffset(95);
  // mpu.setYGyroOffset(24);
  // mpu.setZGyroOffset(-4);
  if (devStatus == 0) 
  {
      mpu.setDMPEnabled(true);
      mpuIntStatus = mpu.getIntStatus();
      dmpReady = true;
      packetSize = mpu.dmpGetFIFOPacketSize();
  }
}

void rotateMotor(int speed1, int speed2)
{
  if (speed1 < 0) {
    digitalWrite(IN1,HIGH);
    digitalWrite(IN2,LOW);    
  } else {
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,HIGH);      
  }

  if (speed2 < 0) {
    digitalWrite(IN3,LOW);
    digitalWrite(IN4,HIGH);    
  } else {
    digitalWrite(IN3,HIGH);
    digitalWrite(IN4,LOW);      
  }

  speed1 = abs(speed1) + SPEED_MIN;
  speed2 = abs(speed2) + SPEED_MIN;

  speed1 = constrain(speed1, SPEED_MIN, 255);
  speed2 = constrain(speed2, SPEED_MIN, 255);
  
  analogWrite(ENA,speed1);
  analogWrite(ENB,speed2); 
}

float computePID(float setpoint, float input, float &lastError, float &integral, float Kp, float Ki, float Kd)
{
  error = setpoint - input;
  if (output > 10 || output < -10)  error += output * 0.015;

  integral += error * (PID_SAMPLE / 1000.0);
  derivative = (error - lastError) / (PID_SAMPLE / 1000.0);
  output = (Kp * error) + (Ki * integral) + (Kd * derivative);
  
  if (pitchGyroAngle > 35 || pitchGyroAngle < -35 || Activated == 0){          
    output = 0;
    integral = 0;                                                         
    Activated = 0;
    Setpoint = 0;
    analogWrite(ENA,0);
    analogWrite(ENB,0); 
  } 

  lastError = error;
  if (output < 10 && output > - 10)  output = 0;
  return constrain(output, PID_MIN, PID_MAX);
}

void Remote() {
  if (SerialBT.available() > 0) { 
    uint8_t value = SerialBT.read(); 
    // Serial.println(value);
    switch (value) {
    case 1:  // Maju
        if (Setpoint < 2.5){
          for(int i = 0; i < 6; i++){
            Setpoint -= 0.8;
          }
        }
        break;
        
    case 2:  // Mundur
        if (Setpoint > -2.5){
        for(int i = 0; i < 6; i++){
          Setpoint += 0.8;
          }
        }
        break;
        
    case 3:  // Belok kiri
        YAW_KP = 1;
        SetpointYaw = -35; 
        break;
        
    case 4:  // Belok kanan
        YAW_KP = 1;
        SetpointYaw = 35; 
        break;

    case 5:  // gripset turun
        setServo = 1;
        // status_gripper = 0;  
        // Setpoint = 0;
        // Hitech.write(90); //Naik
        // Hitech.write(20); //Turun
        // Hitech.write(90); //Buka 
        // Hitech.write(50); //Tutup
        break;

      case 6:  // gripset naik
        setServo = 2;
        // status_gripper = 1;
        // Setpoint = -5.5;
        // Hitech.write(80); //Naik
        // Hitech.write(30); //Turun
        // Hitech.write(90); //Buka 
        // Hitech.write(33); //Tutup
        break;

      case 7:  // grip tutup
        // Hitech.write(90); //Buka 
        // Hitech.write(33); //Tutup
        setServo = 3;
        break;

      case 8:  // grip buka
        // status_gripper = 0;
        // Setpoint = 0;
        // Hitech.write(90); //Buka 
        // Hitech.write(50); //Tutup
        setServo = 4;
        break;    
        
    case 0:  // Netral / Stop
        if (output > 10 || output < -10)  error += output * 0.015;
        YAW_KP = 0;
        Setpoint = 0;  
        SetpointYaw = 0;  
        break;
    }
  }
}


void buzzer() {
  pinMode(Buz, OUTPUT);
  for (int i = 0; i < 8; i++) {
    int noteDuration = 1000 / noteDurations[i];
    tone(Buz, melody[i], noteDuration);

    int pauseBetweenNotes = noteDuration * 1.3;
    delay(pauseBetweenNotes);

    noTone(Buz);
  }
}

void setup(){
  Serial.begin(9600);
  SerialBT.begin("HT : BRku");
  buzzer();
  setupMPU();
  setupMotors();
  setupServo();
}

void loop()
{
  if (!dmpReady) return;

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) 
  {  
    // mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  // mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4); 
  // mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8); 
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.dmpGetGyro(&gy, fifoBuffer);

    yawGyroRate = gy.z;
    pitchGyroAngle = ypr[2] * 180/M_PI;

    pitchPIDOutput = computePID(Setpoint, pitchGyroAngle, lastPitchError, pitchIntegral, PITCH_KP, PITCH_KI, PITCH_KD);
    yawPIDOutput = computePID(SetpointYaw, yawGyroRate, lastYawError, yawIntegral, YAW_KP, YAW_KI, YAW_KD);

    rotateMotor(pitchPIDOutput - yawPIDOutput, pitchPIDOutput + yawPIDOutput);
    Remote();
  }
  if (Activated == 0 && pitchGyroAngle > -1.5 && pitchGyroAngle < 1.5) {  
    Activated = 1;   
  }
  // Serial.println(pitchGyroAngle);
  if (setServo == 1) Hitech.write(100);
  else if (setServo == 2) Hitech.write(80);
  else if (setServo == 3) SG.write(50);
  else if (setServo == 4) SG.write(30);
  else {
    Hitech.write(100);
    SG.write(30);
  }
  
  
  
}