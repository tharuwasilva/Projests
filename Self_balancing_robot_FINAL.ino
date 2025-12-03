//#define PRINT_DEBUG_BUILD  // Enable this to print MPU data on Serial Monitor for debugging

// PID library
#include <PID_v1.h>

// Bluetooth (SoftwareSerial)
#include <SoftwareSerial.h>
SoftwareSerial BT(11, 12);  // RX, TX  (connect HC-05 TX -> Arduino 11, HC-05 RX -> Arduino 12 with voltage divider)

// These are needed for MPU
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif

// class default I2C address is 0x68
MPU6050 mpu;
#define INTERRUPT_PIN 2  // (not used for attachInterrupt in this sketch)

// MPU control/status vars
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

// orientation/motion vars
Quaternion q;
VectorFloat gravity;
float ypr[3];
VectorInt16 gy;

// INTERRUPT DETECTION ROUTINE (kept for completeness)
volatile bool mpuInterrupt = false;
void dmpDataReady() { mpuInterrupt = true; }

// PID limits and timing
#define PID_MIN_LIMIT -255
#define PID_MAX_LIMIT 255
#define PID_SAMPLE_TIME_IN_MILLI 10

// Balance offsets and motor min
#define SETPOINT_PITCH_ANGLE_OFFSET -0.2
#define MIN_ABSOLUTE_SPEED 0

// Global state variables
double setpointPitchAngle = SETPOINT_PITCH_ANGLE_OFFSET;
double pitchGyroAngle = 0;
double pitchPIDOutput = 0;

double setpointYawRate = 0;
double yawGyroRate = 0;
double yawPIDOutput = 0;

// PID tuning (use your values)
#define PID_PITCH_KP 75
#define PID_PITCH_KI 35
#define PID_PITCH_KD 3

#define PID_YAW_KP 1.6
#define PID_YAW_KI 0.1
#define PID_YAW_KD 0

PID pitchPID(&pitchGyroAngle, &pitchPIDOutput, &setpointPitchAngle, PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, DIRECT);
PID yawPID(&yawGyroRate, &yawPIDOutput, &setpointYawRate, PID_YAW_KP, PID_YAW_KI, PID_YAW_KD, DIRECT);

// Motor pins
int enableMotor1 = 9;
int motor1Pin1   = 5;
int motor1Pin2   = 6;

int motor2Pin1   = 7;
int motor2Pin2   = 8;
int enableMotor2 = 10;

// Bluetooth command variable (global)
char btCommand = '0';  // '0' = stop / balanced

// Function prototype (fixes "not declared in this scope")
void rotateMotor(int speed1, int speed2);

// ---------- Setup helper functions ----------
void setupPID()
{
  pitchPID.SetOutputLimits(PID_MIN_LIMIT, PID_MAX_LIMIT);
  pitchPID.SetMode(AUTOMATIC);
  pitchPID.SetSampleTime(PID_SAMPLE_TIME_IN_MILLI);

  yawPID.SetOutputLimits(PID_MIN_LIMIT, PID_MAX_LIMIT);
  yawPID.SetMode(AUTOMATIC);
  yawPID.SetSampleTime(PID_SAMPLE_TIME_IN_MILLI);
}

void setupMotors()
{
  pinMode(enableMotor1, OUTPUT);
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);

  pinMode(enableMotor2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);

  // ensure motors are stopped at boot
  rotateMotor(0, 0);
}

void setupMPU()
{
  // join I2C bus
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000);
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  devStatus = mpu.dmpInitialize();

  // Use your calibrated offsets
  mpu.setXAccelOffset(196);
  mpu.setYAccelOffset(-3421);
  mpu.setZAccelOffset(380);
  mpu.setXGyroOffset(41);
  mpu.setYGyroOffset(2);
  mpu.setZGyroOffset(2);

  if (devStatus == 0)
  {
    mpu.setDMPEnabled(true);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
  {
    // DMP init failed: you can handle error here (flash LED, etc.)
  }
}

// ---------- Arduino core ----------
void setup()
{
  // Motors, MPU, PID
  setupMotors();
  setupMPU();
  setupPID();

  // Serials
  Serial.begin(115200); // optional debug
  BT.begin(9600);       // HC-05 default baud

  // Small delay for stable start
  delay(200);
}

void loop()
{
  // If MPU not ready, do nothing
  if (!dmpReady) return;

  // --- Read Bluetooth command if any ---
  if (BT.available())
  {
    btCommand = BT.read(); // correct read from software serial

    // Interpret numeric commands:
    // '1' = forward, '2' = backward, '3' = left, '4' = right, '0' = stop/balance
    if (btCommand == '1') {
      setpointPitchAngle = SETPOINT_PITCH_ANGLE_OFFSET + 5.0; // tilt forward
      setpointYawRate = 0;
    }
    else if (btCommand == '2') {
      setpointPitchAngle = SETPOINT_PITCH_ANGLE_OFFSET - 5.0; // tilt backward
      setpointYawRate = 0;
    }
    else if (btCommand == '3') {
      // keep forward/back setpoint neutral, request yaw
      setpointPitchAngle = SETPOINT_PITCH_ANGLE_OFFSET;
      setpointYawRate = -50.0; // turn left (tweak magnitude as needed)
    }
    else if (btCommand == '4') {
      setpointPitchAngle = SETPOINT_PITCH_ANGLE_OFFSET;
      setpointYawRate = 50.0; // turn right
    }
    else if (btCommand == '0') {
      setpointPitchAngle = SETPOINT_PITCH_ANGLE_OFFSET;
      setpointYawRate = 0;
    }
    // else: ignore unknown characters
  }

  // --- Read MPU data and compute PID ---
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
  {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.dmpGetGyro(&gy, fifoBuffer);

    // update sensor-derived variables
    yawGyroRate = gy.z;                  // degrees/sec (raw DMP units often map; check if units need scaling)
    pitchGyroAngle = ypr[1] * 180.0 / M_PI; // degrees

    // compute PIDs
    pitchPID.Compute();
    yawPID.Compute();

    // send outputs to motors (mix pitch + yaw)
    int outLeft  = (int)(pitchPIDOutput + yawPIDOutput);
    int outRight = (int)(pitchPIDOutput - yawPIDOutput);

    rotateMotor(outLeft, outRight);

    #ifdef PRINT_DEBUG_BUILD
      Serial.print("Pitch: "); Serial.print(pitchGyroAngle);
      Serial.print("  Setpt: "); Serial.print(setpointPitchAngle);
      Serial.print("  Pout: "); Serial.print(pitchPIDOutput);
      Serial.print("  Yrate: "); Serial.print(yawGyroRate);
      Serial.print("  Yout: "); Serial.println(yawPIDOutput);
      delay(50);
    #endif
  }
}

// ---------- Motor control ----------
void rotateMotor(int speed1, int speed2)
{
  // set directions
  if (speed1 < 0) {
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
  } else {
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
  }

  if (speed2 < 0) {
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
  } else {
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
  }

  // absolute speeds with minimum offset
  int s1 = abs(speed1) + MIN_ABSOLUTE_SPEED;
  int s2 = abs(speed2) + MIN_ABSOLUTE_SPEED;

  s1 = constrain(s1, MIN_ABSOLUTE_SPEED, 255);
  s2 = constrain(s2, MIN_ABSOLUTE_SPEED, 255);

  analogWrite(enableMotor1, s1);
  analogWrite(enableMotor2, s2);
}
