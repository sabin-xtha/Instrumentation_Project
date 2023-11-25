// Wiring in Arduino is as follows:
// Interrupt--->>2
// SCL---->>A5
// SDA---->>A6

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
// #include "motor.h"
#include "PID_v1.h"
#include <AFMotor.h>

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13       // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;   // set true if DMP init was successful
uint8_t mpuIntStatus;    // holds actual interrupt status byte from MPU
uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;      // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];  // FIFO storage buffer

// orentation/motion vars
Quaternion q;         // [w, x, y, z]         quaternion container
VectorFloat gravity;  // [x, y, z]            gravity vector
float ypr[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// motor variables
// motor _motor;
// Motor_config motor_conf;

// PID values
double orignalSetpoint = 173;
double setpoint = 0;
double movingAngleOffset = 0.1;
double input, output;

// tuning values
double kp = 20;

double ki = 0;
double kd = 0;
PID pid(&input, &output, &setpoint, kp, ki, kd, P_ON_E, DIRECT);

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;  // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

// ==================================================================
// ===               			SETUP				              ===
// ==================================================================



AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
void forward(int speed) {
  motor1.setSpeed(speed);  //Define maximum velocity
  motor1.run(FORWARD);     //rotate the motor clockwise
  motor2.setSpeed(speed);  //Define maximum velocity
  motor2.run(FORWARD);     //rotate the motor clockwise
}

void backward(int speed) {
  motor1.setSpeed(speed);
  motor1.run(BACKWARD);  //rotate the motor counterclockwise
  motor2.setSpeed(speed);
  motor2.run(BACKWARD);  //rotate the motor counterclockwise
}

void Stop() {
  motor1.setSpeed(0);
  motor1.run(RELEASE);  //turn motor1 off
  motor2.setSpeed(0);
  motor2.run(RELEASE);  //turn motor2 off
}


void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  I2Cinit();

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  Serial.begin(115200);

  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
  // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.

  // initialize device ::::: pid is setup inside it

  MPUinit();

  // intialize motor
  // motorsetup();

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
}

// ==================================================================
// ===               			LOOP				              ===
// ==================================================================

void loop() {
  if (!dmpReady)
    return;
  while (!mpuInterrupt && fifoCount < packetSize) {
    pid.Compute();
    Serial.print(input);
    Serial.print("---->");

    Serial.println(output);
    // if (input > -60 && input < 60) {
    output > 0 ? forward(output) : backward(-output);
    // }else{
    //   Stop();
    // }
  }
  readMPUdata();
}

void I2Cinit() {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
}

void MPUinit() {
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(93);
  mpu.setYGyroOffset(1);
  mpu.setZGyroOffset(35);
  mpu.setXAccelOffset(1526);
  mpu.setYAccelOffset(2509);
  mpu.setZAccelOffset(-1044);  // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();

    // setup pid
    pidsetup();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.print(F(")"));
  }
}

void readMPUdata() {
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize)
      fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    input = ypr[1] * 180 / M_PI;  // + 180;
    // Serial.println(input);
  }
}
void pidsetup() {
  pid.SetOutputLimits(-255, 255);
  pid.SetSampleTime(10);
  pid.SetTunings(kp, ki, kd);
  pid.SetMode(AUTOMATIC);
}