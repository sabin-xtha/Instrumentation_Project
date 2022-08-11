// Wiring in Arduino is as follows:
// Interrupt--->>2
// SCL---->>A5
// SDA---->>A6

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "motor.h"


// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL

#define INTERRUPT_PIN 2 // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13		// (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define MOTOR1_PIN1 4
#define MOTOR1_PIN2 7
#define MOTOR1_PWMPIN 5
#define MOTOR2_PIN1 8
#define MOTOR2_PIN2 9
#define MOTOR2_PWMPIN 6
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;	// set true if DMP init was successful
uint8_t mpuIntStatus;	// holds actual interrupt status byte from MPU
uint8_t devStatus;		// return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;	// expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;		// count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orentation/motion vars
Quaternion q;		 // [w, x, y, z]         quaternion container
VectorFloat gravity; // [x, y, z]            gravity vector
float ypr[3];		 // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//motor variables
motor _motor;
Motor_config motor_conf;

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
	mpuInterrupt = true;
}




// ==================================================================
// ===               			SETUP				              ===
// ==================================================================

void setup()
{
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

	// initialize device
	MPUinit();

	// intialize motor
	motorsetup();

	// configure LED for output
	pinMode(LED_PIN, OUTPUT);
}


// ==================================================================
// ===               			LOOP				              ===
// ==================================================================

void loop()
{
	runmotor(&_motor);
	if (!dmpReady)
		return;
	
	readMPUdata();
}






void I2Cinit(){
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	Wire.begin();
	Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
	Fastwire::setup(400, true);
#endif
}

void MPUinit(){
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
	mpu.setXGyroOffset(102);
	mpu.setYGyroOffset(-14);
	mpu.setZGyroOffset(-15);
  	mpu.setXAccelOffset(-1871);
  	mpu.setYAccelOffset(-527);
	mpu.setZAccelOffset(507); // 1688 factory default for my test chip

	// make sure it worked (returns 0 if so)
	if (devStatus == 0)
	{
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
	}
	else
	{
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		Serial.print(F("DMP Initialization failed (code "));
		Serial.print(devStatus);
		Serial.println(F(")"));
	}

}

void readMPUdata(){
	// read a packet from FIFO
	if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
	{ // Get the Latest packet
		// display gyro angles	 in degrees
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
		Serial.print("ypr\t");
		Serial.print(ypr[0] * 180 / M_PI);
		Serial.print("\t");
		Serial.print(ypr[1] * 180 / M_PI);
		Serial.print("\t");
		Serial.println(ypr[2] * 180 / M_PI);
		// blink LED to indicate activity
		blinkState = !blinkState;
		digitalWrite(LED_PIN, blinkState);
	}
}


void motorsetup(){
	 motor_config_init();
    _motor = motor(&motor_conf);
    _motor.motors_init();
}

void motor_config_init(){
    motor_conf.motor1.outputpin1=4;
    motor_conf.motor1.outputpin2=7;
    motor_conf.motor1.speedpin=5;
    motor_conf.motor1.speed=255;
    motor_conf.motor1.direction=CLOCKWISE;
	 
 

    motor_conf.motor2.outputpin1=8;
    motor_conf.motor2.outputpin2=9;
    motor_conf.motor2.speedpin=6;
    motor_conf.motor2.speed=255;
    motor_conf.motor1.direction=CLOCKWISE;
}