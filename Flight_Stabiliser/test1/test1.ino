#include <Servo.h>
#include "I2Cdev.h"
#include <PID_v1.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h" 
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high
/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

//OUTPUTS
Servo sAileron, sElevator, sRudder, sThrottle;
int pinA = 4, pinE = 5, pinT = 6, pinR = 7;
int inA = 8, inE = 9, inT = 10, inR = 11, inAux1 = 12;

//PRESETS
#define SWITCH_THRESHOLD 1500
#define XGO 220
#define YGO 76
#define ZGO -85
#define ZAO 1788
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards



//******************************PID******************************

double setPitch = 90, setRoll = 90, pitchOut, rollOut;
double doubleP, doubleR;
double Kp=2, Ki=5, Kd=1;
PID pitchPID(&doubleP, &pitchOut, &setPitch, Kp, Ki, Kd, DIRECT);
PID rollPID(&doubleR, &rollOut, &setRoll, Kp, Ki, Kd, DIRECT);


//***************************************************************


void setup() {
  sAileron.attach(pinA);
  sElevator.attach(pinE);
  sThrottle.attach(pinT);
  sRudder.attach(pinR);
  pinMode(inA,INPUT);
  pinMode(inE,INPUT);
  pinMode(inT,INPUT);
  pinMode(inR,INPUT);
  pinMode(inAux1,INPUT);

  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  // initialize serial communication
  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately
    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    // wait for ready
    Serial.println(F("\nBeginning now\n "));
    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(XGO);
    mpu.setYGyroOffset(YGO);
    mpu.setZGyroOffset(ZGO);
    mpu.setZAccelOffset(ZAO);
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
  pitchPID.SetMode(AUTOMATIC);
  rollPID.SetMode(AUTOMATIC);
}

void loop() {  
  if (!dmpReady) return; //If the device does not initialise, exit immediately 
  while(!mpuInterrupt && fifoCount < packetSize && pulseIn(inAux1,HIGH)<SWITCH_THRESHOLD) {
    //Manual Mode
      sAileron.writeMicroseconds(pulseIn(inA,HIGH));
      sElevator.writeMicroseconds(pulseIn(inE,HIGH));
      sRudder.writeMicroseconds(pulseIn(inR,HIGH));  
      sThrottle.writeMicroseconds(pulseIn(inT,HIGH));
  }
  //Stabilise Mode
  //Stabilise in 2 axes
  sRudder.write(90);
  sThrottle.writeMicroseconds(pulseIn(inT,HIGH));
  //calculate error
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
  }else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);  
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;
  }
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  Serial.print("ypr\t");
  Serial.print(ypr[0] * 180/M_PI);
  Serial.print("\t");
  Serial.print(ypr[1] * 180/M_PI);
  Serial.print("\t");
  Serial.println(ypr[2] * 180/M_PI);
  doubleP = (double) (ypr[1]*180/M_PI + 180)/2;
  doubleR = (double) (ypr[2]*180/M_PI + 180)/2;
  //calculate correction value 
  pitchPID.Compute();
  rollPID.Compute();
  
  //write correction value 
  sAileron.write(rollOut);
  sElevator.write(pitchOut);    
}
