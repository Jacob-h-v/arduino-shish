#define USE_ARDUINO_INTERRUPTS true // Set-up low-level interrupts for most accurate bpm math.
#include <PulseSensorPlayground.h>  // Includes the PulseSensorPlayground Library.
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif


const int OUTPUT_TYPE = SERIAL_PLOTTER;

const int PULSE_INPUT = A0;
const int PULSE_BLINK = LED_BUILTIN;
const int PULSE_FADE = 5;
const int THRESHOLD = 475; // ADJUST THIS NUMBER TO AVOID NOISE WHEN IDLE

PulseSensorPlayground pulseSensor;
MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL

#define LED_PIN 13
bool blinkState = false;

// MPU control/status variables
bool dmpReady = false; // set true if DMP(digital motion processing unit) init was succesful
uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU(motion processing unit)
uint8_t devStatus;    // return status after each device operation ( 0 = succeess, !0 = error)
uint16_t packetSize;  // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;   // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO(first in - first out) storage buffer

// Orientation / motion variables
Quaternion q;         // [w, x, y, z]     Quaternion container
VectorInt16 aa;       // [x, y, z]        Accel sensor measurements
VectorInt16 aaReal;    // [x, y, z]        Gravity-free accel sensor measurements
VectorInt16 aaWorld;   // [x, y, z]        World-frame accel sensor measurements
VectorFloat gravity;   // [x, y ,z]        Gravity Vector
float euler[3];       // [psi, theta, phi]  Euler angle container
float ypr[3];         // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

// ---- Interrupt Detection Routine ----
volatile bool mpuInterrupt = false;   // Indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
  mpuInterrupt = true;
}

void setup() {

  // join I2C bus (I2Cdev library doesnt do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    TWBR = 24;    // 400kHz I2C clock (200kHz if CPU is 8MHz)
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    fastwire::setup(400, true);
  #endif

   Serial.begin(115200);

  // Configure the PulseSensor manager.
  pulseSensor.analogInput(PULSE_INPUT);
  pulseSensor.blinkOnPulse(PULSE_BLINK);
  pulseSensor.fadeOnPulse(PULSE_FADE); // Probably dont need this

  pulseSensor.setSerial(Serial);
  pulseSensor.setOutputType(OUTPUT_TYPE);
  pulseSensor.setThreshold(THRESHOLD);

  // Now that everything is ready, start reading the PulseSensor signal.
  if (!pulseSensor.begin())
  {
    /*
       PulseSensor initialization failed,
       likely because our particular Arduino platform interrupts
       aren't supported yet.

       If your Sketch hangs here, try PulseSensor_BPM_Alternative.ino,
       which doesn't use interrupts.
    */
    for(;;)
    {
      // Flash the led to show things didnt work
      digitalWrite(PULSE_BLINK, LOW);
      delay(50);
      digitalWrite(PULSE_BLINK, HIGH);
      delay(50);
    }
  }

  while (!Serial); // wait for a Leonardo enumeration, others continue immediately

  // Initialize device
  Serial.println(F("Initializing I2C Devices..."));
  mpu.initialize();

  // Verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 Connection Succesful") : F("MPU6050 Connection Failed"));

  // Wait for ready
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read());   // empty buffer again

    // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  // ---- OUR OWN GYRO OFFSETS GO HERE, scaled for sensitivity ----
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 might be factory default for chip

  // make sure it worked (return 0 if so)
  if (devStatus == 0)  
  {
    // turn on the DMP, now that it is ready
    Serial.println(F("enabling DMP..."));
    mpu.setDMPEnabled(true);

    //Enable Arduino interrupt Detection
    Serial.println(F("Enabling interrupt detection(Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it is okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // Get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();

  }else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // if an error occours it'll usually be 1    
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // Configure LED for output
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  delay(20);
  
  // Write the latest sample to Serial
  pulseSensor.outputSample();

  /* If a beat has happened since we last checked, write the per-beat information to serial.
  */
  if (pulseSensor.sawStartOfBeat())
  {
    pulseSensor.outputBeat();
  }

  // if programming failed, don't do anything:
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  /*while (!mpuInterrupt && fifoCount < packetSize)
  {
    // can add other programming here. Could be a break; condition
    // to immediately process data if mpuInterrupt is true.

    // this while loop can potentially break the program, just comment it out
    // or add some logic to break it.
  }*/

  // Reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // Get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // Check for overflow (this should never happen unless the code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    // Reset in order to continue running cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO Overflow! FIFO buffer cleared."));

    // Otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02){
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // Read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // Track FIFO count here in case there is > 1 packet available
    // in order to start reading more without waiting for a new interrupt
    fifoCount -= packetSize;

    #ifdef OUTPUT_READABLE_QUATERNION
      // Display Quaternion values in easy matrix form: w x y z
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      Serial.print("quat\t");
      Serial.print(q.w);
      Serial.print("\t");
      Serial.print(q.x);
      Serial.print("\t");
      Serial.print(q.y);
      Serial.print("\t");
      Serial.println(q.z);
    #endif

    #ifdef OUTPUT_READABLE_EULER
      // Display Euler angles in degress
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetEuler(euler, &q);
      Serial.print("euler\t");
      Serial.print(euler[0] * 180/M_PI);
      Serial.print("\t");
      Serial.print(euler[1] * 180/M_PI);
      Serial.print("\t");
      Serial.println(euler[2] * 180/M_PI);
    #endif

    #ifdef OUTPUT_READABLE_YAWPITCHROLL
      // Display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      Serial.print("ypr\t");
      Serial.print(ypr[0] * 180/M_PI);
      Serial.print("\t");
      Serial.print(ypr[1] * 180/M_PI);
      Serial.print("\t");
      Serial.println(ypr[2] * 180/M_PI);
    #endif

    #ifdef OUTPUT_READABLE_REALACCEL
      // Display real acceleration, adjusted to remove gravity
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
      Serial.print("areal\t");
      Serial.print(aaReal.x);
      Serial.print("\t");
      Serial.print(aaReal.y);
      Serial.print("\t");
      Serial.println(aaReal.z);
    #endif

    #ifdef OUTPUT_READABLE_WORLDACCEL
      // Display initial world-frame acceleration, adjusted to remove gravity
      // and rotated based on known orientation from quaternion
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetLinearAccel(&aaReal, &aa, &q);
      Serial.print("aworld\t");
      Serial.print(aaWorld.y);
      Serial.print("\t");
      Serial.print(aaWorld.y);
      Serial.print("\t");
      Serial.println(aaWorld.z);
    #endif

    #ifdef OUTPUT_TEAPOT
      // Display quaternion values in InvenSense Teapot demo format:
      teapotPacket[2] = fifoBuffer[0];
      teapotPacket[3] = fifoBuffer[1];
      teapotPacket[4] = fifoBuffer[4];
      teapotPacket[5] = fifoBuffer[5];
      teapotPacket[6] = fifoBuffer[8];
      teapotPacket[7] = fifoBuffer[9];
      teapotPacket[8] = fifoBuffer[12];
      teapotPacket[9] = fifoBuffer[13];
      Serial.write(teapotPacket, 14);
      teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
    #endif

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }


}
