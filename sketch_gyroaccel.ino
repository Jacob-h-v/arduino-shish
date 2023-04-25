#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;

#define LED_PIN 13
bool blinkState = false;

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorInt16 aaWorld;
VectorFloat gravity;
float gyroX;
float gyroY;
float gyroZ;
float euler[3];
float ypr[3];

bool newDataAvailable = false;

void setup() {
  Serial.begin(115200);

  Wire.begin();
  TWBR = 24;

  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setZAccelOffset(1788);

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  if (dmpReady) {
    fifoCount = mpu.getFIFOCount();
    if (fifoCount >= packetSize) {
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize;

      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      newDataAvailable = true;
    }
  }


  if (newDataAvailable && Serial.available() > 0) {
    //Serial.println("Uno received: " + incomingChar);
    char incomingChar = Serial.read();
    if (incomingChar == 'e') {
      gyroX = (ypr[0] * 180/M_PI);
      gyroY = (ypr[1] * 180/M_PI);
      gyroZ = (ypr[2] * 180/M_PI);
      String gyroXStr = String(gyroX, 2);
      String gyroYStr = String(gyroY, 2);
      String gyroZStr = String(gyroZ, 2);

      String dataToSend = gyroXStr + "\t" + gyroYStr + "\t" + gyroZStr + "\n";
      Serial.print(dataToSend);

      newDataAvailable = false;
    }
  }
}
