
#include <SoftwareSerial.h>
#include <BlynkSimpleSerialBLE.h>
#include <NewPing.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

// Pin configuation..............................
#define BT_BAUD 9600
#define BUILTIN_LED 13

// SONAR
#define TRIG_PIN 11
#define ECHO_PIN 12
#define MAX_DIST 300

// Motor driver
#define en1 5
#define en2 6
#define l1 7
#define l2 8
#define r1 9
#define r2 10

// MPU6050
#define INTR_PIN 2
//...........................................................

bool SERIAL_PRINT = true;

SoftwareSerial bt(4,3);   // Rx,Tx
BlynkTimer timer;
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DIST);
MPU6050 mpu;

// Blynk token from email.
char auth[] = "bcb21e631c55432b998b24a9e387a540";
unsigned int sonarReading;

bool dmpReady = false;
volatile bool mpuIntr = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize, fifoCount;
uint8_t fifoBuffer[64];
float ypr[3];
Quaternion q;
VectorFloat gravity;

void(*resetFunction)(void) = 0;

void setup()
{
   if(SERIAL_PRINT)
  {
    Serial.begin(9600);
    Serial.println("Starting balance bot sketch...");
  }
  
  bootBlink();
  bt.begin(BT_BAUD);
  initMotors();
  initMpu();
  Blynk.begin(bt,auth);
   timer.setInterval(1000L,readSonar);
}

void loop()
{
  if(!dmpReady)     // if MPU fails
  {
    return;
  }
  while(!mpuIntr && fifoCount < packetSize)   // Non-MPU things here..
  {
    Blynk.run();
    timer.run();
  }
  mpuIntr = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  if((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    mpu.resetFIFO();
  }
  else if(mpuIntStatus & 0x02)
  {
    while(fifoCount < packetSize)
    {
      fifoCount = mpu.getFIFOCount();
    }
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
    mpu.dmpGetQuaternion(&q,fifoBuffer);
    mpu.dmpGetGravity(&gravity,&q);
    mpu.dmpGetYawPitchRoll(ypr,&q,&gravity);
    for(byte k=0;k<3;k++)
    {
      ypr[k] = ypr[k] * 180/M_PI;
    }
    
  }
  
}

// BLYNK interface routines -------------------------------

BLYNK_CONNECTED()
{
  Blynk.syncAll();
}

BLYNK_WRITE(V0)         // read joystick data over bluetooth
{
  unsigned int x = param[0].asInt();
  unsigned int y = param[1].asInt();
  if(SERIAL_PRINT)
  {
    Serial.print("X:\t");
    Serial.print(x,DEC);
    Serial.print("\tY:\t");
    Serial.println(y,DEC);
  }
  if((y > 128) && (y > x))
  {
    set_speed(abs((y-128)*2));
    fwd();
  }
  else if((y < 128) && (y < x))
  {
    set_speed(abs((y-128)*2));
    rev();
  }
  else if((x < 128) && (x < y))
  {
    set_speed(abs((x-128)*2));
     left();
  }
   else
   {
     set_speed(abs((x-128)*2));
     right();
   }
}

BLYNK_WRITE(V3)       // Reset arduino through app
{
  unsigned int val = param.asInt();
  if(val == 1)
  {
    if(SERIAL_PRINT)
    {
      Serial.println("Resetting in 3 seconds...");
    }
    resetBlink();
    resetFunction();
  }
}

unsigned int readSonar()
{
   sonarReading = sonar.ping_cm();
  Blynk.virtualWrite(V1,sonarReading);
}


// Notification LED routines --------------------------

void resetBlink()
{
  for(byte i=0;i<3;i++)
  {
    digitalWrite(BUILTIN_LED, 1);
    delay(500);
    digitalWrite(BUILTIN_LED, 0);
    delay(500);
  }
}

void bootBlink()
{
  pinMode(BUILTIN_LED, OUTPUT);
  for(byte i=0;i<2;i++)
  {
    digitalWrite(BUILTIN_LED, 1);
    delay(50);
    digitalWrite(BUILTIN_LED, 0);
    delay(50);
  }
}

void errorBlink()
{
  for(byte k=0;k<3;k++)
  {
    digitalWrite(BUILTIN_LED, 1);
    delay(70);
    digitalWrite(BUILTIN_LED, 0);
    delay(70);
  }
  delay(500);
}

// MOTOR CONTROL routines ---------------------

void initMotors()
{
  pinMode(en1,OUTPUT);
  pinMode(en2,OUTPUT);
  pinMode(l1,OUTPUT);
  pinMode(l2,OUTPUT);
  pinMode(r1,OUTPUT);
  pinMode(r2,OUTPUT);
}

void left_fwd()
{
  digitalWrite(l1,HIGH);
  digitalWrite(l2,LOW);
}

void left_rev()
{
  digitalWrite(l1,LOW);
  digitalWrite(l2,HIGH);
}

void right_fwd()
{
  digitalWrite(r1,HIGH);
  digitalWrite(r2,LOW);
}

void right_rev()
{
  digitalWrite(r1,LOW);
  digitalWrite(r2,HIGH);
}

void fwd()
{
  left_fwd();
  right_fwd();
}

void rev()
{
  left_rev();
  right_rev();
}

void left()
{
  left_rev();
  right_fwd();
}

void right()
{
  left_fwd();
  right_rev();
}

void set_speed(unsigned int val)
{
  analogWrite(en1,val);
  analogWrite(en2,val);
}

// MPU6050 IMU routines -------------------

void dmpDataReady()           // DMP ISR
{
  mpuIntr = true;
}

void initMpu()
{
  Wire.begin();
  Wire.setClock(400000);
  mpu.initialize();
  pinMode(INTR_PIN, INPUT);
  if(SERIAL_PRINT)
  {
     if(!mpu.testConnection())
     {
        Serial.println("*** MPU6050 connection failure.");
        while(1)
          errorBlink();
     }
  }
  devStatus = mpu.dmpInitialize();
  
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1688);

  if(devStatus == 0)
  {
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTR_PIN),dmpDataReady,RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
  {
    if(SERIAL_PRINT)
      Serial.println("MPU-DMP initialization failed.");
    while(1)
      errorBlink();
  }
}

