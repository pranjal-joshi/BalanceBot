
#include <SoftwareSerial.h>
#include <BlynkSimpleSerialBLE.h>
#include <NewPing.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <PID_v1.h>
#include "HMC5883L.h"

// Pin configuation..............................
#define BT_BAUD 9600
#define BUILTIN_LED 13
#define MPU_VCC_PIN A0
#define SONAR_VCC_PIN A1

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
#define MPU_SKIP_INTR 20

// PID constants
#define kp 7
#define ki 1
#define kd 0.05
#define PID_SAMPLE_TIME 5   //ms
#define ANGLE_OFFSET 0.3
#define ORIG_SETPOINT 0
//...........................................................

bool SERIAL_DEBUG = true;
double input,output;
double setpoint = 0;

SoftwareSerial bt(4,3);   // Rx,Tx
BlynkTimer timer;
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DIST);
MPU6050 mpu;
HMC5883L compass;
PID pid(&input,&output,&setpoint,kp,ki,kd,DIRECT);

// Blynk token from email.
char auth[] = "bcb21e631c55432b998b24a9e387a540";
unsigned int sonarReading;

bool dmpReady = false;
volatile bool mpuIntr = false;
volatile byte mpuIntrCnt = 0;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize, fifoCount;
uint8_t fifoBuffer[64];
float ypr[3];
Quaternion q;
VectorFloat gravity;

int16_t mx,my,mz;

void(*resetFunction)(void) = 0;

void setup()
{
   if(SERIAL_DEBUG)
  {
    Serial.begin(38400);
    Serial.println(F("Starting balance bot sketch..."));
  }

  pinMode(MPU_VCC_PIN,OUTPUT);
  digitalWrite(MPU_VCC_PIN,HIGH);
  pinMode(SONAR_VCC_PIN,OUTPUT);
  digitalWrite(SONAR_VCC_PIN,HIGH);
  bootBlink();
  bt.begin(BT_BAUD);
  initMotors();
  initMpu();
  initCompass();
  initPID();
  Blynk.begin(bt,auth);
   timer.setInterval(667L, readSonar);
   timer.setInterval(500L, sendPitch);
   timer.setInterval(1333L, readCompass);
}

void loop()
{
  if(!dmpReady)     // if MPU fails
  {
    return;
  }
  while(!mpuIntr && fifoCount < packetSize)   // Non-MPU things here..
  {
    /*
    Blynk.run();
    timer.run();
    balance();
    */
    // do nothing here...
  }
  mpuIntr = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  if((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    mpu.resetFIFO();
    if(SERIAL_DEBUG)
      Serial.println(F("Resetting FIFO.."));
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
    ypr[1] = ypr[1] * 180/M_PI;
    input = ypr[1];
    
    if(SERIAL_DEBUG)
    {
      //Serial.print("Pitch: ");
      //Serial.println(ypr[1]);
    }
    Blynk.run();
    timer.run();
    balance();
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
  if(SERIAL_DEBUG)
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
    setpoint = ORIG_SETPOINT - ANGLE_OFFSET;
  }
  else if((y < 128) && (y < x))
  {
    set_speed(abs((y-128)*2));
    rev();
    setpoint = ORIG_SETPOINT + ANGLE_OFFSET;
  }
  else if((x < 128) && (x < y))
  {
    set_speed(abs((x-128)*2));
     left();
     setpoint = ORIG_SETPOINT;
  }
   else
   {
     set_speed(abs((x-128)*2));
     right();
     setpoint = ORIG_SETPOINT;
   }
}

BLYNK_WRITE(V3)       // Reset arduino through app
{
  unsigned int val = param.asInt();
  if(val == 1)
  {
    if(SERIAL_DEBUG)
    {
      Serial.println(F("Resetting in 3 seconds..."));
    }
    resetBlink();
    resetFunction();
  }
}

// Sensors to Blynk PUSH
void readSonar()
{
   sonarReading = sonar.ping_cm();
  Blynk.virtualWrite(V1,sonarReading);
}

void sendPitch()
{
  Blynk.virtualWrite(V2,ypr[1]);
}

void readCompass()
{
  compass.getHeading(&mx,&my,&mz);
  float head = atan2(my,mx);
  if(head < 0)
    head += 2 * M_PI;
  head = head *180/M_PI;
  Blynk.virtualWrite(V4, head);
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
  mpuIntrCnt++;
  if(mpuIntrCnt > MPU_SKIP_INTR)
  {
    mpuIntrCnt = 0;
    mpuIntr = true;
  }
}

void initMpu()
{
  Wire.begin();
  Wire.setClock(400000);      // set I2C bus frequency.
  mpu.initialize();
  pinMode(INTR_PIN, INPUT);
  if(SERIAL_DEBUG)
  {
     if(!mpu.testConnection())
     {
        Serial.println(F("*** MPU6050 connection failure."));
        while(1)
          errorBlink();
     }
     else
      Serial.println(F("*** MPU6050 connected."));
  }
  devStatus = mpu.dmpInitialize();
  
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);

  if(devStatus == 0)
  {
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTR_PIN),dmpDataReady,RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    if(SERIAL_DEBUG)
      Serial.println(F("*** DMP initialized."));
  }
  else
  {
    if(SERIAL_DEBUG)
      Serial.println(F("MPU-DMP initialization failed."));
    while(1)
      errorBlink();
  }
}

void initCompass()
{
  compass.initialize();
  if(SERIAL_DEBUG)
    Serial.println(compass.testConnection() ? "*** Compass connected." : "*** Compass failed.");
}

// PID & Control routines ---------------------

void initPID()
{
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-127,127);
  pid.SetSampleTime(PID_SAMPLE_TIME);
}

void balance()
{
  pid.Compute();
  if(output < -0.01)
  {
    rev();
    set_speed(abs(output*2));
  }
  else if(output > 0.01)
  {
    fwd();
    set_speed(output*2);
  }
  else              // motor cut off if balanced.
  {
    analogWrite(en1,0);
    analogWrite(en2,0);
    setpoint = ORIG_SETPOINT;
  }
  if(SERIAL_DEBUG)
  {
    Serial.print(output*2);
    Serial.print("\t");
    String k;
    output > 0? k="fwd\t" : k="rev\t";
    Serial.print(k);
    Serial.println(input);
  }
}

