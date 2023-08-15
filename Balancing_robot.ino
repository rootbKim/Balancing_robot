#include <Streaming.h>  // only for simple print
#include <SoftwareSerial.h>
#include "TB6612.h"
#include "motorPID.h"
#include "QEncoder.h"
#include "MPU9250_SPI.h"
#include "eeprom_utils.h"
#include "parameters.h"

//#define RAW_DATA
//#define CALIBRATION_MODE

MPU9250_SPI mpu(SPI,MPU_CS,INT_PIN);
SoftwareSerial BT(8, 10); // RX | TX
EEPROM_BACKUP eeprom;
TB6612 motorA(APWM, AIN1, AIN2);  //create a motor instance
TB6612 motorB(BPWM, BIN1, BIN2);  //create a motor instance
rPID  pidA(0.05, 0, 0.21, 255);     // kp, ki, kd,imax
rPID  pidB(0.05, 0, 0.21,  255);     // kp, ki, kd,imax
rPID  bala(150, 3500, 9.9, 5000);     // 1차 성공 200, 2040,5,5000 840 ok
//rPID  bala(180, 4750, 12.2, 5000);     // 1차 성공 200, 2040,5,5000 840 ok
rPID  movePID(0.08,0,0.07,3);
rPID  rotateA(100, 3, 5, 10);
Encoders encoderA(ENA_A,ENA_B), encoderB(ENB_A,ENB_B);      // Create an Encoder instance (2,3) (1,0)


uint32_t prevTime=0,intVal=2000, pT=0;
int32_t ref=0, desiredPos=1000, cnt=0;
int32_t desiredVelA=0,desiredVelB=0;
char Rxbuff[16];
char myChar = 0;
float gain = 2;
float rot_gain = 0;
float rot_gain1 = 0;
int a = 0;
int yaw_cnt = 0;
float old_yawDeg = 0;
float velA=0, velB=0,yaw=0, dt=DT;
int Desired_vel_A=0,Desired_vel_B=0;
float Desired_yaw_deg=25;
bool second_num=false;

void setup() {
  
  Serial.begin(115200);
  BT.begin(9600);

  pinMode(LED_PIN,OUTPUT);
  
  motorA.stop(); motorB.stop();
  encoderA.setEncoderCount(0);
  encoderB.setEncoderCount(0);
  
  mpu.setup();
  mpu.setMagneticDeclination(8.5);
  mpu.setSampleRate(SR_100HZ);
  mpu.setGyroRange(GYRO_RANGE_2000DPS);
  mpu.setAccelRange(ACCEL_RANGE_16G);
  mpu.setDlpfBandwidth( DLPF_BANDWIDTH_184HZ); 
  mpu.enableDataReadyInterrupt();  
#ifdef CALIBRATION_MODE 
  calibrationProcess();
#endif    
 eeprom.loadCalibration();  // calibration data
 prevTime=micros(); pT=micros();
 
}


void loop() {
 if (mpu.isDataReady()){
    getDt();
    
  #ifndef RAW_DATA
    mpu.update(COMPLEMENTARY); //  MAGDWICK  /COMPLEMENTARY
  #else
     Vect3  a, g, m;  // acc/gyro/mag vectors
     mpu.update(a,g,m);
  #endif   
    Vect3 gyro=mpu.getGyroVect();
    float pitchDeg=mpu.getPitch()*RAD_TO_DEG;
    float yawDeg=mpu.getYaw()*RAD_TO_DEG;

    if(yawDeg <=0) yawDeg = 180+yawDeg;   // yaw 보정
    if(old_yawDeg - yawDeg >= 100)
    {
      yaw_cnt++;
    }
    if(old_yawDeg - yawDeg <= -100)
    {
      yaw_cnt--;
    }
    float new_yawDeg = yaw_cnt*180 + yawDeg;
    old_yawDeg = yawDeg;
    
    int32_t curPosA=encoderA.getEncoderCount();
    int32_t curPosB=encoderA.getEncoderCount();
    float curPos=(curPosA+curPosB)/2;
    float pitchRef=movePID.wheelControl(gain*desiredPos,curPos,dt);
    float mCommand= bala.balanceControl(-pitchRef,pitchDeg, gyro.x, dt);
    float yawCommandA = -rotateA.yawControl(new_yawDeg + rot_gain*Desired_yaw_deg, new_yawDeg, rot_gain1*gyro.z, dt);
    float yawCommandB = -yawCommandA;
    
    mCommand=constrain(mCommand,-5000,5000);
    yawCommandA=constrain(yawCommandA,-500,500);
    yawCommandB=constrain(yawCommandB,-500,500);
    desiredVelA=mCommand+yawCommandA; desiredVelB=mCommand+yawCommandB;

    int32_t uA= pidA.speedControl(desiredVelA,curPosA, dt, velA);
    int32_t uB= pidB.speedControl(desiredVelB,curPosB, dt, velB);

    if ((pitchDeg < ANGLE_LIMIT)&&(pitchDeg > -ANGLE_LIMIT)){  // fail
      motorA.run(uA); motorB.run(uB);  
    }
    else{ motorA.run(0);motorB.run(0); bala.setIntegrator(0); }
   
    while(BT.available())
    {
      myChar = (char)BT.read();
    
      if(a==0)
      {
        if(myChar == '!')
        {
          Rxbuff[a] = myChar;
          a++;
        }
        else
        {
          Rxbuff[2] = 0;
          a = 0;
        }
      }
      else if (a==1)
      {
        Rxbuff[a] = myChar;
        a++;
      }
      else if (a==2)
      {
        Rxbuff[a] = myChar;
        a++;
      }

      if(myChar == '?')
      {
        Rxbuff[a] = myChar;
        a = 0;
      }

      if (Rxbuff[0] == '!' && Rxbuff[1] == 'G' && Rxbuff[2] == '?')
      {
        gain = gain + 1;
//        BT.print("Displacement");BT.print("\t");BT.println(gain);
        Rxbuff[2] = 0;
      }
      else if (Rxbuff[0] == '!' && Rxbuff[1] == 'B' && Rxbuff[2] == '?')
      {
        gain = gain - 1;
//        BT.print("Displacement");BT.print("\t");BT.println(gain);
        Rxbuff[2] = 0;
      }
      else if (Rxbuff[0] == '!' && Rxbuff[1] == 'R' && Rxbuff[2] == 'R' && Rxbuff[3] == '?')
      {
        rot_gain = +1;
        rot_gain1 = 1;
//        BT.print("Rotate");BT.print("\t");BT.println(rot_gain);
        Rxbuff[2] = 0;
      }
      else if (Rxbuff[0] == '!' && Rxbuff[1] == 'L' && Rxbuff[2] == 'R' && Rxbuff[3] == '?')
      {
        rot_gain = -1;
        rot_gain1 = 1;
//        BT.print("Rotate");BT.print("\t");BT.println(rot_gain);
        Rxbuff[2] = 0;
      }
      else if (Rxbuff[0] == '!' && Rxbuff[1] == 'R' && Rxbuff[2] == 'S' && Rxbuff[3] == '?')
      {
        rot_gain = 0;
        rot_gain1 = 0;
//        BT.print("Rotate");BT.print("\t");BT.println(rot_gain);
        Rxbuff[2] = 0;
      }
    }
  }
}

void getDt(){
  uint32_t cT=micros();
  dt=uint32_t(cT - pT)/1000000.0;
  pT=cT;
}

void calibrationProcess(){
  mpu.calibrateGyro();   
  mpu.calibrateMag();
  eeprom.saveCalibration();
  Serial.println("Calibration Completed !!!!!!!!");  
    while(1);
}
