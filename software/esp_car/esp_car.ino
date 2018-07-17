#define ESP32
//////////////////////////Motor//////////////////////
#include "SparkFun_TB6612.h"
Motor MotorR = Motor(18, 19, 25, 0, 1); //制御ピン18，19, PWMピン25
Motor MotorL = Motor(16, 17, 26, 1, 1); //制御ピン16，17, PWMピン26

/////////////////////////ENC////////////////////////////
#include "driver/pcnt.h"
int16_t count_R = 0;
int16_t count_L = 0;
/////////////////////////LED///////////////////////
#include "Adafruit_NeoPixel.h"
#define LEDNUM 4
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(LEDNUM, 13, NEO_GRB + NEO_KHZ800);
char rainbow[7][3] = {{255, 0, 0}, {255, 165, 0}, {255, 255, 0}, {0, 128, 0}, {0, 255, 255}, {0, 0, 255}, {128, 0, 128}};

/////////////////////////BLE///////////////////////
/////////////////////////I2C///////////////////////
#include <Wire.h>
TwoWire I2C = TwoWire(0);
/////////////////////////IMU///////////////////////
#include <MPU9250_asukiaaa.h>
MPU9250 mpu;
float aX, aY, aZ, aSqrt, gX, gY, gZ, mDirection, mX, mY, mZ;
/////////////////////////ToF///////////////////////
#include "VL53L0X.h"
VL53L0X  tof_c;

//int tof_c_xshut = 32;//GPIO32 プルアップ済み

////////////////////////battery////////////////
int BAT_PIN = 27;
/////////////////////////BLE////////////////////
void qei_setup_x4(pcnt_unit_t pcnt_unit, int gpioA, int gpioB)
{
  pcnt_config_t pcnt_confA;
  pcnt_config_t pcnt_confB;

  pcnt_confA.unit = pcnt_unit;
  pcnt_confA.channel = PCNT_CHANNEL_0;
  pcnt_confA.pulse_gpio_num = gpioA;
  pcnt_confA.ctrl_gpio_num = gpioB;
  pcnt_confA.pos_mode = PCNT_COUNT_INC;
  pcnt_confA.neg_mode = PCNT_COUNT_DEC;
  pcnt_confA.lctrl_mode = PCNT_MODE_REVERSE;
  pcnt_confA.hctrl_mode = PCNT_MODE_KEEP;
  pcnt_confA.counter_h_lim = 32767;
  pcnt_confA.counter_l_lim = -32768;

  pcnt_confB.unit = pcnt_unit;
  pcnt_confB.channel = PCNT_CHANNEL_1;
  pcnt_confB.pulse_gpio_num = gpioB;
  pcnt_confB.ctrl_gpio_num = gpioA;
  pcnt_confB.pos_mode = PCNT_COUNT_INC;
  pcnt_confB.neg_mode = PCNT_COUNT_DEC;
  //ここが逆になる
  pcnt_confB.lctrl_mode = PCNT_MODE_KEEP;
  pcnt_confB.hctrl_mode = PCNT_MODE_REVERSE;
  pcnt_confB.counter_h_lim = 32767;
  pcnt_confB.counter_l_lim = -32768;

  /* Initialize PCNT unit */
  pcnt_unit_config(&pcnt_confA);
  pcnt_unit_config(&pcnt_confB);

  pcnt_counter_pause(pcnt_unit);
  pcnt_counter_clear(pcnt_unit);
}
void qei_setup_x1(pcnt_unit_t pcnt_unit, int gpioA, int gpioB)
{
  pcnt_config_t pcnt_confA;

  pcnt_confA.unit = pcnt_unit;
  pcnt_confA.channel = PCNT_CHANNEL_0;
  pcnt_confA.pulse_gpio_num = gpioA;
  pcnt_confA.ctrl_gpio_num = gpioB;
  pcnt_confA.pos_mode = PCNT_COUNT_INC;//立ち上がりのみカウント
  pcnt_confA.neg_mode = PCNT_COUNT_DIS;//立ち下がりはカウントしない
  pcnt_confA.lctrl_mode = PCNT_MODE_REVERSE;//立ち上がり時にB相がHighなら逆転
  pcnt_confA.hctrl_mode = PCNT_MODE_KEEP;
  pcnt_confA.counter_h_lim = 32767;
  pcnt_confA.counter_l_lim = -32768;

  /* Initialize PCNT unit */
  pcnt_unit_config(&pcnt_confA);

  pcnt_counter_pause(pcnt_unit);
  pcnt_counter_clear(pcnt_unit);
}
//////////////////////////////////////setup///////////////////////////////////////////////
void setup()
{
  Serial.begin(115200);

  //ENC
  qei_setup_x1(PCNT_UNIT_0, 35, 34);
  qei_setup_x1(PCNT_UNIT_1, 39, 36);
  pcnt_counter_resume(PCNT_UNIT_0);
  pcnt_counter_resume(PCNT_UNIT_1);

  //LED
  pixels.begin(); // This initializes the NeoPixel library.

  pixels.setBrightness(64); //0-255

  delay(200);
  ///IMU
  I2C.begin(21,22,400000); // SDA, SCL

  mpu.setWire(&I2C);
  mpu.beginAccel();
  mpu.beginGyro();
  mpu.beginMag();
  delay(200);

  //ToF
  tof_c.setWire(&I2C);
  tof_c.init();
  tof_c.setTimeout(500);
  // LONG RANGE MODE
  // lower the return signal rate limit (default is 0.25 MCPS)
  tof_c.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  tof_c.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  tof_c.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  
}

void loop()
{
  
  for (int i = 0; i < 1024; i++)
  {
    
    Serial.print("T0:");
    Serial.print(touchRead(T0));
    Serial.print("T1:");
    Serial.print(touchRead(T1));
    Serial.print("T2:");
    Serial.print(touchRead(T2));
    Serial.print("T3:");
    Serial.print(touchRead(T3));
    Serial.print("T5:");
    Serial.print(touchRead(T5));
    Serial.print("\t");
    /////////////battery moniter//////////////
    float vbat = ((float)analogRead(BAT_PIN)/4095)*((10.0+22.0)/10.0)*3.3;
    Serial.print("vbat:");
    Serial.print(vbat);
    Serial.print("\t");
    if(vbat < 6.0){
      MotorL.drive(0);
      MotorR.drive(0);
      break;
    }

    //MotorL.drive(i);     //-1023 - 0 - 1023
    //MotorR.drive(-1*i);  //-1023 - 0 - 1023


    //////////qei/////////////
    pcnt_get_counter_value(PCNT_UNIT_0, &count_R);
    pcnt_get_counter_value(PCNT_UNIT_1, &count_L);
    Serial.print("tire lotate value:");
    Serial.print(count_R / (357.7 * 4));
    Serial.print("\t");
    Serial.print(count_L / (357.7 * 4));
    Serial.print("\t");

    /*
    mpu.accelUpdate();
    aX = mpu.accelX();
    aY = mpu.accelY();
    aZ = mpu.accelZ();
    aSqrt = mpu.accelSqrt();
    Serial.print("accelX: " + String(aX));
    Serial.print("accelY: " + String(aY));
    Serial.print("accelZ: " + String(aZ));
    Serial.print("accelSqrt: " + String(aSqrt));
    Serial.print("\t");

    mpu.gyroUpdate();
    gX = mpu.gyroX();
    gY = mpu.gyroY();
    gZ = mpu.gyroZ();
    Serial.print("gyroX: " + String(gX));
    Serial.print("gyroY: " + String(gY));
    Serial.print("gyroZ: " + String(gZ));
    Serial.print("\t");

    mpu.magUpdate();
    mX = mpu.magX();
    mY = mpu.magY();
    mZ = mpu.magZ();
    mDirection = mpu.magHorizDirection();
    Serial.print("magX: " + String(mX));
    Serial.print("maxY: " + String(mY));
    Serial.print("magZ: " + String(mZ));
    Serial.print("horizontal direction: " + String(mDirection));
    Serial.print("\n");
    */
    //////////////////////
    uint16_t dist_c = tof_c.readRangeSingleMillimeters();
    Serial.print(dist_c);
    //if (tof_c.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
    Serial.print("\t");

    //////////////////////Led
    /*
    for (int e = 0; e < LEDNUM; e++)
    {
      pixels.setPixelColor(e, pixels.Color(rainbow[(i / 10) % 7][0], rainbow[(i / 10) % 7][1], rainbow[(i / 10) % 7][2]));
      pixels.show();
    }
    */
    /////////////control
    if(dist_c > 8000) // lost -> search object
    {
      for (int e = 0; e < LEDNUM; e++)
      {
        pixels.setPixelColor(e, pixels.Color(rainbow[6][0], rainbow[6][1], rainbow[6][2]));
        pixels.show();
      }

      while(true){
        //走査しながら左旋回
        pcnt_counter_clear(PCNT_UNIT_0);
        pcnt_counter_clear(PCNT_UNIT_1);
        MotorL.drive(-200);
        MotorR.drive(200);
        
        uint16_t dist_min = 65535;
        int16_t count_L_rem = 0;
        int16_t count_R_rem = 0;

        while (true)
        {
          uint16_t dist_c = tof_c.readRangeSingleMillimeters();
          //最大距離とその時のエンコーダの値を記憶
          if(dist_c < dist_min)
          {
            pcnt_get_counter_value(PCNT_UNIT_1, &count_L_rem);
            pcnt_get_counter_value(PCNT_UNIT_0, &count_R_rem);
          }
          if(count_R_rem > 10000) //:TODO 約1回転の値を入れるあとで。
          {
            break;
          }
          delay(10);
        }

        for (int e = 0; e < LEDNUM; e++)
        {
          pixels.setPixelColor(e, pixels.Color(rainbow[5][0], rainbow[5][1], rainbow[5][2]));
          pixels.show();
        }

        //記憶した位置まで右旋回して戻る
        MotorL.drive(200);
        MotorR.drive(-200);
        while(true)
        {
            pcnt_get_counter_value(PCNT_UNIT_1, &count_R);
            if(count_R < count_R_rem){
              break;
            }
        }
        //距離を確認して記憶した最小値＋50mmに物体があれば抜け、だめなら再試行
          uint16_t dist_c = tof_c.readRangeSingleMillimeters();
          if(dist_c < (dist_min + 50))
          {
            break;
          }
      }
      
      

    }else if(dist_c < 50 ) //near -> stop
    {
      MotorL.drive(0);
      MotorR.drive(0);
      for (int e = 0; e < LEDNUM; e++)
      {
        pixels.setPixelColor(e, pixels.Color(rainbow[1][0], rainbow[1][1], rainbow[1][2]));
        pixels.show();
      }
    }else{ //follow
      MotorL.drive(dist_c/2 + 50);
      MotorR.drive(dist_c/2 + 50);

      for (int e = 0; e < LEDNUM; e++)
      {
        pixels.setPixelColor(e, pixels.Color(rainbow[0][0], rainbow[0][1], rainbow[0][2]));
        pixels.show();
      }
    }

    Serial.println("");
    delay(50);
  }
}
