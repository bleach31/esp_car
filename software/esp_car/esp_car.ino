#define ESP32
//////////////////////////Motor//////////////////////
#include "SparkFun_TB6612.h"
Motor MotorR = Motor(18, 19, 25, 0, 1); //制御ピン18，19, PWMピン25
Motor MotorL = Motor(16, 17, 26, 1, 1); //制御ピン16，17, PWMピン26
//制御用目標値RPM　－９０～９０ぐらい
double rpm_trg_L = 0;
double rpm_trg_R = 0; 
/////////////////////////LED///////////////////////
#include "Adafruit_NeoPixel.h"
#define LEDNUM 4
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(LEDNUM, 13, NEO_GRB + NEO_KHZ800);
char rainbow[7][3] = {{255, 0, 0}, {255, 165, 0}, {255, 255, 0}, {0, 128, 0}, {0, 255, 255}, {0, 0, 255}, {128, 0, 128}};

/////////////////////////BLE///////////////////////

/////////////////////////I2C///////////////////////
#include <Wire.h>
TwoWire I2C = TwoWire(1);
/////////////////////////IMU///////////////////////
#include <MPU9250_asukiaaa.h>
MPU9250 mpu;
float aX, aY, aZ, aSqrt, gX, gY, gZ, mDirection, mX, mY, mZ;
/////////////////////////ToF///////////////////////
#include "VL53L0X.h"
VL53L0X tof_c;

//int tof_c_xshut = 32;//GPIO32 プルアップ済み

////////////////////////battery////////////////
int BAT_PIN = 27;
float vbat;
////////////////////////Touch/////////////////
uint8_t touchThr = 20;
enum Mode
{
  DEBUG,
  FOLLOW,
  RC
};
enum Mode mode = DEBUG;
bool touch2detected = false;
void gotTouch0()
{
  rpm_trg_L = 0;
  rpm_trg_R = 0; 
  pixels.clear();
  pixels.setPixelColor(0, pixels.Color(rainbow[1][0], rainbow[1][1], rainbow[1][2]));
  pixels.show();

  mode = DEBUG;
}
void gotTouch2()
{
  mode = FOLLOW;
}

void gotTouch3() {
  rpm_trg_L = 100;
  rpm_trg_R = -100; 
}

void gotTouch5()
{
  //mode = RC;
}
/////////////////////////ENC///////////////////////
#include "qei.hpp"
double rpm_L = 0;
double rpm_R = 0;
hw_timer_t *timer = NULL;
TaskHandle_t th[2];
//

//////////////////////////setup/////////////////////

void setup()
{
  Serial.begin(115200);

  //RPM
  qei_setup_x4(PCNT_UNIT_0, 35, 34); //LEFT
  qei_setup_x4(PCNT_UNIT_1, 39, 36); //RIGHT

  qei_setup_x1(PCNT_UNIT_2, 35, 34);
  qei_setup_x1(PCNT_UNIT_3, 39, 36);
  //タイマー
  timer = timerBegin(0, 80, true); //80分周で1usec

  //制御
  xTaskCreatePinnedToCore(loop2,"loop2", 4096, NULL, 3, &th[0], 0); //core0でloop2を実行,優先度3

  //LED
  pixels.begin(); // This initializes the NeoPixel library.

  pixels.setBrightness(64); //0-255

  delay(200);
  ///IMU
  I2C.begin(21, 22, 400000); // SDA, SCL

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

  //Touch
  touchAttachInterrupt(T0, gotTouch0, touchThr);
  touchAttachInterrupt(T2, gotTouch2, touchThr);
  touchAttachInterrupt(T3, gotTouch3, touchThr);
  touchAttachInterrupt(T5, gotTouch5, touchThr);
}

void loop()
{

  switch (mode)
  {
  case DEBUG:
    debug();
    break;
  case FOLLOW:
    following();
    break;
  case RC:
    break;

  default:
    break;
  }

  delay(20);
}
//制御、定期センシング用

void loop2(void *pvParameters)
{
  uint8_t c = 0;
  while (true)
  {
    
    Serial.print("\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t");

    if(-1 < rpm_trg_L && rpm_trg_L < 1)
    {
      MotorL.brake(); 
    }else
    {
      int16_t duty_L = rpm_trg_L * 10 + (rpm_trg_L - rpm_L)*1.0;
      Serial.print(" duty_L:");
      Serial.print(duty_L);
      Serial.print(" rpm_trg_L:");
      Serial.print(rpm_trg_L);
      Serial.print(" rpm_L:");
      Serial.print(rpm_L);

      MotorL.drive(duty_L);
    }

    if(-1 < rpm_trg_R && rpm_trg_R < 1)
    {
      MotorR.brake(); 
    }else
    {
      int16_t duty_R = rpm_trg_R * 10 + (rpm_trg_R - rpm_R)*1.0;
      Serial.print(" duty_R:");
      Serial.print(duty_R);
      MotorR.drive(duty_R);
    }

    
    if(c >= 10){
      //RPM
      uint64_t time_now;
      int16_t count_L;
      int16_t count_R;

      time_now = timerRead(timer);
      pcnt_get_counter_value(PCNT_UNIT_0, &count_L);
      pcnt_get_counter_value(PCNT_UNIT_1, &count_R);

      rpm_L = (double)(count_L) / ((double)time_now/10000.0);
      rpm_R = (double)(count_R) / ((double)time_now/10000.0);

      //タイマーとカウンターのリセット
      timer = timerBegin(0, 80, true); //80分周で1usec
      pcnt_counter_clear(PCNT_UNIT_0);
      pcnt_counter_clear(PCNT_UNIT_1);

      Serial.print("RPM L:");
      Serial.print(rpm_L);
      Serial.print(" R:");
      Serial.print(rpm_R);

      //バッテリー値
      vbat = ((float)analogRead(BAT_PIN) / 4095) * ((35.0) / 10.0) * 3.3;//35/10はは抵抗値の適合後の値
      Serial.print(" vbat:");
      Serial.print(vbat);

      c = 0;
    }
      Serial.println("");
    c++;
    vTaskDelay(10);
  }
}


void debug()
{
  Serial.print(" T0:");
  Serial.print(touchRead(T0));
  Serial.print(" T2:");
  Serial.print(touchRead(T2));
  Serial.print(" T3:");
  Serial.print(touchRead(T3));
  Serial.print(" T5:");
  Serial.print(touchRead(T5));
  Serial.print("\t");

  //////////qei/////////////

  int16_t count_L;
  int16_t count_R;
  pcnt_get_counter_value(PCNT_UNIT_2, &count_L);
  pcnt_get_counter_value(PCNT_UNIT_3, &count_R);

  Serial.print("Pulse L:");
  Serial.print(count_L);
  Serial.print(" R:");
  Serial.print(count_R);
  Serial.print("\t");

  /*
    mpu.accelUpdate();
    aX = mpu.accelX();
    aY = mpu.accelY();
    aZ = mpu.accelZ();
    aSqrt = mpu.accelSqrt();
    Serial.print("accelX: " + String(aX));
    Serial.print("accelY: " + String(aY));
    Serial.print("accelZ: " + String(aZ))
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

  //////////////////////Led
  /*
    for (int e = 0; e < LEDNUM; e++)
    {
      pixels.setPixelColor(e, pixels.Color(rainbow[1][0], rainbow[1][1], rainbow[1][2]));
      pixels.show();
    }
    */

  Serial.println("");
}

void following()
{
  //////////////////////距離
  uint16_t dist_c = tof_c.readRangeSingleMillimeters();
  Serial.print(dist_c);
  //if (tof_c.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  Serial.print("\t");
  /////////////control
  if (dist_c > 8000) // lost -> search object
  {
    while (true)
    {
      for (int e = 0; e < LEDNUM; e++)
      {
        pixels.setPixelColor(e, pixels.Color(rainbow[6][0], rainbow[6][1], rainbow[6][2]));
        pixels.show();
      }
      //走査しながら左旋回
      pcnt_counter_clear(PCNT_UNIT_2);
      pcnt_counter_clear(PCNT_UNIT_3);

      rpm_trg_L = -20;
      rpm_trg_R = 20; 

      uint16_t dist_min = 65535;
      int16_t count_L_rem = 0;
      int16_t count_R_rem = 0;
      int16_t count_R = 0;

      while (true)
      {
        Serial.print("L:");
        Serial.print(count_L_rem);
        Serial.print("R:");
        Serial.println(count_R_rem);

        uint16_t dist_c = tof_c.readRangeSingleMillimeters();
        //最大距離とその時のエンコーダの値を記憶
        if (dist_c < dist_min)
        {
          dist_min = dist_c;
          pcnt_get_counter_value(PCNT_UNIT_2, &count_L_rem);
          pcnt_get_counter_value(PCNT_UNIT_3, &count_R_rem);
        }
        pcnt_get_counter_value(PCNT_UNIT_3, &count_R);
        if (count_R > 1500) //:TODO 約1回転ほど進んだら抜ける
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
      rpm_trg_L = 20;
      rpm_trg_R = -20; 
      while (true)
      {
        pcnt_get_counter_value(PCNT_UNIT_3, &count_R);
        if (count_R < count_R_rem)
        {
          break;
        }
      }
      //距離を確認して記憶した最小値＋50mmに物体があれば抜け、だめなら再試行
      uint16_t dist_c = tof_c.readRangeSingleMillimeters();
      if (dist_c < (dist_min + 50))
      {
        break;
      }
    }
  }
  else if (dist_c < 50) //near -> stop
  {
    rpm_trg_L = 0;
    rpm_trg_R = 0; 
    for (int e = 0; e < LEDNUM; e++)
    {
      pixels.setPixelColor(e, pixels.Color(rainbow[1][0], rainbow[1][1], rainbow[1][2]));
      pixels.show();
    }
  }
  else
  { //follow
  
    //distが最大2000なので目標値が20~60　rpmぐらい。
    rpm_trg_L = dist_c / 50 + 20;
    rpm_trg_R = dist_c / 50 + 20; 

    for (int e = 0; e < LEDNUM; e++)
    {
      pixels.setPixelColor(e, pixels.Color(rainbow[0][0], rainbow[0][1], rainbow[0][2]));
      pixels.show();
    }
  }
}
