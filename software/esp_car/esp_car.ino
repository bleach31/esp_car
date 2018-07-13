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
#include <Adafruit_NeoPixel.h>
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

void qei_setup(pcnt_unit_t pcnt_unit, int gpioA, int gpioB)
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

//////////////////////////////////////setup///////////////////////////////////////////////
void setup()
{
  Serial.begin(9600);

  //ENC
  qei_setup(PCNT_UNIT_0, 35, 34);
  qei_setup(PCNT_UNIT_1, 39, 36);
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
    //MotorL.drive(i);     //-1023 - 0 - 1023
    //MotorR.drive(-1*i);  //-1023 - 0 - 1023
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
    Serial.println(tof_c.readRangeSingleMillimeters());
    if (tof_c.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

    //////////////////////
    for (int e = 0; e < LEDNUM; e++)
    {
      pixels.setPixelColor(e, pixels.Color(rainbow[(i / 10) % 7][0], rainbow[(i / 10) % 7][1], rainbow[(i / 10) % 7][2]));
      pixels.show();
    }
    delay(200);
  }
}
