#include <Wire.h>

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
/////////////////////////IMU///////////////////////
#include <MPU9250_asukiaaa.h>
MPU9250 mySensor;
float aX, aY, aZ, aSqrt, gX, gY, gZ, mDirection, mX, mY, mZ;

/////////////////////////BLE///////////////////////

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
  qei_setup(PCNT_UNIT_0, 34, 35);
  qei_setup(PCNT_UNIT_1, 36, 39);
  pcnt_counter_resume(PCNT_UNIT_0);
  pcnt_counter_resume(PCNT_UNIT_1);

  //LED
  pixels.begin(); // This initializes the NeoPixel library.

  pixels.setBrightness(64); //0-255

  ///IMU
  Wire.begin(21, 22);

  mySensor.setWire(&Wire);
  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag();
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

    mySensor.accelUpdate();
    aX = mySensor.accelX();
    aY = mySensor.accelY();
    aZ = mySensor.accelZ();
    aSqrt = mySensor.accelSqrt();
    Serial.print("accelX: " + String(aX));
    Serial.print("accelY: " + String(aY));
    Serial.print("accelZ: " + String(aZ));
    Serial.print("accelSqrt: " + String(aSqrt));
    Serial.print("\t");

    mySensor.gyroUpdate();
    gX = mySensor.gyroX();
    gY = mySensor.gyroY();
    gZ = mySensor.gyroZ();
    Serial.print("gyroX: " + String(gX));
    Serial.print("gyroY: " + String(gY));
    Serial.print("gyroZ: " + String(gZ));
    Serial.print("\t");

    mySensor.magUpdate();
    mX = mySensor.magX();
    mY = mySensor.magY();
    mZ = mySensor.magZ();
    mDirection = mySensor.magHorizDirection();
    Serial.print("magX: " + String(mX));
    Serial.print("maxY: " + String(mY));
    Serial.print("magZ: " + String(mZ));
    Serial.print("horizontal direction: " + String(mDirection));
    Serial.print("\n");

    for (int e = 0; e < LEDNUM; e++)
    {
      pixels.setPixelColor(e, pixels.Color(rainbow[(i / 10) % 7][0], rainbow[(i / 10) % 7][1], rainbow[(i / 10) % 7][2]));
    }
    pixels.show();

    delay(100);
  }
}
