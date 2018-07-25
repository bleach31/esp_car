#define ESP32
#include "esp_deep_sleep.h"
//////////////////////////Motor//////////////////////
#include "SparkFun_TB6612.h"
Motor MotorR = Motor(GPIO_NUM_18, GPIO_NUM_19, GPIO_NUM_25, 0, 1); //制御ピン18，19, PWMピン25
Motor MotorL = Motor(GPIO_NUM_16, GPIO_NUM_17, GPIO_NUM_26, 1, 1); //制御ピン16，17, PWMピン26
//制御用目標RPM　-90～90ぐらい
double rpm_trg_L = 0;
double rpm_trg_R = 0;
/////////////////////////LED///////////////////////
#include "Adafruit_NeoPixel.h"
#define LEDNUM 4
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(LEDNUM, GPIO_NUM_13, NEO_GRB + NEO_KHZ800);

uint32_t red = pixels.Color(255, 0, 0);
uint32_t orange = pixels.Color(255, 165, 0);
uint32_t yellow = pixels.Color(255, 255, 0);
uint32_t green = pixels.Color(0, 128, 0);
uint32_t cyan = pixels.Color(0, 255, 255);
uint32_t blue = pixels.Color(0, 0, 255);
uint32_t purple = pixels.Color(128, 0, 128);

/////////////////////////I2C///////////////////////
#include <Wire.h>
TwoWire I2C = TwoWire(0);
/////////////////////////IMU///////////////////////
#include "MPU9250_asukiaaa.h"
MPU9250 mpu;
float aX, aY, aZ, aSqrt, gX, gY, gZ, mDirection, mX, mY, mZ;
/////////////////////////ToF//////////////////////
#include "VL53L0X.h"
VL53L0X tof_c;

//int tof_c_xshut = 32;//GPIO32 とりあえず1個運用

////////////////////////battery////////////////
int BAT_PIN = GPIO_NUM_33;
double vbat;

///////////////////////ENC/////////////////////
#include "qei.hpp"
double rpm_L = 0;
double rpm_R = 0;
hw_timer_t *timer = NULL;
TaskHandle_t th[2];

///////////////////////Touch/////////////////
uint8_t touchThr = 20;

/////////////////////////BLE///////////////////////
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include <string.h>

BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t txValue = 0;

#define SERVICE_UUID           "8ff3d648-3ce2-4013-874a-4162cd85300d" // UART service UUID
#define CHARACTERISTIC_UUID_RX "f35a5414-d140-447b-90e9-3e0263ac7a05"
#define CHARACTERISTIC_UUID_TX "dd673cd0-24bf-4421-95d5-67bb3ab1f3d6"

class MyServerCallbacks : public BLEServerCallbacks {
	void onConnect(BLEServer* pServer) {
		deviceConnected = true;
	};

	void onDisconnect(BLEServer* pServer) {
		deviceConnected = false;
	}
};
class MyCallbacks : public BLECharacteristicCallbacks {
	void onWrite(BLECharacteristic *pCharacteristic) {
		//String rxValue = pCharacteristic->getValue();
		std::string rxValue = pCharacteristic->getValue();

		if (rxValue.length() > 0) {

			
			int sep = rxValue.find(",");
			const char* xChar = rxValue.substr(0, sep - 1).c_str();
			const char* yChar = rxValue.substr(sep + 1, rxValue.length()).c_str();

			float x = strtof(xChar, NULL);
			float y = strtof(yChar, NULL);
			
			//不感帯
			if (-1 < x && x < 1)
				x = 0;
			if (-1 < y && y < 1)
				y = 0;

			rpm_trg_L = y * -10 + x * -10;
			rpm_trg_R = y * -10 + x * 10;
			/*
			Serial.print(" x:");
			Serial.print(x);
			Serial.print(" y:");
			Serial.print(y)
			Serial.println("");
			*/

		}
	}
};

///////////////////////Mode///////////////////
enum Mode
{
	DEBUG,
	FOLLOW,
	RC,
	INIT
};
enum Mode mode = DEBUG;
enum Mode pre_mode = INIT;
bool touch2detected = false;
void gotTouch0()
{
	mode = DEBUG;
}
void gotTouch2()
{

}

void gotTouch3() 
{
	mode = RC;
}

void gotTouch5()
{
	mode = FOLLOW;
}

//////////////////////////setup/////////////////////

void setup()
{
	Serial.begin(115200);

	//RPM用
	qei_setup_x4(PCNT_UNIT_0, GPIO_NUM_35, GPIO_NUM_34); //LEFT
	qei_setup_x4(PCNT_UNIT_1, GPIO_NUM_39, GPIO_NUM_36); //RIGHT

	//制御用
	qei_setup_x1(PCNT_UNIT_2, GPIO_NUM_35, GPIO_NUM_34);
	qei_setup_x1(PCNT_UNIT_3, GPIO_NUM_39, GPIO_NUM_36);

	//タイマー
	timer = timerBegin(0, 80, true); //80分周で1usec

	//タスク設定 taskNO_AFFINITYでコア選択はスケジューラに委託
	xTaskCreatePinnedToCore(loop_fast, "loop_fast", 4096, NULL, 2, &th[0], tskNO_AFFINITY);
	xTaskCreatePinnedToCore(loop_slow, "loop_slow", 4096, NULL, 3, &th[0], tskNO_AFFINITY);

	//LED
	pixels.begin(); // This initializes the NeoPixel library.
	pixels.setBrightness(32); //0-255
	pixels.clear();
	pixels.setPixelColor(0, red);

	//IMU
	I2C.begin(GPIO_NUM_21, GPIO_NUM_22, 400000); // SDA, SCL

	mpu.setWire(&I2C);
	mpu.beginAccel();
	mpu.beginGyro();
	mpu.beginMag();

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

	//モードごとの定期処理
	switch (mode)
	{
	case DEBUG:
		debug();
		break;
	case FOLLOW:
		//呼び出し関数内部でループする場合,while(mode == {xxx})とし、モード変更時に抜けられるようにする
		following();
		break;
	case RC:
		{
			// disconnecting
			if (!deviceConnected && oldDeviceConnected) {
				rpm_trg_L = 0;
				rpm_trg_R = 0;
				pixels.clear();
				pixels.setPixelColor(0, blue);
				pixels.show();
				delay(500); // give the bluetooth stack the chance to get things ready
				pServer->startAdvertising(); // restart advertising
				Serial.println("restart advertising");
				oldDeviceConnected = deviceConnected;
			}
			// connecting
			if (deviceConnected && !oldDeviceConnected) {
				for (int e = 0; e < LEDNUM; e++)
				{
					pixels.setPixelColor(e, blue);
					pixels.show();
				}
				// do stuff here on connecting
				oldDeviceConnected = deviceConnected;
			}
			break;
		}

	default:
		break;
	}

	//モードごとの初期化処理
	if (pre_mode != mode)
	{
		// BLEラジコンから抜けるときは強制リセット。　↓がCLOSEされるまで。
		//		Dynamically Start Restart Bluetooth(BLE) · Issue #351 · nkolban / esp32 - snippets https ://github.com/nkolban/esp32-snippets/issues/351
		if (pre_mode == RC)
		{
			ESP.restart();
		}

		rpm_trg_L = 0;
		rpm_trg_R = 0;
		pixels.clear();
		switch (mode)
		{
		case DEBUG:
			pixels.setPixelColor(0, orange);
			pixels.show();
			break;
		case FOLLOW:
			break;
		case RC:
			{
				pixels.setPixelColor(0, blue);
				pixels.show();
				ble_setup();
				break;
			}

		default:
			break;
		}
	}

	pre_mode = mode;
	delay(20);
}

//遅い周期タスク
void loop_slow(void *pvParameters)
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 100 / portTICK_PERIOD_MS;
	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();

	while (true)
	{

		//RPM
		uint64_t time_now;
		int16_t count_L;
		int16_t count_R;

		time_now = timerRead(timer);
		pcnt_get_counter_value(PCNT_UNIT_0, &count_L);
		pcnt_get_counter_value(PCNT_UNIT_1, &count_R);

		rpm_L = (double)(count_L) / ((double)time_now / 10000.0);
		rpm_R = (double)(count_R) / ((double)time_now / 10000.0);

		//タイマーとカウンターのリセット
		timer = timerBegin(0, 80, true); //80分周で1usec
		pcnt_counter_clear(PCNT_UNIT_0);
		pcnt_counter_clear(PCNT_UNIT_1);

		/*
		Serial.print("RPM L:");
		Serial.print(rpm_L);
		Serial.print(" R:");
		Serial.print(rpm_R);
		*/

		//バッテリー値
		vbat = ((double)analogRead(BAT_PIN) / 4095) * ((35.0) / 10.0) * 3.3;//35/10はは抵抗値の適合後の値
		if (vbat < 6.0) {
			rpm_trg_L = 0;
			rpm_trg_R = 0;
			pixels.clear();

			Serial.print(" low battery\n");

			// スリープのための諸々の設定(今回はGPIO0ピンを使う)
			esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
			esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
			esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
			esp_deep_sleep_pd_config(ESP_PD_DOMAIN_MAX, ESP_PD_OPTION_OFF);

			esp_deep_sleep_start(); // スリープモード実行（Wakeupオプションなし＝外部リセットまで）

		}
		// Wait for the next cycle.
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}
//速い周期タスク
void loop_fast(void *pvParameters)
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 20 / portTICK_PERIOD_MS;
	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();

	while (true)
	{

		//制御　FF　+　P制御
		if (-1 < rpm_trg_L && rpm_trg_L < 1)
		{
			MotorL.brake();
		}
		else
		{
			//90rpmで8vぐらい。＋FB誤差補正
			double trg_v = rpm_trg_L * 0.09 + (rpm_trg_L - rpm_L)*0.01;
			MotorL.drive(((int)((trg_v * 1000) /vbat)));
			/*
			Serial.print("\t\t\t\t\t\t\t\t\t\t\t\t");
			Serial.print(trg_v);
			Serial.print("\t");
			Serial.print(((int)((trg_v*1000) / vbat)));
			Serial.println("");
			*/
		}

		if (-1 < rpm_trg_R && rpm_trg_R < 1)
		{
			MotorR.brake();
		}
		else
		{
			double trg_v = rpm_trg_R * 0.10 + (rpm_trg_R - rpm_R)*0.01;
			MotorR.drive(((int)((trg_v * 1000) / vbat)));
		}

		// Wait for the next cycle.
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}


void debug()
{
	Serial.print("TrgRPM L:");
	Serial.print(rpm_trg_L);
	Serial.print(" R:");
	Serial.print(rpm_trg_R);

	Serial.print(" T0:");
	Serial.print(touchRead(T0));
	Serial.print(" T1:");
	Serial.print(touchRead(T1));
	Serial.print(" T2:");
	Serial.print(touchRead(T2));
	Serial.print(" T3:");
	Serial.print(touchRead(T3));
	Serial.print(" T5:");
	Serial.print(touchRead(T5));

	int16_t count_L;
	int16_t count_R;
	pcnt_get_counter_value(PCNT_UNIT_2, &count_L);
	pcnt_get_counter_value(PCNT_UNIT_3, &count_R);

	Serial.print(" Pulse L:");
	Serial.print(count_L);
	Serial.print(" R:");
	Serial.print(count_R);
	Serial.print(" ");
	/*
	mpu.accelUpdate();
	aX = mpu.accelX();
	aY = mpu.accelY();
	aZ = mpu.accelZ();
	aSqrt = mpu.accelSqrt();
	Serial.print(" accelX: " + String(aX) + " accelY: " + String(aY) + " accelZ: " + String(aZ));
	Serial.print(" accelSqrt: " + String(aSqrt));
	Serial.print(" ");

	mpu.gyroUpdate();
	gX = mpu.gyroX();
	gY = mpu.gyroY();
	gZ = mpu.gyroZ();
	Serial.print(" gyroX: " + String(gX) + " gyroY: " + String(gY) + " gyroZ: " + String(gZ));
	Serial.print(" ");

	mpu.magUpdate();
	mX = mpu.magX();
	mY = mpu.magY();
	mZ = mpu.magZ();
	mDirection = mpu.magHorizDirection();
	Serial.print(" magX: " + String(mX) + " maxY: " + String(mY) +" magZ: " + String(mZ));
	Serial.print("horizontal direction: " + String(mDirection));
	*/

	uint16_t dist_c = tof_c.readRangeSingleMillimeters();
	Serial.print(" dist:");
	Serial.print(dist_c);

	Serial.print(" vbat:");
	Serial.print(vbat);

	Serial.println("");
}

uint8_t lost_flag=2^4;
void following()
{
	//////////////////////距離取得
	uint16_t dist_c = tof_c.readRangeSingleMillimeters();
	//if (tof_c.timeoutOccurred()) { Serial.print(" TIMEOUT"); }


	Serial.print(" dist:");
	Serial.println(dist_c);
	if (dist_c > 8000) // lost -> search object
	{
		lost_flag /=2;//5以上割ったら0

		while (mode == FOLLOW && lost_flag <= 0)//5連続でロスト判定
		{
			//走査しながら左旋回
			pcnt_counter_clear(PCNT_UNIT_2);
			pcnt_counter_clear(PCNT_UNIT_3);
			for (int e = 0; e < LEDNUM; e++)
			{
				pixels.setPixelColor(e, purple);
				pixels.show();
			}

			rpm_trg_L = -20;
			rpm_trg_R = 20;

			uint16_t dist_min = 65535;
			int16_t count_L_rem = 0;
			int16_t count_R_rem = 0;
			int16_t count_R = 0;

			while (mode == FOLLOW)
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
				delay(1);
			}

			for (int e = 0; e < LEDNUM; e++)
			{
				pixels.setPixelColor(e, cyan);
				pixels.show();
			}

			//記憶した位置まで右旋回して戻る
			rpm_trg_L = 20;
			rpm_trg_R = -20;
			while (mode == FOLLOW)
			{
				pcnt_get_counter_value(PCNT_UNIT_3, &count_R);
				if (count_R < count_R_rem)
				{
					break;
				}
				delay(1);
			}
			//距離を確認して記憶した最小値＋100mmに物体があれば抜け、だめなら再試行
			uint16_t dist_c = tof_c.readRangeSingleMillimeters();
			if (dist_c < (dist_min + 100))
			{
				break;
			}
		}
	}
	else if (dist_c < 50) //near -> stop
	{
		lost_flag = 2 ^ 4;
		rpm_trg_L = 0;
		rpm_trg_R = 0;
		for (int e = 0; e < LEDNUM; e++)
		{
			pixels.setPixelColor(e, yellow);
			pixels.show();
		}
	}
	else if (dist_c < 2000) //2m まで　follow
	{ 
		lost_flag = 2 ^ 4;
	  //distが最大2000なので目標値が20~60　rpmぐらい。
		rpm_trg_L = dist_c / 50 + 20;
		rpm_trg_R = dist_c / 50 + 20;

		for (int e = 0; e < LEDNUM; e++)
		{
			pixels.setPixelColor(e, red);
			pixels.show();
		}
	}
}
void ble_setup() {
	// Create the BLE Device
	BLEDevice::init("UART Service");

	// Create the BLE Server
	pServer = BLEDevice::createServer();
	pServer->setCallbacks(new MyServerCallbacks());

	// Create the BLE Service
	BLEService *pService = pServer->createService(SERVICE_UUID);

	// Create a BLE Characteristic
	pTxCharacteristic = pService->createCharacteristic(
		CHARACTERISTIC_UUID_TX,
		BLECharacteristic::PROPERTY_NOTIFY
	);

	pTxCharacteristic->addDescriptor(new BLE2902());

	BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
		CHARACTERISTIC_UUID_RX,
		BLECharacteristic::PROPERTY_WRITE
	);

	pRxCharacteristic->setCallbacks(new MyCallbacks());

	// Start the service
	pService->start();
	// Start advertising
	pServer->getAdvertising()->start();
	Serial.println("Waiting a client connection to notify...");
}