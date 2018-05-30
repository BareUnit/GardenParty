#include "Arduino.h"
#include "OneWire.h"
#include "DallasTemperature.h"
#include "ESP8266WiFi.h"
#include "Adafruit_Sensor.h"
#include "Adafruit_BME280.h"

// Pin definitions
#define pinAnalogMoistureSensor A0
#define pinDigitalThermometer D4
#define pinMoistureSensorPower D5
#define pinFlowSensor D6
#define pinRelay D7
#define pinBME280scl D1
#define pinBME280sda D2
#define i2cclock 100000

// Relay module states ON/OFF
#define RelayOn 0
#define RelayOff 1

//Global delay time for transaction
#define delay_time 0

#define SoilMoistureTreshhold 400
#define sleepSeconds 10

//flow sensor
volatile int PulseCount = 0;
volatile int CurrentSoilMoisture = 1024;

// BME280 init
Adafruit_BME280 bme; // I2C
float h, t, p, pin, dp;
char temperatureFString[6];
char dpString[6];
char humidityString[6];
char pressureString[7];
char pressureInchString[6];


// enum for program states
enum States { INIT, MEASURE, START_WATERING, WATERING, STOP_WATERING, SEND_DATA, SLEEP};
States state = INIT;

// struct for measurement
struct SoilMeasurement {
	float Soil_Temperature;
	int Soil_Moisture;
	float Air_Temperature;
	int Air_Pressure;
	int Humidity;
};

SoilMeasurement SoilMeasurementSample = {0,0,0,0,0};

// create instance oneWireDS from OneWire library
OneWire oneWireDS(pinDigitalThermometer);
// create instance sensorDS from DallasTemperature library
DallasTemperature SoilThermometer(&oneWireDS);


// flow sensor


void ActivateRelay() {
	Serial.println("Aktivuji rele ...");
	digitalWrite(pinRelay, RelayOn);
	delay(delay_time);
}

void DeactivateRelay() {
	Serial.println("Dektivuji rele ...");
	digitalWrite(pinRelay, RelayOff);
	delay(delay_time);
}

void ActivateSoilMoistureSensor() {
	digitalWrite(pinMoistureSensorPower, 1);
	delay(5000);
}

void DeactivateSoilMoistureSensor() {
	digitalWrite(pinMoistureSensorPower, 0);
	delay(5000);
}

void Get_CurrentSoilMoister() {
	CurrentSoilMoisture = analogRead(pinAnalogMoistureSensor);
}

// Get soil temperature
float Get_SoilTemperature() {
	SoilThermometer.requestTemperatures();
	return SoilThermometer.getTempCByIndex(0);
}

// Get soil moisture
int Get_AverageSoilMoisture() {
	  digitalWrite(pinMoistureSensorPower, 1);
	  delay(10000);
	  int AverageSoilMoisture = 0;
	  int counter = 0;
	  while(counter < 15 ) {
		  AverageSoilMoisture += analogRead(pinAnalogMoistureSensor);
		  counter++;
		  delay(500);
	  }
	  AverageSoilMoisture = AverageSoilMoisture / counter;
	  digitalWrite(pinMoistureSensorPower, 0);
	  return AverageSoilMoisture;
}

// !!! To be implemented
float Get_AirTemperature() {
	return 0; // replace 0 with return variable
}

// !!! To be implemented
int Get_AirPressure() {
	return 0; // replace 0 with return variable
}

// !!! To be implemented
int Get_Humidity() {
	return 0; // replace 0 with return variable
}

void getWeather() {

    h = bme.readHumidity();
    t = bme.readTemperature();
    t = t*1.8+32.0;
    dp = t-0.36*(100.0-h);

    p = bme.readPressure()/100.0F;
    pin = 0.02953*p;
    dtostrf(t, 5, 1, temperatureFString);
    dtostrf(h, 5, 1, humidityString);
    dtostrf(p, 6, 1, pressureString);
    dtostrf(pin, 5, 2, pressureInchString);
    dtostrf(dp, 5, 1, dpString);
    delay(100);

}

// Get measured values - soil moisture, soil temperature, humidity, air pressure, air temperature
void Get_SoilConditions() {
	SoilMeasurementSample = {Get_SoilTemperature(),Get_AverageSoilMoisture(),Get_AirTemperature(),Get_AirPressure(),Get_Humidity()};
	getWeather();
	Serial.print("Teplota pudy   : "); Serial.println(SoilMeasurementSample.Soil_Temperature);
	Serial.print("Vlhkost pudy   : "); Serial.println(SoilMeasurementSample.Soil_Moisture);
	//Serial.print("Teplota vzduchu: "); Serial.println(SoilMeasurementSample.Air_Temperature);
	//Serial.print("Tlak vzduchu   : "); Serial.println(SoilMeasurementSample.Air_Pressure);
	//Serial.print("Vlhkost vzduchu: "); Serial.println(SoilMeasurementSample.Humidity);

	Serial.print("Teplota vzduchu: "); Serial.println(temperatureFString);
	Serial.print("Tlak vzduchu   : "); Serial.println(humidityString);
	Serial.print("Vlhkost vzduchu: "); Serial.println(pressureString);
}

void ClearSessionVariables() {
	CurrentSoilMoisture = 1024;
	PulseCount = 0;
}

// interrupt function
void AddPulse() {
	PulseCount++;
	Get_CurrentSoilMoister();

}

void setup(void) {
  // set serial line baud rate to 115200
  Serial.begin(115200);
  // flow sensor
  pinMode(pinFlowSensor, INPUT_PULLUP);
  pinMode(pinRelay, OUTPUT);
  pinMode(pinMoistureSensorPower, OUTPUT);
  // close water valve
  DeactivateRelay();
  // disable power to soil thermometer
  digitalWrite(pinMoistureSensorPower, 0);
  // initiate i2c bus
  Wire.begin(pinBME280scl, pinBME280sda);
  Wire.setClock(i2cclock);
  SoilThermometer.begin();
  state = INIT;
}

void loop(void) {
	switch (state) {
	case INIT:
		state = MEASURE;
		break;
	case MEASURE:
		Get_SoilConditions();
		if (SoilMeasurementSample.Soil_Moisture > SoilMoistureTreshhold) {
			state = START_WATERING;
		}
		else {
			state = SEND_DATA;
		}
		break;
	case START_WATERING: // attach interrupt here
		ActivateSoilMoistureSensor();
		attachInterrupt(digitalPinToInterrupt(pinFlowSensor), AddPulse, RISING);
		ActivateRelay();
		state = WATERING;
		break;
	case WATERING:
		//if ((SoilMeasurementSample.Soil_Moisture - PulseCount) < SoilMoistureTreshhold) { //debug
		if (CurrentSoilMoisture < SoilMoistureTreshhold) {
			// flow sensor
			Serial.print("Pocet pulzu celkem:"); Serial.println(PulseCount);
			state = STOP_WATERING; //move this line to if condition above
		}
		break;
	case STOP_WATERING: // detach interrupt here
		DeactivateRelay();
		detachInterrupt(digitalPinToInterrupt(pinFlowSensor));
		DeactivateSoilMoistureSensor();
		state = SEND_DATA;
		break;
	case SEND_DATA: // create structure and send data to Raspberry PI with MQTT
		Serial.println("Odesilam data ...");
		state = SLEEP;
		break;
	case SLEEP:
		Serial.println("Jdu spat ...");
		ClearSessionVariables();
		ESP.deepSleep(sleepSeconds * 1000000, WAKE_NO_RFCAL);
		state = INIT;
		break;
	}
}

