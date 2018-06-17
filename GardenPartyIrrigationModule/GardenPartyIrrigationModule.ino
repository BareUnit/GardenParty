#include "Arduino.h"
#include "OneWire.h"
#include "Wire.h"
#include "DallasTemperature.h"
#include "ESP8266WiFi.h"
#include "BME280I2C.h"

// Pin definitions
#define pinAnalogMoistureSensor A0
#define pinDigitalThermometer D4
#define pinMoistureSensorPower D5
#define pinFlowSensor D6
#define pinRelay D7
#define pinBME280scl D1
#define pinBME280sda D2
#define i2cclock 100000


// BME280 definition
BME280I2C bme;

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

// enum for program states
enum States { INIT, MEASURE, START_WATERING, WATERING, STOP_WATERING, SEND_DATA, SLEEP};
States state = INIT;

// struct for measurement
struct SoilMeasurement {
	float Soil_Temperature;
	int Soil_Moisture;
	float Air_Temperature;
	float Air_Pressure;
	float Humidity;
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

void Read_BME280_Data() {

	   BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
	   BME280::PresUnit presUnit(BME280::PresUnit_Pa);

	   bme.read(SoilMeasurementSample.Air_Pressure , SoilMeasurementSample.Air_Temperature, SoilMeasurementSample.Humidity, tempUnit, presUnit);
}

// !!! To be implemented
float Get_AirTemperature() {
	return SoilMeasurementSample.Air_Temperature;
}

// !!! To be implemented
float Get_AirPressure() {
	return SoilMeasurementSample.Air_Pressure;
}

// !!! To be implemented
float Get_Humidity() {
	return SoilMeasurementSample.Humidity;
}

void getWeather() {

}

// Get measured values - soil moisture, soil temperature, humidity, air pressure, air temperature
void Get_SoilConditions() {
	SoilMeasurementSample = {Get_SoilTemperature(),Get_AverageSoilMoisture(),Get_AirTemperature(),Get_AirPressure(),Get_Humidity()};
	Serial.print("Teplota pudy   : "); Serial.println(SoilMeasurementSample.Soil_Temperature);
	Serial.print("Vlhkost pudy   : "); Serial.println(SoilMeasurementSample.Soil_Moisture);
	Serial.print("Teplota vzduchu: "); Serial.println(SoilMeasurementSample.Air_Temperature);
	Serial.print("Tlak vzduchu   : "); Serial.println(SoilMeasurementSample.Air_Pressure);
	Serial.print("Vlhkost vzduchu: "); Serial.println(SoilMeasurementSample.Humidity);
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
  SoilThermometer.begin();
  Wire.begin();
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

