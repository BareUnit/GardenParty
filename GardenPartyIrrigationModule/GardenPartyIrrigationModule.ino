#include "Arduino.h"
#include "OneWire.h"
#include "Wire.h"
#include "DallasTemperature.h"
#include "ESP8266WiFi.h"
#include "Adafruit_Sensor.h"
#include "Adafruit_BME280.h"
#include "PubSubClient.h"

// Pin definitions
#define pinAnalogMoistureSensor A0
#define pinDigitalThermometer D4
#define pinMoistureSensorPower D5
#define pinFlowSensor D6
#define pinRelay D7
#define pinBME280scl D1
#define pinBME280sda D2
#define i2cclock 100000


// WIFI
const char* ssid = "Print.RedFrog.Lan";
const char* wifi_password = "Password.01";
WiFiClient wifiClient;

// MQTT
const char* mqtt_server = "192.168.10.210";
const char* mqtt_topic = "garden/bed";
const char* mqtt_username = "imodule1";
const char* mqtt_password = "imodule1";
// The client id identifies the ESP8266 device. Think of it a bit like a hostname (Or just a name, like Greg).
const char* clientID = "iModule1";
PubSubClient client(mqtt_server, 1883, wifiClient); // 1883 is the listener port for the Broker

// BME280 definition
Adafruit_BME280 bme; // I2C

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

// !!! To be implemented
float Get_AirTemperature() {
	return bme.readTemperature();
}

// !!! To be implemented
float Get_AirPressure() {
	return bme.readPressure();
}

// !!! To be implemented
float Get_Humidity() {
	return bme.readHumidity();
}

void Get_GardenConditions() {

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
  //SoilThermometer.begin();
  state = INIT;

  bool status = bme.begin();
  if (!status) {
      Serial.println("Could not find a valid BME280 sensor, check wiring!");
      while (1);
  }
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
		 WiFi.mode(WIFI_STA);
		  WiFi.hostname("ESP8266");
		  WiFi.begin(ssid, wifi_password);
		  while (WiFi.status() != WL_CONNECTED) {
		      delay(500);
		      Serial.print(".");
		  }
		  Serial.println("");
		  Serial.print("Pripojeno k WiFi siti ");
		  Serial.println("Print.RedFrog.Lan");
		  Serial.print("IP adresa: ");
		  Serial.println(WiFi.localIP());

		  // Connect to MQTT Broker
		  if (client.connect(clientID, mqtt_username, mqtt_password)) {
		    Serial.println("Connected to MQTT Broker!");
		  }
		  else {
		    Serial.println("Connection to MQTT Broker failed...");
		  }
		char mqttmessage[8];
		dtostrf(SoilMeasurementSample.Air_Temperature,6,2,mqttmessage);
		if (client.publish(mqtt_topic, mqttmessage)) {
			Serial.println("Air Temperature sent to MQTT broker");
		}
		else {
			Serial.println("Message failed to send. Reconnecting to MQTT Broker and trying again");
			client.connect(clientID, mqtt_username, mqtt_password);
			delay(10); // This delay ensures that client.publish doesn't clash with the client.connect call
			client.publish(mqtt_topic, mqttmessage);
		}
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

