#include <I2CSoilMoistureSensor.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

I2CSoilMoistureSensor sensor;
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2);

int Humidity;
int temperature;
int Light;
int indicator;
int HumidTreshmin = 450;
int HumidTreshmax = 500;
int TempTresh = 25;
int EN1 = 14; // nilai masukkan
int DIR1 = 12; // arah servo

#define USE_CLOUD 0


#if USE_CLOUD
#include <MakestroCloudClient.h>
#include <DCX_AppSetting.h>
#include <DCX_WifiManager.h>
#include <ESP8266WiFi.h>

DCX_WifiManager wifiManager(AppSetting);
MakestroCloudClient makestroCloudClient("Hisyam_Kamil", "CjIzfO689VdBxb0O5krsgLk7zdQWEDlwhz49eA0AaZ7b6rW2ZMAryQiguDIqeyE0", "plantWatering", "Hisyam_Kamil-plantWatering-default");

#endif

#if USE_CLOUD

  void  onSubscribedPropertyCallback(const String prop, const String value) {
        Serial.print("incoming: ");
        Serial.print(prop);
        Serial.print(" = ");
        Serial.print(value);
        Serial.println();
    }


  void onMqttConnect(bool sessionPresent) {
    Serial.println("** Connected to the broker **");
    makestroCloudClient.subscribeProperty("switch", onSubscribedPropertyCallback);

  }

  void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
    Serial.println("** Disconnected from the broker **");
  }

  void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
    Serial.println("** Subscribe acknowledged **");
    Serial.print("  packetId: ");
    Serial.println(packetId);
    Serial.print("  qos: ");
    Serial.println(qos);
  }

#endif


void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(9600);
  pinMode(EN1, OUTPUT);
  pinMode(DIR1, OUTPUT);
  digitalWrite(DIR1, HIGH); 
  sensor.begin();
  delay(1000);
  Serial.print("I2C Soil Moisture Sensor Address: ");
  Serial.println(sensor.getAddress(),HEX);
  Serial.print("Sensor Firmware version: ");
  Serial.println(sensor.getVersion(),HEX);
  Serial.println();

  #if USE_CLOUD
    AppSetting.load();
    AppSetting.debugPrintTo(Serial);

    wifiManager.onWifiConnectStarted([]() {
        DEBUG_SERIAL("WIFI CONNECTING STARTED\r\n");
    });

    wifiManager.onWifiConnected([](boolean newConn) {
        DEBUG_SERIAL("WIFI CONNECTED\r\n");

//        board.turnOffLED();

        makestroCloudClient.onConnect(onMqttConnect);
        makestroCloudClient.onDisconnect(onMqttDisconnect);
        makestroCloudClient.onSubscribe(onMqttSubscribe);

        makestroCloudClient.connect();
    });

    wifiManager.onWifiConnecting([](unsigned long elapsed) {
        DEBUG_SERIAL("%d\r\n", elapsed);
//        board.toggleLED();
    });

    wifiManager.onWifiDisconnected([](WiFiDisconnectReason reason) {
        DEBUG_SERIAL("WIFI GIVE UP\r\n");
        //board.turnOffLED();
    });

//    wifiManager.begin();
    wifiManager.begin("DyWare-AP2", "957DaegCen");
#endif  
}

void loop() {
  // put your main code here, to run repeatedly:
  Humidity = sensor.getCapacitance();
  Serial.print("Humidity = ");
  Serial.println(Humidity);
  temperature = sensor.getTemperature()/(float)10;
  Serial.print("Temperature = ");
  Serial.println(temperature);
  Light = sensor.getLight(true);
  delay(100);
  if (Humidity < HumidTreshmin ) {
    ledcWrite(24, 70); 
    #if USE_CLOUD
    indicator = 1;
    makestroCloudClient.publishKeyValue("Indicator",indicator);
    #endif
  }
  if (Humidity > HumidTreshmax ) {
    ledcWrite(24, 0); 
    #if USE_CLOUD
    indicator = 0;
    makestroCloudClient.publishKeyValue("Indicator",indicator);
    #endif
  }

  #if USE_CLOUD
    if (makestroCloudClient.connected()) {
                makestroCloudClient.publishKeyValue("Humidity", Humidity);
                makestroCloudClient.publishKeyValue("Temperature", temperature);
                makestroCloudClient.publishKeyValue("Light",Light);
                Serial.print("Publish to Cloud");
            }
  
  #endif
}

void ledcAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 255) {
  // calculate duty, 8191 from 2 ^ 13 - 1
  uint32_t duty = (8191 / valueMax) * min(value, valueMax);

  // write duty to LEDC
  ledcWrite(channel, duty);
}
