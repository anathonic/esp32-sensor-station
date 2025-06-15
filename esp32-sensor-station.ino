#include "DHT.h"
#include <SPI.h>
#include <WiFi.h>
#include <SPIFFS.h>
#include <WiFiUdp.h>
#include <TFT_eSPI.h>
#include <NTPClient.h>
#include <ArduinoJson.h>
#include <AsyncMqttClient.h>

extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/timers.h"
}

#define MAX_JSON_SIZE 2048

#define DHTPIN 18  
#define DHTTYPE DHT11 

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

DHT dht(DHTPIN, DHTTYPE);
TFT_eSPI tft = TFT_eSPI();

String nodeConfigFile = "/node_config.json";
String routerConfigFile = "/router_config.json";
String sensorConfigFile = "/sensor_config.json";

String wifiSsid;
String wifiPassword;

int mqttPort;
IPAddress mqttHost;


int frameId = 0;
String nodeName;
float temp, humidity;
String formattedDate;

String sensorId;
String sensorName;
String sensorType;
String sensorManufacturer;

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

long interval;
unsigned long previousMillis = 0;

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(wifiSsid.c_str(), wifiPassword.c_str());
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch(event) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0);
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}

String readFile(fs::FS &fs, String filename){
  Serial.println("readFile -> Reading file: " + filename);

  File file = fs.open(filename);
  if(!file || file.isDirectory()){
    Serial.println("readFile -> failed to open file for reading");
    return "";
  }

  String fileText = file.readString();
  file.close();
  return fileText;
}

void extractNodeJsonDoc(StaticJsonDocument<MAX_JSON_SIZE>& doc) {
  nodeName = doc["node"]["name"].as<String>();
  interval = doc["node"]["interval"];
  mqttHost.fromString(doc["mqtt"]["host"].as<String>());
  mqttPort = doc["mqtt"]["port"];
}

void extractRouterJsonDoc(StaticJsonDocument<MAX_JSON_SIZE>& doc) {
  wifiSsid = doc["router"]["network"]["wifi"]["ssid"].as<String>();
  wifiPassword = doc["router"]["network"]["wifi"]["password"].as<String>();
}

void extractSensorJsonDoc(StaticJsonDocument<MAX_JSON_SIZE>& doc) {
  sensorName = doc["sensor"]["name"].as<String>();
  sensorId = doc["sensor"]["id"].as<String>();
  sensorManufacturer = doc["sensor"]["manufacturer"].as<String>();
  sensorType = doc["sensor"]["type"].as<String>();
}

bool readConfig(String file_name, String type) {
  String file_content = readFile(SPIFFS, file_name);

  int config_file_size = file_content.length();
  Serial.println("Config file size: " + String(config_file_size));

  if(config_file_size > MAX_JSON_SIZE) {
    Serial.println("Config file too large");
    return false;
  }

  StaticJsonDocument<MAX_JSON_SIZE> doc;
  auto error = deserializeJson(doc, file_content);
  if (error) {
    Serial.println("Error interpreting config file");
    return false;
  }

  if(type == "node"){
    extractNodeJsonDoc(doc);
  } else if (type == "router") {
    extractRouterJsonDoc(doc);
  } else if (type == "sensor") {
    extractSensorJsonDoc(doc);
  }

  return true;
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.printf("  packetId: %d\n  qos: %d\n", packetId, qos);
}

void onMqttUnsubscribe(uint16_t packetId) {
  Serial.printf("Unsubscribe acknowledged. packetId: %d\n", packetId);
}

void onMqttPublish(uint16_t packetId) {
  Serial.printf("Publish acknowledged. packetId: %d\n", packetId);
}

void printSensorInformation() {
    tft.fillScreen(TFT_BLACK);
    tft.setTextSize(1);
    tft.setCursor(5, 5);
    tft.print(sensorName + " " + sensorId);
    tft.setCursor(5, 30);
    tft.print("Temp: ");
    tft.print(temp);
    tft.print(" C");
    tft.setCursor(5, 50);
    tft.print("Humidity: ");
    tft.print(humidity);
    tft.print(" %");
}

void setup() {
  Serial.begin(115200);
  Serial.println();

  dht.begin();

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);

  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS mount failed");
    return;
  } else {
    Serial.println("SPIFFS mounted successfully");
  }

  if (!readConfig(nodeConfigFile, "node")) {
    Serial.println("setup -> Could not read node config file -> initializing new file");
  }

  if (!readConfig(routerConfigFile, "router")) {
    Serial.println("setup -> Could not read router config file -> initializing new file");
  }

  if (!readConfig(sensorConfigFile, "sensor")) {
    Serial.println("setup -> Could not read sensor config file -> initializing new file");
  }

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(mqttHost, mqttPort);

  connectToWifi();

  timeClient.begin();
  timeClient.setTimeOffset(3600); // dynamic change for config
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    temp = dht.readTemperature();
    humidity = dht.readHumidity();

    if (isnan(temp) || isnan(humidity)) {
      Serial.println(F("Failed to read from DHT sensor!"));
      return;
    }

    frameId++;
    timeClient.forceUpdate();
    formattedDate = String(timeClient.getEpochTime());

    StaticJsonDocument<300> doc;
    doc["frameId"] = frameId;
    doc["timestamp"] = formattedDate;
    doc["temperature"] = temp;
    doc["temperature_unit"] = "C";
    doc["humidity"] = humidity;
    doc["humidity_unit"] = "%";

    String output;
    serializeJson(doc, output);
    String topic = "node/" + nodeName + "/" + sensorManufacturer + "/" + sensorType + "/" + sensorId + "/data";
    uint16_t packetIdPub = mqttClient.publish(topic.c_str(), 1, true, output.c_str());
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %d\n", topic.c_str(), packetIdPub);
    Serial.printf("Message: temp: %.2f°C, humidity: %.2f%%\n", temp, humidity);
    printSensorInformation();
  }
}
