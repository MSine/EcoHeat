#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <esp_sleep.h>
#include "soc/rtc_cntl_reg.h"

// Configuration
#define DHTPIN 4
#define DHTTYPE DHT11
#define GPIO_PIR 2
#define uS_TO_S_FACTOR 1000000
#define RTC_COEFF 20/3
#define SLEEP_DUR 60 //second

const char* ssid = "Robcave";
const char* password = "********";
const char* mqtt_server = "192.168.1.9";

// MQTT Topics
const char* tempTopic = "room/temperature";
const char* humTopic = "room/humidity";
const char* pirTopic = "room/pir";

// RTC memory variables
RTC_DATA_ATTR bool movementDetected = false;
RTC_DATA_ATTR uint64_t lastTransmissionTime = 0;

WiFiClient espClient;
PubSubClient client(espClient);
DHT dht(DHTPIN, DHTTYPE);

uint64_t get_rtc_time() {
  uint32_t hi, lo, hi2;
  do {
    hi = REG_READ(RTC_CNTL_TIME1_REG);    // High 16 bits
    lo = REG_READ(RTC_CNTL_TIME0_REG);    // Low 32 bits
    hi2 = REG_READ(RTC_CNTL_TIME1_REG);
  } while (hi != hi2); // Ensure consistent read
  return ((uint64_t)hi << 32) | lo;
}

void setup() {
  Serial.begin(115200);
  dht.begin();

  uint64_t now = get_rtc_time();
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

  if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0) {
    // PIR interrupt occurred
    movementDetected = true;
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_EXT0);
    Serial.println("Movement detected");
    
    // Calculate remaining sleep time
    uint64_t remainingSleep = SLEEP_DUR * uS_TO_S_FACTOR - (now - lastTransmissionTime)*RTC_COEFF;
    
    
    // Set wakeup timer for remaining time
    if(remainingSleep > 0) {
      esp_sleep_enable_timer_wakeup(remainingSleep);
      Serial.printf("Resuming sleep for %llu seconds\n", remainingSleep/uS_TO_S_FACTOR);
      esp_deep_sleep_start();
    }
    else {
        execute_transmission_then_sleep();
    }
  }
  else if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) {
        execute_transmission_then_sleep();
  }
  
  else {
    // Initial setup (first boot)
    Serial.println("Initial setup");
    configure_sleep_then_sleep();
  }
}

void execute_transmission_then_sleep()
{
    // Regular transmission cycle
    sendSensorData();
    movementDetected = false;
    lastTransmissionTime = get_rtc_time();
    configure_sleep_then_sleep();

}

void configure_sleep_then_sleep()
{
    // Schedule next wakeup
    esp_sleep_enable_timer_wakeup(SLEEP_DUR * uS_TO_S_FACTOR);
    esp_sleep_enable_ext0_wakeup((gpio_num_t)GPIO_PIR, HIGH);
    Serial.println("Going to sleep for " + String(SLEEP_DUR) + " seconds");
    esp_deep_sleep_start();
}
void loop() {}

void sendSensorData() {
  // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected!");

  // Connect to MQTT broker
  client.setServer(mqtt_server, 1883);
  Serial.print("Connecting to MQTT");
  while (!client.connect("ESP32Client")) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nMQTT Connected!");

  // Read sensor data
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();

  // Validate DHT readings
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("Failed to read DHT sensor!");
    temperature = NAN;
    humidity = NAN;
  }

  // Publish data
  if (!isnan(temperature)) {
    client.publish(tempTopic, String(temperature).c_str());
    Serial.println("Temperature published: " + String(temperature));
  }
  if (!isnan(humidity)) {
    client.publish(humTopic, String(humidity).c_str());
    Serial.println("Humidity published: " + String(humidity));
  }
  
  // Publish PIR status
  client.publish(pirTopic, String(movementDetected ? 1 : 0).c_str());
  Serial.println("PIR status published: " + String(movementDetected ? 1 : 0));

  // Disconnect
  client.disconnect();
  WiFi.disconnect(true);
  Serial.println("Disconnected from network");
}
