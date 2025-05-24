#include <WiFi.h>
#include <PubSubClient.h>
#include <Stepper.h>

// WiFi credentials
const char* ssid = "murat_wifi";
const char* password = "*******************************";
  
// MQTT Broker settings
const char* mqtt_server = "192.168.1.9"; 
const int mqtt_port = 1883;
const char* radiator_control_topic = "radiator/control";
const char* fan_control_topic = "fan/control";
const char* radiator_status_topic = "radiator/status";
const char* fan_status_topic = "fan/status";
const char* client_id = "esp32_radiator_controller1";

// Motor specifications (2048 steps/rev for full-step mode)
const int stepsPerRevolution = 2048;
const int radiatorSpin = 9800;  // 4.75 rotations (2048 * 4.75 ≈ 9800)

// ESP32 GPIO connections
#define IN1 12   // ULN2003 IN1
#define IN2 14   // ULN2003 IN2
#define IN3 27   // ULN2003 IN3
#define IN4 26   // ULN2003 IN4
#define RADIATOR_RELAY_PIN 25  // Relay control pin for radiator motor
#define FAN_RELAY_PIN 33      // Relay control pin for fan

// Initialize stepper library
Stepper myStepper(stepsPerRevolution, IN1, IN3, IN2, IN4);

WiFiClient espClient;
PubSubClient client(espClient);

enum DeviceState { UNKNOWN, ON, OFF };
DeviceState radiatorState = UNKNOWN;
DeviceState fanState = UNKNOWN;

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.println(message);

  // Process the message
  if (String(topic) == radiator_control_topic) {
    if (message == "ON" && radiatorState != ON) {
      activateRadiator();
    } else if (message == "OFF" && radiatorState != OFF) {
      deactivateRadiator();
    } else {
      Serial.print("Unrecognized radiator message: ");
      Serial.println(message);    
    }
  }
  else if (String(topic) == fan_control_topic) {
    if (message == "ON" && fanState != ON) {
      activateFan();
    } else if (message == "OFF" && fanState != OFF) {
      deactivateFan();
    } else {
      Serial.print("Unrecognized fan message: ");
      Serial.println(message);    
    }
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    
    if (client.connect(client_id)) {
      Serial.println("connected");
      // Subscribe to both control topics
      client.subscribe(radiator_control_topic);
      client.subscribe(fan_control_topic);
      Serial.println("Subscribed to topics:");
      Serial.println(" - " + String(radiator_control_topic));
      Serial.println(" - " + String(fan_control_topic));
      
      // Publish current statuses after reconnection
      publishRadiatorStatus();
      publishFanStatus();
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void activateFan() {
  Serial.println("Turning fan ON");
  digitalWrite(FAN_RELAY_PIN, LOW);  // Turn on relay to power fan
  delay(100);  // Short delay to ensure power is stable
  fanState = ON;
  publishFanStatus();
}

void deactivateFan() {
  Serial.println("Turning fan OFF");
  digitalWrite(FAN_RELAY_PIN, HIGH);  // Turn off relay to cut power to fan
  delay(100);  // Short delay to ensure power is stable
  fanState = OFF;
  publishFanStatus();
}

void enableRadiatorMotorPower() {
  digitalWrite(RADIATOR_RELAY_PIN, LOW);  // Turn on relay to power motor
  delay(100);  // Short delay to ensure power is stable
}

void disableRadiatorMotorPower() {
  delay(100);  // Short delay before cutting power
  digitalWrite(RADIATOR_RELAY_PIN, HIGH); // Turn off relay to cut power to motor
}

void activateRadiator() {
  if (radiatorState == ON) return;
  
  Serial.println("Turning radiator ON (CCW rotation)");
  enableRadiatorMotorPower();
  myStepper.step(-radiatorSpin);
  disableRadiatorMotorPower();
  radiatorState = ON;
  publishRadiatorStatus();
}

void deactivateRadiator() {
  if (radiatorState == OFF) return;
  
  Serial.println("Turning radiator OFF (CW rotation)");
  enableRadiatorMotorPower();
  myStepper.step(radiatorSpin);
  disableRadiatorMotorPower();
  radiatorState = OFF;
  publishRadiatorStatus();
}

void publishRadiatorStatus() {
  String status = (radiatorState == ON) ? "ON" : "OFF";
  client.publish(radiator_status_topic, status.c_str());
  Serial.println("Published radiator status: " + status);
}

void publishFanStatus() {
  String status = (fanState == ON) ? "ON" : "OFF";
  client.publish(fan_status_topic, status.c_str());
  Serial.println("Published fan status: " + status);
}

void setup() {
  Serial.begin(115200);
  
  // Initialize relay pins
  pinMode(RADIATOR_RELAY_PIN, OUTPUT);
  digitalWrite(RADIATOR_RELAY_PIN, HIGH);  // Start with radiator motor power off
  pinMode(FAN_RELAY_PIN, OUTPUT);
  digitalWrite(FAN_RELAY_PIN, HIGH);      // Start with fan power off
  
  // Initialize stepper motor (but it won't have power yet)
  myStepper.setSpeed(100);  // Set RPM (start with 5-15 RPM)
  
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}
