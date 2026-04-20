#include <Arduino.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <NeoPixelBus.h>
#include <Adafruit_TLC5947.h>
#include <ArduinoJson.h> // For MQTT JSON parsing
#include <ArduinoOTA.h> // For OTA updates
#include <TelnetStream.h>

// --- Gamma Correction ---
const float GAMMA = 2.8;
uint8_t gamma8[256];

// --- Smart Power Limiting ---
const int MAX_TOTAL_CHANNEL_VALUE = 250;

// --- Task and Mutex Handles ---
TaskHandle_t NetworkTask;
SemaphoreHandle_t rgbwStateMutex;
SemaphoreHandle_t starrySkyStateMutex;
SemaphoreHandle_t starrySkySpeedStateMutex;

// --- Configuration ---
const char* DEVICE_NAME = "WolkLampAnne"; 

// MQTT
const char* MQTT_SERVER = "192.168.1.126";
const int MQTT_PORT = 1883;
const char* MQTT_USER = "mqtt";
const char* MQTT_PASSWORD = "mqtt";
const char* MQTT_CLIENT_ID = "WolklampESP32"; 

// Home Assistant MQTT Discovery
const char* HA_DISCOVERY_PREFIX = "homeassistant";
const char* HA_DEVICE_NAME = DEVICE_NAME;
const char* HA_DEVICE_ID = "wolkLampAnne_esp32_s3"; 

// MQTT Topics
String TOPIC_STARRY_SKY_STATE;
String TOPIC_STARRY_SKY_COMMAND;
String TOPIC_230V_STATE;
String TOPIC_STARRY_SKY_SPEED_STATE;
String TOPIC_STARRY_SKY_SPEED_COMMAND;

String TOPIC_DEFAULT_STARRY_SKY_BRIGHTNESS_COMMAND;
String TOPIC_DEFAULT_STARRY_SKY_BRIGHTNESS_STATE;

String TOPIC_DEFAULT_RGBW_BRIGHTNESS_COMMAND;
String TOPIC_DEFAULT_RGBW_BRIGHTNESS_STATE;
String TOPIC_DEFAULT_RGBW_COLOR_COMMAND;
String TOPIC_DEFAULT_RGBW_COLOR_STATE; 
String TOPIC_RGBW_COMMAND;
String TOPIC_RGBW_STATE; 
String TOPIC_BUILD_INFO;

// Pin Definitions
#define PIN_RGBW_STRIP      2   
#define PIN_TLC_SIN         14  
#define PIN_TLC_SCLK        21  
#define PIN_TLC_BLANK       47  
#define PIN_TLC_XLAT        48  
#define PIN_230V_INPUT      10  

// RGBW Strip
const uint16_t PixelCount = 120; 
NeoPixelBus<NeoGrbwFeature, Neo800KbpsMethod> strip(PixelCount, PIN_RGBW_STRIP);
bool rgbw_on = false;
RgbwColor targetRgbwColor(0, 0, 0, 0); 
RgbwColor defaultWhiteColorOn230V(0, 0, 0, 255); 
uint8_t targetRgbwBrightness = 0;      

// TLC5947 Starry Sky
#define TLC_NUM_CHIPS   1    
Adafruit_TLC5947 tlc = Adafruit_TLC5947(TLC_NUM_CHIPS, PIN_TLC_SCLK, PIN_TLC_SIN, PIN_TLC_XLAT);
bool starry_sky_on = false; 
bool starry_sky_last_state = false; 
uint8_t targetStarrySkyBrightness = 0; 
uint8_t targetStarrySkySpeed = 50; 
uint8_t currentStarrySkyBrightness = 0; 
float currentStarrySkyBrightnessFading = 0.0; 
unsigned long lastStarrySkyUpdateTime = 0;
const unsigned long starrySkyFadeDuration = 400; 

// Default settings for 230V input trigger
uint8_t defaultRgbwBrightnessOn230V = 255; 
RgbwColor defaultRgbwColorOn230V(0, 0, 0, 255); 
uint8_t defaultStarrySkyBrightnessOn230V = 255; 

// RGBW Strip Fading
RgbwColor currentRgbwDisplayColor(0, 0, 0, 0); 
uint8_t currentRgbwDisplayBrightness = 0;      
float currentRgbwDisplayBrightnessFading = 0.0;
unsigned long lastRgbwUpdateTime = 0;
const unsigned long rgbwFadeDuration = 400; 


// 230V Input Button State
unsigned long lastButtonPressTime = 0;
bool buttonState = HIGH;      
bool lastButtonState = HIGH;  
const long debounceDelay = 50; 
bool is230vAbsent = false; 
unsigned long input230vAbsentStartTime = 0; 

// Light Source Toggle State
enum LightSource { RGBW_STRIP, STARRY_SKY, NONE };
LightSource currentActiveLightSource = NONE;

// Starry Sky Animation
struct Star {
  uint16_t currentBrightness; 
  uint16_t targetBrightness;
  unsigned long fadeStartTime;
  unsigned long phaseStartTime;
  unsigned long lastUpdateTime;
  enum { STAR_OFF, STAR_FADE_IN, STAR_ON, STAR_FADE_OUT } phase;

  uint16_t minBrightnessPWM; 
  uint16_t maxBrightnessPWM; 
  unsigned long offDuration; 
  unsigned long onDuration; 
  unsigned long fadeInDuration;    
  unsigned long fadeOutDuration;   
};

Star stars[TLC_NUM_CHIPS * 24]; 
unsigned long lastStarUpdateTime = 0;
const unsigned long starUpdateInterval = 20; 

// WiFiClient and PubSubClient objects
WiFiClient espClient;
PubSubClient client(espClient);

// Function prototypes
void setupWifi();
void reconnectMqtt();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void publishMqttDiscovery();
void handle230VInput();
void updateRgbwStripState();
void updateStarrySkyState();
void sendRgbwState();
void sendStarrySkyState();
void sendStarrySkySpeedState();
void send230VState();
void sendDefaultRgbwBrightnessState();
void sendDefaultStarrySkyBrightnessState();
void sendBuildInfo();
void networkTask(void *pvParameters);

void setup() {
  delay(1000);
  Serial.begin(115200);

  for (int i = 0; i < 256; i++) {
    gamma8[i] = (uint8_t)(pow((float)i / 255.0, GAMMA) * 255.0 + 0.5);
  }

  TOPIC_STARRY_SKY_STATE = String(DEVICE_NAME) + "/starrysky/state";
  TOPIC_STARRY_SKY_COMMAND = String(DEVICE_NAME) + "/starrysky/set";
  TOPIC_230V_STATE = String(DEVICE_NAME) + "/230v/state";
  TOPIC_STARRY_SKY_SPEED_STATE = String(DEVICE_NAME) + "/starrysky/speed/state";
  TOPIC_STARRY_SKY_SPEED_COMMAND = String(DEVICE_NAME) + "/starrysky/speed/set";
  TOPIC_DEFAULT_STARRY_SKY_BRIGHTNESS_COMMAND = String(DEVICE_NAME) + "/config/default_starry_sky_brightness/set";
  TOPIC_DEFAULT_STARRY_SKY_BRIGHTNESS_STATE = String(DEVICE_NAME) + "/config/default_starry_sky_brightness/state";
  TOPIC_DEFAULT_RGBW_BRIGHTNESS_COMMAND = String(DEVICE_NAME) + "/config/default_rgbw_brightness/set";
  TOPIC_DEFAULT_RGBW_BRIGHTNESS_STATE = String(DEVICE_NAME) + "/config/default_rgbw_brightness/state";
  TOPIC_DEFAULT_RGBW_COLOR_COMMAND = String(DEVICE_NAME) + "/config/default_rgbw_color/set";
  TOPIC_DEFAULT_RGBW_COLOR_STATE = String(DEVICE_NAME) + "/config/default_rgbw_color/state";
  TOPIC_RGBW_COMMAND = String(DEVICE_NAME) + "/rgbw/set";
  TOPIC_RGBW_STATE = String(DEVICE_NAME) + "/rgbw/state";
  TOPIC_BUILD_INFO = String(DEVICE_NAME) + "/build_info";

  Serial.println("\nStarting Wolklamp ESP32-S3 - Core 0");

  rgbwStateMutex = xSemaphoreCreateMutex();
  starrySkyStateMutex = xSemaphoreCreateMutex();
  starrySkySpeedStateMutex = xSemaphoreCreateMutex();

  pinMode(PIN_230V_INPUT, INPUT_PULLUP);
  pinMode(PIN_TLC_BLANK, OUTPUT);
  digitalWrite(PIN_TLC_BLANK, LOW);

  strip.Begin();
  strip.Show(); 

  tlc.begin();
  tlc.setPWM(0, 0); 
  tlc.write();

  randomSeed(esp_random());

  for (int i = 0; i < TLC_NUM_CHIPS * 24; i++) {
    stars[i].phase = Star::STAR_OFF;
    stars[i].currentBrightness = 0;
    stars[i].targetBrightness = 0;
    stars[i].fadeStartTime = 0;
    stars[i].phaseStartTime = 0;
    stars[i].lastUpdateTime = 0;
    stars[i].offDuration = random(2, 10) * 1000; 
    stars[i].minBrightnessPWM = 0;
    delay(5); 
  }

  xTaskCreatePinnedToCore(
      networkTask,   
      "NetworkTask", 
      10000,         
      NULL,          
      1,             
      &NetworkTask,  
      1);            

  Serial.println("Light and input handling running on Core 0");
}

void networkTask(void *pvParameters) {
  Serial.println("Network task running on Core 1");
  setupWifi();
  
  if (WiFi.status() == WL_CONNECTED) {
    client.setServer(MQTT_SERVER, MQTT_PORT);
    client.setCallback(mqttCallback);
    client.setBufferSize(1024);

    ArduinoOTA
      .onStart([]() {
        TelnetStream.println("Start updating");
      })
      .onEnd([]() { TelnetStream.println("\nEnd"); })
      .onProgress([](unsigned int progress, unsigned int total) { TelnetStream.printf("Progress: %u%%\r", (progress / (total / 100))); })
      .onError([](ota_error_t error) {
        TelnetStream.printf("Error[%u]\n", error);
      });
    ArduinoOTA.begin();
    TelnetStream.begin();
  }

  for (;;) { 
    if (WiFi.status() == WL_CONNECTED) {
      if (!client.connected()) {
        reconnectMqtt();
      }
      client.loop();
      ArduinoOTA.handle();
    }
    vTaskDelay(10); 
  }
}

void loop() {
  handle230VInput();
  updateRgbwStripState();
  updateStarrySkyState();
  vTaskDelay(5); 
}

void setupWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(DEVICE_NAME);
  WiFiManager wm;
  wm.setConfigPortalTimeout(180); 
  if (wm.autoConnect((String(WiFi.getHostname()) + "AP").c_str())) {
    Serial.println("WiFi connected");
  } else {
    Serial.println("Failed to connect");
  }
}

void reconnectMqtt() {
  static unsigned long lastMqttReconnectAttempt = 0;
  if (millis() - lastMqttReconnectAttempt > 5000) {
    lastMqttReconnectAttempt = millis();
    if (!client.connected()) {
      Serial.print("Attempting MQTT connection...");
      String clientId = String(DEVICE_NAME) + "ESP32";
      if (client.connect(clientId.c_str(), MQTT_USER, MQTT_PASSWORD)) {
        Serial.println("connected");
        publishMqttDiscovery();

        client.subscribe(TOPIC_STARRY_SKY_COMMAND.c_str());
        client.subscribe(TOPIC_DEFAULT_STARRY_SKY_BRIGHTNESS_COMMAND.c_str());
        client.subscribe(TOPIC_STARRY_SKY_SPEED_COMMAND.c_str());
        client.subscribe(TOPIC_RGBW_COMMAND.c_str());
        client.subscribe(TOPIC_DEFAULT_RGBW_BRIGHTNESS_COMMAND.c_str());
        client.subscribe(TOPIC_DEFAULT_RGBW_COLOR_COMMAND.c_str());

        sendRgbwState();
        sendStarrySkyState();
        sendStarrySkySpeedState();
        send230VState();
        sendDefaultRgbwBrightnessState();
        sendDefaultStarrySkyBrightnessState();
        sendBuildInfo();

      } else {
        Serial.print("failed, rc=");
        Serial.println(client.state());
      }
    }
  }
}

void publishMqttDiscovery() {
  StaticJsonDocument<256> deviceDoc;
  deviceDoc["name"] = HA_DEVICE_NAME;
  JsonArray identifiers = deviceDoc.createNestedArray("identifiers");
  identifiers.add(HA_DEVICE_ID);
  deviceDoc["manufacturer"] = "Roy";
  deviceDoc["model"] = "Wolklamp ESP32-S3";
  String deviceJsonString;
  serializeJson(deviceDoc, deviceJsonString);

  StaticJsonDocument<1024> doc_rgbw;
  doc_rgbw["name"] = "Wolk Kleur";
  doc_rgbw["unique_id"] = String(HA_DEVICE_ID) + "_rgbw";
  doc_rgbw["state_topic"] = TOPIC_RGBW_STATE;
  doc_rgbw["command_topic"] = TOPIC_RGBW_COMMAND;
  doc_rgbw["schema"] = "json";
  doc_rgbw["color_mode"] = true;
  doc_rgbw["supported_color_modes"][0] = "rgbw";
  doc_rgbw["brightness"] = true;
  doc_rgbw["qos"] = 0;
  doc_rgbw["retain"] = true;
  doc_rgbw["device"] = serialized(deviceJsonString);

  String discovery_topic_rgbw = String(HA_DISCOVERY_PREFIX) + "/light/" + HA_DEVICE_ID + "_rgbw/config";
  String payload_rgbw;
  serializeJson(doc_rgbw, payload_rgbw);
  client.publish(discovery_topic_rgbw.c_str(), payload_rgbw.c_str(), true);

  StaticJsonDocument<1024> doc_starrysky;
  doc_starrysky["name"] = "Sterrenhemel";
  doc_starrysky["unique_id"] = String(HA_DEVICE_ID) + "_starrysky";
  doc_starrysky["state_topic"] = TOPIC_STARRY_SKY_STATE;
  doc_starrysky["command_topic"] = TOPIC_STARRY_SKY_COMMAND;
  doc_starrysky["schema"] = "json";
  doc_starrysky["brightness"] = true;
  doc_starrysky["color_mode"] = false;
  doc_starrysky["qos"] = 0;
  doc_starrysky["retain"] = true;
  doc_starrysky["device"] = serialized(deviceJsonString);

  String discovery_topic_starrysky = String(HA_DISCOVERY_PREFIX) + "/light/" + HA_DEVICE_ID + "_starrysky/config";
  String payload_starrysky;
  serializeJson(doc_starrysky, payload_starrysky);
  client.publish(discovery_topic_starrysky.c_str(), payload_starrysky.c_str(), true);

  StaticJsonDocument<512> doc_230v;
  doc_230v["name"] = "230V Input";
  doc_230v["unique_id"] = String(HA_DEVICE_ID) + "_230v";
  doc_230v["state_topic"] = TOPIC_230V_STATE;
  doc_230v["device_class"] = "power";
  doc_230v["device"] = serialized(deviceJsonString);
  doc_230v["qos"] = 0;
  doc_230v["retain"] = true;

  String discovery_topic_230v = String(HA_DISCOVERY_PREFIX) + "/binary_sensor/" + HA_DEVICE_ID + "_230v/config";
  String payload_230v;
  serializeJson(doc_230v, payload_230v);
  client.publish(discovery_topic_230v.c_str(), payload_230v.c_str(), true);

  StaticJsonDocument<512> doc_default_rgbw_brightness;
  doc_default_rgbw_brightness["name"] = "Default RGBW Helderheid";
  doc_default_rgbw_brightness["unique_id"] = String(HA_DEVICE_ID) + "_default_rgbw_brightness";
  doc_default_rgbw_brightness["command_topic"] = TOPIC_DEFAULT_RGBW_BRIGHTNESS_COMMAND;
  doc_default_rgbw_brightness["state_topic"] = TOPIC_DEFAULT_RGBW_BRIGHTNESS_STATE;
  doc_default_rgbw_brightness["min"] = 0;
  doc_default_rgbw_brightness["max"] = 100;
  doc_default_rgbw_brightness["step"] = 1;
  doc_default_rgbw_brightness["unit_of_measurement"] = "%";
  doc_default_rgbw_brightness["device"] = serialized(deviceJsonString);
  doc_default_rgbw_brightness["value_template"] = "{{ value_json.value }}";
  doc_default_rgbw_brightness["qos"] = 0;
  doc_default_rgbw_brightness["retain"] = true;

  String discovery_topic_default_rgbw_brightness = String(HA_DISCOVERY_PREFIX) + "/number/" + HA_DEVICE_ID + "_default_rgbw_brightness/config";
  String payload_default_rgbw_brightness;
  serializeJson(doc_default_rgbw_brightness, payload_default_rgbw_brightness);
  client.publish(discovery_topic_default_rgbw_brightness.c_str(), payload_default_rgbw_brightness.c_str(), true);

  StaticJsonDocument<512> doc_default_starry_sky_brightness;
  doc_default_starry_sky_brightness["name"] = "Default Sterrenhemel Helderheid";
  doc_default_starry_sky_brightness["unique_id"] = String(HA_DEVICE_ID) + "_default_starry_sky_brightness";
  doc_default_starry_sky_brightness["command_topic"] = TOPIC_DEFAULT_STARRY_SKY_BRIGHTNESS_COMMAND;
  doc_default_starry_sky_brightness["state_topic"] = TOPIC_DEFAULT_STARRY_SKY_BRIGHTNESS_STATE;
  doc_default_starry_sky_brightness["min"] = 0;
  doc_default_starry_sky_brightness["max"] = 100;
  doc_default_starry_sky_brightness["step"] = 1;
  doc_default_starry_sky_brightness["unit_of_measurement"] = "%";
  doc_default_starry_sky_brightness["device"] = serialized(deviceJsonString);
  doc_default_starry_sky_brightness["value_template"] = "{{ value_json.value }}";
  doc_default_starry_sky_brightness["qos"] = 0;
  doc_default_starry_sky_brightness["retain"] = true;

  String discovery_topic_default_starry_sky_brightness = String(HA_DISCOVERY_PREFIX) + "/number/" + HA_DEVICE_ID + "_default_starry_sky_brightness/config";
  String payload_default_starry_sky_brightness;
  serializeJson(doc_default_starry_sky_brightness, payload_default_starry_sky_brightness);
  client.publish(discovery_topic_default_starry_sky_brightness.c_str(), payload_default_starry_sky_brightness.c_str(), true);

  // Starry Sky Speed Discovery (Number entity)
  StaticJsonDocument<512> doc_starry_sky_speed;
  doc_starry_sky_speed["name"] = "Sterrenhemel Snelheid";
  doc_starry_sky_speed["unique_id"] = String(HA_DEVICE_ID) + "_starry_sky_speed";
  doc_starry_sky_speed["command_topic"] = TOPIC_STARRY_SKY_SPEED_COMMAND;
  doc_starry_sky_speed["state_topic"] = TOPIC_STARRY_SKY_SPEED_STATE;
  doc_starry_sky_speed["min"] = 0;
  doc_starry_sky_speed["max"] = 100;
  doc_starry_sky_speed["step"] = 1;
  doc_starry_sky_speed["unit_of_measurement"] = "%";
  doc_starry_sky_speed["device"] = serialized(deviceJsonString);
  doc_starry_sky_speed["value_template"] = "{{ value_json.value }}";
  doc_starry_sky_speed["qos"] = 0;
  doc_starry_sky_speed["retain"] = true;

  String discovery_topic_starry_sky_speed = String(HA_DISCOVERY_PREFIX) + "/number/" + HA_DEVICE_ID + "_starry_sky_speed/config";
  String payload_starry_sky_speed;
  serializeJson(doc_starry_sky_speed, payload_starry_sky_speed);
  client.publish(discovery_topic_starry_sky_speed.c_str(), payload_starry_sky_speed.c_str(), true);

  // Build Info Discovery
  StaticJsonDocument<512> doc_build_info;
  doc_build_info["name"] = "Build Info";
  doc_build_info["unique_id"] = String(HA_DEVICE_ID) + "_build_info";
  doc_build_info["state_topic"] = TOPIC_BUILD_INFO;
  doc_build_info["device"] = serialized(deviceJsonString);
  doc_build_info["value_template"] = "{{ value_json.file }} - {{ value_json.date }} {{ value_json.time }}";
  doc_build_info["qos"] = 0;
  doc_build_info["retain"] = true;

  String discovery_topic_build_info = String(HA_DISCOVERY_PREFIX) + "/sensor/" + HA_DEVICE_ID + "_build_info/config";
  String payload_build_info;
  serializeJson(doc_build_info, payload_build_info);
  client.publish(discovery_topic_build_info.c_str(), payload_build_info.c_str(), true);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message_payload = "";
  for (int i = 0; i < length; i++) {
    message_payload += (char)payload[i];
  }
  TelnetStream.printf("Received MQTT message on topic %s: %s\n", topic, message_payload.c_str());
  StaticJsonDocument<512> doc;
  deserializeJson(doc, message_payload);

  if (strcmp(topic, TOPIC_RGBW_COMMAND.c_str()) == 0) {
    if (xSemaphoreTake(rgbwStateMutex, portMAX_DELAY) == pdTRUE) {
      if (doc.containsKey("state")) { rgbw_on = (strcmp(doc["state"], "ON") == 0); }
      if (doc.containsKey("brightness")) { targetRgbwBrightness = doc["brightness"].as<uint8_t>(); }
      if (doc.containsKey("color")) {
        targetRgbwColor.R = doc["color"]["r"].as<uint8_t>();
        targetRgbwColor.G = doc["color"]["g"].as<uint8_t>();
        targetRgbwColor.B = doc["color"]["b"].as<uint8_t>();
        if (doc["color"].containsKey("w")) { targetRgbwColor.W = doc["color"]["w"].as<uint8_t>(); }
      }
      xSemaphoreGive(rgbwStateMutex);
    }
    sendRgbwState();
  }
  else if (strcmp(topic, TOPIC_STARRY_SKY_COMMAND.c_str()) == 0) {
    if (xSemaphoreTake(starrySkyStateMutex, portMAX_DELAY) == pdTRUE) {
      if (doc.containsKey("state")) { starry_sky_on = (strcmp(doc["state"], "ON") == 0); }
      if (doc.containsKey("brightness")) { targetStarrySkyBrightness = doc["brightness"].as<uint8_t>(); }
      xSemaphoreGive(starrySkyStateMutex);
    }
    sendStarrySkyState();
  }

  else if (strcmp(topic, TOPIC_DEFAULT_RGBW_COLOR_COMMAND.c_str()) == 0) { if (doc.containsKey("color")) { defaultRgbwColorOn230V.R = doc["color"]["r"].as<uint8_t>(); defaultRgbwColorOn230V.G = doc["color"]["g"].as<uint8_t>(); defaultRgbwColorOn230V.B = doc["color"]["b"].as<uint8_t>(); if (doc["color"].containsKey("w")) { defaultRgbwColorOn230V.W = doc["color"]["w"].as<uint8_t>(); } } }
  else if (strcmp(topic, TOPIC_DEFAULT_RGBW_BRIGHTNESS_COMMAND.c_str()) == 0) {
    uint8_t value = 0;
    if (doc.containsKey("value")) {
      value = (uint8_t)(doc["value"].as<uint8_t>() * 2.55); // Convert 0-100 to 0-255
    } else if (message_payload.length() > 0 && isdigit(message_payload[0])) {
      value = (uint8_t)(atoi(message_payload.c_str()) * 2.55); // Convert 0-100 to 0-255
    }
    if (value > 0 || message_payload == "0") {
      defaultRgbwBrightnessOn230V = value;
      TelnetStream.printf("Default RGBW brightness on 230V set to: %d\n", defaultRgbwBrightnessOn230V);
      sendDefaultRgbwBrightnessState();
    }
  }  
  else if (strcmp(topic, TOPIC_DEFAULT_STARRY_SKY_BRIGHTNESS_COMMAND.c_str()) == 0) {
    uint8_t value = 0;
    if (doc.containsKey("value")) {
      value = (uint8_t)(doc["value"].as<uint8_t>() * 2.55); // Convert 0-100 to 0-255
    } else if (message_payload.length() > 0 && isdigit(message_payload[0])) {
      value = (uint8_t)(atoi(message_payload.c_str()) * 2.55); // Convert 0-100 to 0-255
    }
    if (value > 0 || message_payload == "0") {
      defaultStarrySkyBrightnessOn230V = value;
      TelnetStream.printf("Default Starry Sky brightness on 230V set to: %d\n", defaultStarrySkyBrightnessOn230V);
      sendDefaultStarrySkyBrightnessState();
    }
  }
  else if (strcmp(topic, TOPIC_STARRY_SKY_SPEED_COMMAND.c_str()) == 0) {
    if (xSemaphoreTake(starrySkySpeedStateMutex, portMAX_DELAY) == pdTRUE) {
      uint8_t value = 0;
      if (doc.containsKey("value")) {
        value = doc["value"].as<uint8_t>();
      } else if (message_payload.length() > 0 && isdigit(message_payload[0])) {
        value = atoi(message_payload.c_str());
      }
      if (value > 0 || message_payload == "0") {
        targetStarrySkySpeed = value;
        TelnetStream.printf("Starry Sky speed set to: %d%%\n", targetStarrySkySpeed);
      }
      xSemaphoreGive(starrySkySpeedStateMutex);
    }
    sendStarrySkySpeedState();
  }
}
void handle230VInput() {
  bool reading = digitalRead(PIN_230V_INPUT);
  
  // Debouncing
  if (reading != lastButtonState) { 
    lastButtonPressTime = millis(); 
  }
  
  if ((millis() - lastButtonPressTime) > debounceDelay) {
    // React only on state changes (edges)
    if (reading != buttonState) {
      buttonState = reading;
      
      if (xSemaphoreTake(rgbwStateMutex, portMAX_DELAY) == pdTRUE &&
          xSemaphoreTake(starrySkyStateMutex, portMAX_DELAY) == pdTRUE) {
        
        // Falling edge: 230V present (buttonState goes LOW)
        if (buttonState == LOW) { 
          is230vAbsent = false;
          bool longPressDetected = (millis() - input230vAbsentStartTime) > 2000;
          if (longPressDetected) {
            // Long press: activate RGBW with default settings
            rgbw_on = true; 
            targetRgbwBrightness = defaultRgbwBrightnessOn230V; 
            targetRgbwColor = defaultRgbwColorOn230V;
            starry_sky_on = false;
            currentActiveLightSource = RGBW_STRIP;
          } else {
            // Short press: toggle between light sources
            switch (currentActiveLightSource) {
              case NONE:
              case STARRY_SKY:
                starry_sky_on = false;
                rgbw_on = true; 
                targetRgbwBrightness = defaultRgbwBrightnessOn230V; 
                targetRgbwColor = defaultRgbwColorOn230V;
                currentActiveLightSource = RGBW_STRIP;
                break;
              case RGBW_STRIP:
                rgbw_on = false;
                starry_sky_on = true; 
                targetStarrySkyBrightness = defaultStarrySkyBrightnessOn230V;
                currentActiveLightSource = STARRY_SKY;
                break;
            }
          }
        } 
        // Rising edge: 230V absent (buttonState goes HIGH)
        else { 
          is230vAbsent = true;
          input230vAbsentStartTime = millis();
          rgbw_on = false;
          starry_sky_on = false;
        }
        
        xSemaphoreGive(starrySkyStateMutex);
        xSemaphoreGive(rgbwStateMutex);
        send230VState();
      }
    }
  }
  
  lastButtonState = reading;
}

void updateRgbwStripState() {
  unsigned long currentTime = millis();
  if (currentTime - lastRgbwUpdateTime < 10) return;
  lastRgbwUpdateTime = currentTime;

  bool on_status;
  RgbwColor finalTargetColor;
  uint8_t finalTargetBrightness;

  if (xSemaphoreTake(rgbwStateMutex, (TickType_t)5) == pdTRUE) {
    on_status = rgbw_on;
    finalTargetColor = on_status ? targetRgbwColor : RgbwColor(0);
    finalTargetBrightness = on_status ? targetRgbwBrightness : 0;
    xSemaphoreGive(rgbwStateMutex);
  } else { return; }

  if (currentRgbwDisplayBrightness == finalTargetBrightness && currentRgbwDisplayColor == finalTargetColor) return;

  if (currentRgbwDisplayBrightness != finalTargetBrightness) {
      float step = (256.0 / (rgbwFadeDuration / 10.0));
      if (finalTargetBrightness > currentRgbwDisplayBrightness) {
          currentRgbwDisplayBrightnessFading += step;
          if (currentRgbwDisplayBrightnessFading > (float)finalTargetBrightness) currentRgbwDisplayBrightnessFading = (float)finalTargetBrightness;
      } else {
          currentRgbwDisplayBrightnessFading -= step;
          if (currentRgbwDisplayBrightnessFading < (float)finalTargetBrightness) currentRgbwDisplayBrightnessFading = (float)finalTargetBrightness;
      }
      currentRgbwDisplayBrightness = (uint8_t)currentRgbwDisplayBrightnessFading;
  }
  
  if (on_status) {
      currentRgbwDisplayColor = finalTargetColor; 
  }

  RgbwColor appliedColor = currentRgbwDisplayColor;
  uint8_t appliedBrightness = currentRgbwDisplayBrightness;

  // Spatial Dithering for lower minimum brightness
  // Below brightness 40, gamma correction normally drops to 0.
  // We fix this by keeping the LED intensity at 40 (min visible) 
  // and increasing the interval between LEDs.
  int pixelInterval = 1;
  uint8_t colorBrightness = appliedBrightness;

  if (appliedBrightness > 0 && appliedBrightness < 40) {
    colorBrightness = 40; // Lock intensity at minimum visible level
    pixelInterval = 40 / appliedBrightness; // Increase interval to dim further
    if (pixelInterval > 40) pixelInterval = 40; 
  }

  appliedColor.R = map(appliedColor.R, 0, 255, 0, colorBrightness);
  appliedColor.G = map(appliedColor.G, 0, 255, 0, colorBrightness);
  appliedColor.B = map(appliedColor.B, 0, 255, 0, colorBrightness);
  appliedColor.W = map(appliedColor.W, 0, 255, 0, colorBrightness);
  
  appliedColor.R = gamma8[appliedColor.R];
  appliedColor.G = gamma8[appliedColor.G];
  appliedColor.B = gamma8[appliedColor.B];
  appliedColor.W = gamma8[appliedColor.W];

  int totalValue = appliedColor.R + appliedColor.G + appliedColor.B + (2*appliedColor.W);
  if (totalValue > MAX_TOTAL_CHANNEL_VALUE) {
    float scale = (float)MAX_TOTAL_CHANNEL_VALUE / (float)totalValue;
    appliedColor.R = (uint8_t)(appliedColor.R * scale);
    appliedColor.G = (uint8_t)(appliedColor.G * scale);
    appliedColor.B = (uint8_t)(appliedColor.B * scale);
    appliedColor.W = (uint8_t)(appliedColor.W * scale);
  }

  for (uint16_t i = 0; i < PixelCount; i++) {
    if (i % pixelInterval == 0) {
      strip.SetPixelColor(i, appliedColor);
    } else {
      strip.SetPixelColor(i, RgbwColor(0));
    }
  }
  strip.Show();

  if (currentRgbwDisplayBrightness == finalTargetBrightness &&
      currentRgbwDisplayColor == finalTargetColor) {
      sendRgbwState();
  }
}

void updateStarrySkyState() {
  unsigned long currentTime = millis();
  if (currentTime - lastStarrySkyUpdateTime < 1) return;
  lastStarrySkyUpdateTime = currentTime;
  
  bool on_status;
  uint8_t finalTargetBrightness;
  uint8_t speed;

  if (xSemaphoreTake(starrySkyStateMutex, (TickType_t)5) == pdTRUE) {
    on_status = starry_sky_on;
    finalTargetBrightness = on_status ? targetStarrySkyBrightness : 0;
    xSemaphoreGive(starrySkyStateMutex);
  } else { return; }

  if (xSemaphoreTake(starrySkySpeedStateMutex, (TickType_t)5) == pdTRUE) {
    speed = targetStarrySkySpeed;
    xSemaphoreGive(starrySkySpeedStateMutex);
  } else { return; }

  if (finalTargetBrightness > currentStarrySkyBrightness) {
    currentStarrySkyBrightnessFading = currentStarrySkyBrightnessFading + (256.0 / (starrySkyFadeDuration / 1.0));
    if (currentStarrySkyBrightnessFading > (float)finalTargetBrightness) currentStarrySkyBrightnessFading = (float)finalTargetBrightness;
    currentStarrySkyBrightness = (uint8_t)currentStarrySkyBrightnessFading;
  } else {
    currentStarrySkyBrightnessFading = currentStarrySkyBrightnessFading - (256.0 / (starrySkyFadeDuration / 1.0));
    if (currentStarrySkyBrightnessFading < (float)finalTargetBrightness) currentStarrySkyBrightnessFading = (float)finalTargetBrightness;
    currentStarrySkyBrightness = (uint8_t)currentStarrySkyBrightnessFading;
  }
  
  if(on_status != starry_sky_last_state) sendStarrySkyState();
  starry_sky_last_state = on_status; 

  float speedScale;
  if (speed <= 50) speedScale = map(speed, 0, 50, 500, 100) / 100.0;
  else speedScale = map(speed, 50, 100, 100, 10) / 100.0;

  if (currentTime - lastStarUpdateTime < starUpdateInterval) return;
  lastStarUpdateTime = currentTime;

  for (int i = 0; i < TLC_NUM_CHIPS * 24; i++) {
    Star& star = stars[i];
    switch (star.phase) {
      case Star::STAR_OFF:
        if (currentTime - star.phaseStartTime > (unsigned long)(star.offDuration * speedScale)) {
          star.phase = Star::STAR_FADE_IN;
          star.fadeInDuration = random(2, 10) * 1000;
          star.maxBrightnessPWM = random((int)(0.50 * 4095), (int)(1.00 * 4095));
          star.fadeStartTime = currentTime;
          star.targetBrightness = star.maxBrightnessPWM;
        }
        break;
      case Star::STAR_FADE_IN: {
        unsigned long elapsed = currentTime - star.fadeStartTime;
        unsigned long scaledFadeInDuration = (unsigned long)(star.fadeInDuration * speedScale);
        if (scaledFadeInDuration > 0) star.currentBrightness = map(elapsed, 0, scaledFadeInDuration, star.minBrightnessPWM, star.maxBrightnessPWM);
        else star.currentBrightness = star.maxBrightnessPWM;
        if (elapsed >= scaledFadeInDuration) {
          star.currentBrightness = star.maxBrightnessPWM;
          star.phase = Star::STAR_ON;
          star.onDuration = random(2, 10) * 1000;
          star.phaseStartTime = currentTime;
        }
        break;
      }
      case Star::STAR_ON:
        if (currentTime - star.phaseStartTime > (unsigned long)(star.onDuration * speedScale)) {
          star.phase = Star::STAR_FADE_OUT;
          star.fadeOutDuration = random(2, 10) * 1000;
          star.fadeStartTime = currentTime;
          star.targetBrightness = star.minBrightnessPWM;
        }
        break;
      case Star::STAR_FADE_OUT: {
        unsigned long elapsed = currentTime - star.fadeStartTime;
        unsigned long scaledFadeOutDuration = (unsigned long)(star.fadeOutDuration * speedScale);
        if (scaledFadeOutDuration > 0) star.currentBrightness = map(elapsed, 0, scaledFadeOutDuration, star.maxBrightnessPWM, star.minBrightnessPWM);
        else star.currentBrightness = star.minBrightnessPWM;
        if (elapsed >= scaledFadeOutDuration) {
          star.currentBrightness = star.minBrightnessPWM;
          star.phase = Star::STAR_OFF;
          star.offDuration = random(2, 10) * 2000;
          star.phaseStartTime = currentTime;
        }
        break;
      }
    }
    uint16_t finalPWM = map(star.currentBrightness, 0, 4095, 0, map(gamma8[currentStarrySkyBrightness], 0, 255, 0, 4095));
    tlc.setPWM(i, finalPWM);
    if (i == 100){
      Serial.printf("Star %d - Phase: %d, Brightness: %d, Target: %d, Final PWM: %d        \r", i, star.phase, star.currentBrightness, star.targetBrightness, finalPWM);
    }
  }
  digitalWrite(PIN_TLC_BLANK, HIGH);
  delayMicroseconds(2);
  tlc.write();
  digitalWrite(PIN_TLC_BLANK, LOW);
}

void sendRgbwState() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (xSemaphoreTake(rgbwStateMutex, (TickType_t)10) == pdTRUE) {
    StaticJsonDocument<256> doc;
    doc["state"] = rgbw_on ? "ON" : "OFF";
    doc["brightness"] = targetRgbwBrightness;
    doc["color_mode"] = "rgbw"; 
    JsonObject color = doc.createNestedObject("color");
    color["r"] = targetRgbwColor.R;
    color["g"] = targetRgbwColor.G;
    color["b"] = targetRgbwColor.B;
    color["w"] = targetRgbwColor.W;
    String payload;
    serializeJson(doc, payload);
    client.publish(TOPIC_RGBW_STATE.c_str(), payload.c_str(), true);
    xSemaphoreGive(rgbwStateMutex);
  }
}

void sendStarrySkyState() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (xSemaphoreTake(starrySkyStateMutex, (TickType_t)10) == pdTRUE) {
    StaticJsonDocument<256> doc;
    doc["state"] = starry_sky_on ? "ON" : "OFF";
    doc["brightness"] = targetStarrySkyBrightness;
    String payload;
    serializeJson(doc, payload);
    client.publish(TOPIC_STARRY_SKY_STATE.c_str(), payload.c_str(), true);
    xSemaphoreGive(starrySkyStateMutex);
  }
}

void sendStarrySkySpeedState() {
  TelnetStream.printf("Sending starry sky speed state: %d%%\n", targetStarrySkySpeed);
  if (WiFi.status() != WL_CONNECTED) return;
  if (xSemaphoreTake(starrySkySpeedStateMutex, (TickType_t)10) == pdTRUE) {
    StaticJsonDocument<64> doc;
    doc["value"] = targetStarrySkySpeed;
    String payload;
    serializeJson(doc, payload);
    TelnetStream.printf("Publishing starry sky speed state: %s\n", payload.c_str());
    client.publish(TOPIC_STARRY_SKY_SPEED_STATE.c_str(), payload.c_str(), true);
    xSemaphoreGive(starrySkySpeedStateMutex);
  }
}

void send230VState() {
  if (WiFi.status() != WL_CONNECTED) return;
  const char* payload = (buttonState == LOW) ? "ON" : "OFF";
  client.publish(TOPIC_230V_STATE.c_str(), payload, true);
}

void sendDefaultRgbwBrightnessState() {
  if (WiFi.status() != WL_CONNECTED) return;
  StaticJsonDocument<64> doc;
  doc["value"] = (uint8_t)(defaultRgbwBrightnessOn230V / 2.55)+1; // Convert 0-255 to 0-100

  String payload;
  serializeJson(doc, payload);
  client.publish(TOPIC_DEFAULT_RGBW_BRIGHTNESS_STATE.c_str(), payload.c_str(), true);
}

void sendDefaultStarrySkyBrightnessState() {
  if (WiFi.status() != WL_CONNECTED) return;
  StaticJsonDocument<64> doc;
  doc["value"] = (uint8_t)(defaultStarrySkyBrightnessOn230V / 2.55)+1; // Convert 0-255 to 0-100
  String payload;
  serializeJson(doc, payload);
  client.publish(TOPIC_DEFAULT_STARRY_SKY_BRIGHTNESS_STATE.c_str(), payload.c_str(), true);
}

void sendBuildInfo() {
  if (WiFi.status() != WL_CONNECTED) return;
  StaticJsonDocument<256> doc;
  // PROJECT_PATH is defined in platformio.ini as the absolute directory
  // __FILE__ is typically "src/main.cpp" or just "main.cpp" relative to source dir
  doc["file"] = String(PROJECT_PATH) + "/" + String(__FILE__);
  doc["date"] = __DATE__;
  doc["time"] = __TIME__;
  String payload;
  serializeJson(doc, payload);
  client.publish(TOPIC_BUILD_INFO.c_str(), payload.c_str(), true);
}