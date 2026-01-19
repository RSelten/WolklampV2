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
// To limit total current to ~800mA.
// Max current (7.2A) is at R=G=B=W=255. The sum of channel values after brightness is 1020.
// The desired current is 800mA.
// Scaling factor = 800 / 7200 = 1/9.
// Max channel sum = 1020 * (1/9) = 113.3. We'll use 113.
const int MAX_TOTAL_CHANNEL_VALUE = 113;

// --- Task and Mutex Handles ---
TaskHandle_t NetworkTask;
SemaphoreHandle_t rgbwStateMutex;
SemaphoreHandle_t mosfetStateMutex;
SemaphoreHandle_t starrySkyStateMutex;

// --- Configuration ---
// MQTT
const char* MQTT_SERVER = "192.168.1.126";
const int MQTT_PORT = 1883;
const char* MQTT_USER = "mqtt";
const char* MQTT_PASSWORD = "mqtt";
const char* MQTT_CLIENT_ID = "WolklampESP32"; // Unique client ID

// Home Assistant MQTT Discovery
const char* HA_DISCOVERY_PREFIX = "homeassistant";
const char* HA_DEVICE_NAME = "Wolklamp";
const char* HA_DEVICE_ID = "wolklamp_esp32_s3"; // Unique ID for the device

// MQTT Topics
// Wolk Kleur (RGBW Strip)
const char* TOPIC_RGBW_STATE = "wolklamp/rgbw/state";
const char* TOPIC_RGBW_COMMAND = "wolklamp/rgbw/set";
// Wolk Wit (Mosfet Strip)
const char* TOPIC_MOSFET_STATE = "wolklamp/mosfet/state";
const char* TOPIC_MOSFET_COMMAND = "wolklamp/mosfet/set";
// Sterrenhemel
const char* TOPIC_STARRY_SKY_STATE = "wolklamp/starrysky/state";
const char* TOPIC_STARRY_SKY_COMMAND = "wolklamp/starrysky/set";

// MQTT Topics for 230V input default settings
const char* TOPIC_DEFAULT_RGBW_BRIGHTNESS_COMMAND = "wolklamp/config/default_rgbw_brightness/set";
const char* TOPIC_DEFAULT_RGBW_BRIGHTNESS_STATE = "wolklamp/config/default_rgbw_brightness/state";
const char* TOPIC_DEFAULT_RGBW_COLOR_COMMAND = "wolklamp/config/default_rgbw_color/set";
const char* TOPIC_DEFAULT_MOSFET_BRIGHTNESS_COMMAND = "wolklamp/config/default_mosfet_brightness/set";
const char* TOPIC_DEFAULT_MOSFET_BRIGHTNESS_STATE = "wolklamp/config/default_mosfet_brightness/state";
const char* TOPIC_DEFAULT_STARRY_SKY_BRIGHTNESS_COMMAND = "wolklamp/config/default_starry_sky_brightness/set";
const char* TOPIC_DEFAULT_STARRY_SKY_BRIGHTNESS_STATE = "wolklamp/config/default_starry_sky_brightness/state";

// Pin Definitions
#define PIN_RGBW_STRIP      2   // IO2 = Output SK6812 RGBW strip 120 leds
#define PIN_MOSFET_STRIP    9   // IO9 = Output Mosfet led strip
#define PIN_TLC_SIN         14  // IO14 = TLC5947 SIN
#define PIN_TLC_SCLK        21  // IO21 = TLC5947 SCLK
#define PIN_TLC_BLANK       47  // IO47 = TLC5947 BLANK
#define PIN_TLC_XLAT        48  // IO48 = TLC5947 XLAT
#define PIN_230V_INPUT      10  // IO10 = 230V input 1

// RGBW Strip
const uint16_t PixelCount = 120; // this is for 120 pixels
NeoPixelBus<NeoGrbwFeature, Neo800KbpsMethod> strip(PixelCount, PIN_RGBW_STRIP);
bool rgbw_on = false;
RgbwColor targetRgbwColor(0, 0, 0, 0); // Target color
uint8_t targetRgbwBrightness = 0;      // Target brightness

// TLC5947 Starry Sky
#define TLC_NUM_CHIPS   1    // One TLC5947 chip for 24 channels
Adafruit_TLC5947 tlc = Adafruit_TLC5947(TLC_NUM_CHIPS, PIN_TLC_SCLK, PIN_TLC_SIN, PIN_TLC_XLAT);
bool starry_sky_on = false;
uint8_t targetStarrySkyBrightness = 0; // Master brightness for starry sky

// MOSFET Strip PWM
const int MosfetLedChannel = 0; // Use LEDC channel 0
const int MosfetPwmFreq = 25000; // 25 KHz to avoid audible noise
const int MosfetPwmResolution = 10; // 10-bit resolution (0-1023)
bool mosfet_on = false;
uint8_t targetMosfetBrightness = 0; // Brightness for MOSFET strip

// Default settings for 230V input trigger
uint8_t defaultRgbwBrightnessOn230V = 255; // Initial max brightness
RgbwColor defaultRgbwColorOn230V(255, 255, 255, 255); // Initial white color
uint8_t defaultMosfetBrightnessOn230V = 255; // Initial max brightness
uint8_t defaultStarrySkyBrightnessOn230V = 255; // Initial max brightness

// RGBW Strip Fading
RgbwColor currentRgbwDisplayColor(0, 0, 0, 0); // What is currently displayed
uint8_t currentRgbwDisplayBrightness = 0;      // What is currently displayed
unsigned long lastRgbwUpdateTime = 0;
const unsigned long rgbwFadeDuration = 1000; // 1 second fade duration

// MOSFET Strip Fading
uint8_t currentMosfetDisplayBrightness = 0; // What is currently displayed
unsigned long lastMosfetUpdateTime = 0;
const unsigned long mosfetFadeDuration = 1000; // 1 second fade duration

// 230V Input Button State
unsigned long lastButtonPressTime = 0;
bool buttonState = HIGH;      // current state of the button
bool lastButtonState = HIGH;  // previous state of the button
const long debounceDelay = 50; // debounce time; increase if needed
bool is230vAbsent = false; // Flag to track if 230V input is currently absent
unsigned long input230vAbsentStartTime = 0; // Timestamp when 230V input went absent

// Light Source Toggle State
enum LightSource { MOSFET_STRIP, RGBW_STRIP, STARRY_SKY, NONE };
LightSource currentActiveLightSource = NONE;

// Starry Sky Animation
struct Star {
  uint16_t currentBrightness; // 0-4095 for TLC5947
  uint16_t targetBrightness;
  unsigned long fadeStartTime;
  unsigned long phaseStartTime;
  unsigned long lastUpdateTime;
  enum { STAR_OFF, STAR_FADE_IN, STAR_ON, STAR_FADE_OUT } phase;

  // Random parameters for each star
  uint16_t minBrightnessPWM; // 0-5% of 4095
  uint16_t maxBrightnessPWM; // 50-100% of 4095
  unsigned long minActiveDuration; // 0-10s
  unsigned long onDuration; // 0-10s
  unsigned long fadeInDuration;    // 0-10s
  unsigned long fadeOutDuration;   // 0-10s
};

Star stars[TLC_NUM_CHIPS * 24]; // Array for 24 stars
unsigned long lastStarUpdateTime = 0;
const unsigned long starUpdateInterval = 20; // Update stars every 20ms

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
void updateMosfetStripState();
void updateStarrySkyState();
void sendRgbwState();
void sendMosfetState();
void sendStarrySkyState();
void sendDefaultRgbwBrightnessState();
void sendDefaultMosfetBrightnessState();
void sendDefaultStarrySkyBrightnessState();
void networkTask(void *pvParameters);

void setup() {
  Serial.begin(115200);

  // Pre-calculate gamma correction table
  for (int i = 0; i < 256; i++) {
    gamma8[i] = (uint8_t)(pow((float)i / 255.0, GAMMA) * 255.0 + 0.5);
  }

  Serial.println("\nStarting Wolklamp ESP32-S3 - Core 0");

  // Create mutexes for shared state variables
  rgbwStateMutex = xSemaphoreCreateMutex();
  mosfetStateMutex = xSemaphoreCreateMutex();
  starrySkyStateMutex = xSemaphoreCreateMutex();

  // Pin modes
  pinMode(PIN_230V_INPUT, INPUT_PULLUP); // Assuming input is active low or requires pullup

  // Initialize RGBW strip
  strip.Begin();
  strip.Show(); // Initialize all pixels to 'off'

  // Initialize TLC5947
  tlc.begin();
  tlc.setPWM(0, 0); // Set all channels to off initially
  tlc.write();

  // Initialize MOSFET PWM
  ledcSetup(MosfetLedChannel, MosfetPwmFreq, MosfetPwmResolution);
  ledcAttachPin(PIN_MOSFET_STRIP, MosfetLedChannel);
  ledcWrite(MosfetLedChannel, 0); // Start with MOSFET strip off

  // Seed random number generator
  randomSeed(esp_random());

  // Initialize Starry Sky parameters
  for (int i = 0; i < TLC_NUM_CHIPS * 24; i++) {
    stars[i].phase = Star::STAR_OFF;
    stars[i].currentBrightness = 0;
    stars[i].targetBrightness = 0;
    stars[i].fadeStartTime = 0;
    stars[i].phaseStartTime = 0;
    stars[i].lastUpdateTime = 0;

    // Generate random parameters for each star
    stars[i].minBrightnessPWM = random(0, (int)(0.05 * 4095));
    stars[i].maxBrightnessPWM = random((int)(0.50 * 4095), (int)(1.00 * 4095));
    stars[i].minActiveDuration = random(0, 10) * 1000;
    stars[i].onDuration = random(0, 10) * 1000;
    stars[i].fadeInDuration = random(0, 10) * 1000;
    stars[i].fadeOutDuration = random(0, 10) * 1000;
  }

  // Create a task for networking and pin it to Core 1
  xTaskCreatePinnedToCore(
      networkTask,   /* Task function. */
      "NetworkTask", /* name of task. */
      10000,         /* Stack size of task */
      NULL,          /* parameter of the task */
      1,             /* priority of the task */
      &NetworkTask,  /* Task handle to keep track of created task */
      1);            /* pin task to core 1 */

  Serial.println("Light and input handling running on Core 0");
}

// Core 1: Handles all networking
void networkTask(void *pvParameters) {
  Serial.println("Network task running on Core 1");

  setupWifi();
  
  if (WiFi.status() == WL_CONNECTED) {
    client.setServer(MQTT_SERVER, MQTT_PORT);
    client.setCallback(mqttCallback);
    client.setBufferSize(1024);

    // OTA setup
    ArduinoOTA
      .onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH) type = "sketch";
        else type = "filesystem";
        TelnetStream.println("Start updating " + type);
      })
      .onEnd([]() { TelnetStream.println("\nEnd"); })
      .onProgress([](unsigned int progress, unsigned int total) { TelnetStream.printf("Progress: %u%%\r", (progress / (total / 100))); })
      .onError([](ota_error_t error) {
        TelnetStream.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) TelnetStream.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) TelnetStream.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) TelnetStream.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) TelnetStream.println("Receive Failed");
        else if (error == OTA_END_ERROR) TelnetStream.println("End Failed");
      });
    ArduinoOTA.begin();
    TelnetStream.begin();
  }

  for (;;) { // Infinite loop for the task
    if (WiFi.status() == WL_CONNECTED) {
      if (!client.connected()) {
        reconnectMqtt();
      }
      client.loop();
      ArduinoOTA.handle();
    } else {
      // If WiFi disconnects, maybe try to reconnect periodically.
      // For now, WiFiManager handles this on startup. A more robust solution could go here.
    }
    vTaskDelay(10); // Small delay to prevent watchdog timeout
  }
}

// Core 0: Handles lights and physical inputs
void loop() {
  handle230VInput();
  updateRgbwStripState();
  updateMosfetStripState();
  updateStarrySkyState();
  
  // The main loop can have a small delay if needed, as time-critical things are handled by their own timers/tasks
  vTaskDelay(5); 
}

void setupWifi() {
  WiFi.mode(WIFI_STA);
  WiFiManager wm;
  wm.setConfigPortalTimeout(180); // 3-minute timeout for the portal

  if (wm.autoConnect("WolklampAP")) {
    Serial.println("WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("Failed to connect and hit timeout");
    // ESP.restart(); // Consider restarting if WiFi is essential and fails to connect
  }
}

void reconnectMqtt() {
  static unsigned long lastMqttReconnectAttempt = 0;
  if (millis() - lastMqttReconnectAttempt > 5000) {
    lastMqttReconnectAttempt = millis();
    if (!client.connected()) {
      Serial.print("Attempting MQTT connection...");
      if (client.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD)) {
        Serial.println("connected");
        TelnetStream.println("MQTT connected");
        publishMqttDiscovery();

        client.subscribe(TOPIC_RGBW_COMMAND);
        client.subscribe(TOPIC_MOSFET_COMMAND);
        client.subscribe(TOPIC_STARRY_SKY_COMMAND);
        client.subscribe(TOPIC_DEFAULT_RGBW_BRIGHTNESS_COMMAND);
        client.subscribe(TOPIC_DEFAULT_RGBW_COLOR_COMMAND);
        client.subscribe(TOPIC_DEFAULT_MOSFET_BRIGHTNESS_COMMAND);
        client.subscribe(TOPIC_DEFAULT_STARRY_SKY_BRIGHTNESS_COMMAND);

        sendRgbwState();
        sendMosfetState();
        sendStarrySkyState();
        sendDefaultRgbwBrightnessState();
        sendDefaultMosfetBrightnessState();
        sendDefaultStarrySkyBrightnessState();

      } else {
        Serial.print("failed, rc=");
        Serial.print(client.state());
        Serial.println(" try again in 5 seconds");
      }
    }
  }
}

void publishMqttDiscovery() {
  // ... (code is identical, no changes needed here)
  StaticJsonDocument<256> deviceDoc;
  deviceDoc["name"] = HA_DEVICE_NAME;
  JsonArray identifiers = deviceDoc.createNestedArray("identifiers");
  identifiers.add(HA_DEVICE_ID);
  deviceDoc["manufacturer"] = "Roy";
  deviceDoc["model"] = "Wolklamp ESP32-S3";
  Serial.println("Publishing MQTT Discovery Messages");
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

  StaticJsonDocument<1024> doc_mosfet;
  doc_mosfet["name"] = "Wolk Wit";
  doc_mosfet["unique_id"] = String(HA_DEVICE_ID) + "_mosfet";
  doc_mosfet["state_topic"] = TOPIC_MOSFET_STATE;
  doc_mosfet["command_topic"] = TOPIC_MOSFET_COMMAND;
  doc_mosfet["schema"] = "json";
  doc_mosfet["brightness"] = true;
  doc_mosfet["color_mode"] = false;
  doc_mosfet["qos"] = 0;
  doc_mosfet["retain"] = true;
  doc_mosfet["device"] = serialized(deviceJsonString);

  String discovery_topic_mosfet = String(HA_DISCOVERY_PREFIX) + "/light/" + HA_DEVICE_ID + "_mosfet/config";
  String payload_mosfet;
  serializeJson(doc_mosfet, payload_mosfet);
  client.publish(discovery_topic_mosfet.c_str(), payload_mosfet.c_str(), true);

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

  // Default RGBW Brightness Discovery (Number entity)
  StaticJsonDocument<512> doc_default_rgbw_brightness;
  doc_default_rgbw_brightness["name"] = "Default RGBW Helderheid";
  doc_default_rgbw_brightness["unique_id"] = String(HA_DEVICE_ID) + "_default_rgbw_brightness";
  doc_default_rgbw_brightness["command_topic"] = TOPIC_DEFAULT_RGBW_BRIGHTNESS_COMMAND;
  doc_default_rgbw_brightness["state_topic"] = TOPIC_DEFAULT_RGBW_BRIGHTNESS_STATE;
  doc_default_rgbw_brightness["min"] = 0;
  doc_default_rgbw_brightness["max"] = 255;
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

  // Default MOSFET Brightness Discovery (Number entity)
  StaticJsonDocument<512> doc_default_mosfet_brightness;
  doc_default_mosfet_brightness["name"] = "Default Wolk Wit Helderheid";
  doc_default_mosfet_brightness["unique_id"] = String(HA_DEVICE_ID) + "_default_mosfet_brightness";
  doc_default_mosfet_brightness["command_topic"] = TOPIC_DEFAULT_MOSFET_BRIGHTNESS_COMMAND;
  doc_default_mosfet_brightness["state_topic"] = TOPIC_DEFAULT_MOSFET_BRIGHTNESS_STATE;
  doc_default_mosfet_brightness["min"] = 0;
  doc_default_mosfet_brightness["max"] = 255;
  doc_default_mosfet_brightness["step"] = 1;
  doc_default_mosfet_brightness["unit_of_measurement"] = "%";
  doc_default_mosfet_brightness["device"] = serialized(deviceJsonString);
  doc_default_mosfet_brightness["value_template"] = "{{ value_json.value }}";
  doc_default_mosfet_brightness["qos"] = 0;
  doc_default_mosfet_brightness["retain"] = true;

  String discovery_topic_default_mosfet_brightness = String(HA_DISCOVERY_PREFIX) + "/number/" + HA_DEVICE_ID + "_default_mosfet_brightness/config";
  String payload_default_mosfet_brightness;
  serializeJson(doc_default_mosfet_brightness, payload_default_mosfet_brightness);
  client.publish(discovery_topic_default_mosfet_brightness.c_str(), payload_default_mosfet_brightness.c_str(), true);

  // Default Starry Sky Brightness Discovery (Number entity)
  StaticJsonDocument<512> doc_default_starry_sky_brightness;
  doc_default_starry_sky_brightness["name"] = "Default Sterrenhemel Helderheid";
  doc_default_starry_sky_brightness["unique_id"] = String(HA_DEVICE_ID) + "_default_starry_sky_brightness";
  doc_default_starry_sky_brightness["command_topic"] = TOPIC_DEFAULT_STARRY_SKY_BRIGHTNESS_COMMAND;
  doc_default_starry_sky_brightness["state_topic"] = TOPIC_DEFAULT_STARRY_SKY_BRIGHTNESS_STATE;
  doc_default_starry_sky_brightness["min"] = 0;
  doc_default_starry_sky_brightness["max"] = 255;
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
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  TelnetStream.print("Message arrived [");
  TelnetStream.print(topic);
  TelnetStream.print("] ");
  String message_payload = "";
  for (int i = 0; i < length; i++) {
    message_payload += (char)payload[i];
  }
  TelnetStream.println(message_payload);

  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, message_payload);

  if (error) {
    TelnetStream.print(F("deserializeJson() failed: "));
    TelnetStream.println(error.f_str());
    return;
  }

  if (strcmp(topic, TOPIC_RGBW_COMMAND) == 0) {
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
  else if (strcmp(topic, TOPIC_MOSFET_COMMAND) == 0) {
    if (xSemaphoreTake(mosfetStateMutex, portMAX_DELAY) == pdTRUE) {
      if (doc.containsKey("state")) { mosfet_on = (strcmp(doc["state"], "ON") == 0); }
      if (doc.containsKey("brightness")) { targetMosfetBrightness = doc["brightness"].as<uint8_t>(); }
      xSemaphoreGive(mosfetStateMutex);
    }
    sendMosfetState();
  }
  else if (strcmp(topic, TOPIC_STARRY_SKY_COMMAND) == 0) {
    if (xSemaphoreTake(starrySkyStateMutex, portMAX_DELAY) == pdTRUE) {
      if (doc.containsKey("state")) { starry_sky_on = (strcmp(doc["state"], "ON") == 0); }
      if (doc.containsKey("brightness")) { targetStarrySkyBrightness = doc["brightness"].as<uint8_t>(); }
      xSemaphoreGive(starrySkyStateMutex);
    }
    sendStarrySkyState();
  }
  // Default settings do not need mutex as they are only written from MQTT and read in the main loop on state changes
  else if (strcmp(topic, TOPIC_DEFAULT_RGBW_BRIGHTNESS_COMMAND) == 0) {
    if (doc.containsKey("value")) { // Changed from "brightness" to "value" for number entity
      defaultRgbwBrightnessOn230V = doc["value"].as<uint8_t>();
      TelnetStream.printf("Default RGBW brightness on 230V set to: %d\n", defaultRgbwBrightnessOn230V);
      sendDefaultRgbwBrightnessState(); // Send updated state
    }
  }
  else if (strcmp(topic, TOPIC_DEFAULT_RGBW_COLOR_COMMAND) == 0) { if (doc.containsKey("color")) { defaultRgbwColorOn230V.R = doc["color"]["r"].as<uint8_t>(); defaultRgbwColorOn230V.G = doc["color"]["g"].as<uint8_t>(); defaultRgbwColorOn230V.B = doc["color"]["b"].as<uint8_t>(); if (doc["color"].containsKey("w")) { defaultRgbwColorOn230V.W = doc["color"]["w"].as<uint8_t>(); } } }
  else if (strcmp(topic, TOPIC_DEFAULT_MOSFET_BRIGHTNESS_COMMAND) == 0) {
    if (doc.containsKey("value")) { // Changed from "brightness" to "value" for number entity
      defaultMosfetBrightnessOn230V = doc["value"].as<uint8_t>();
      TelnetStream.printf("Default Mosfet brightness on 230V set to: %d\n", defaultMosfetBrightnessOn230V);
      sendDefaultMosfetBrightnessState(); // Send updated state
    }
  }
  else if (strcmp(topic, TOPIC_DEFAULT_STARRY_SKY_BRIGHTNESS_COMMAND) == 0) {
    if (doc.containsKey("value")) { // Changed from "brightness" to "value" for number entity
      defaultStarrySkyBrightnessOn230V = doc["value"].as<uint8_t>();
      TelnetStream.printf("Default Starry Sky brightness on 230V set to: %d\n", defaultStarrySkyBrightnessOn230V);
      sendDefaultStarrySkyBrightnessState(); // Send updated state
    }
  }
}

void handle230VInput() {
  bool reading = digitalRead(PIN_230V_INPUT);
  if (reading != lastButtonState) { lastButtonPressTime = millis(); }
  if ((millis() - lastButtonPressTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;
      if (xSemaphoreTake(rgbwStateMutex, portMAX_DELAY) == pdTRUE &&
          xSemaphoreTake(mosfetStateMutex, portMAX_DELAY) == pdTRUE &&
          xSemaphoreTake(starrySkyStateMutex, portMAX_DELAY) == pdTRUE) {
        
        if (buttonState == HIGH) { // Power off
          is230vAbsent = true;
          input230vAbsentStartTime = millis();
          rgbw_on = false;
          mosfet_on = false;
          starry_sky_on = false;
        } else { // Power on
          is230vAbsent = false;
          bool longPressDetected = (millis() - input230vAbsentStartTime) > 2000;
          if (longPressDetected) {
            rgbw_on = false;
            starry_sky_on = false;
            mosfet_on = true; targetMosfetBrightness = defaultMosfetBrightnessOn230V;
            currentActiveLightSource = MOSFET_STRIP;
          } else {
            switch (currentActiveLightSource) {
              case NONE:
              case STARRY_SKY:
                rgbw_on = false; starry_sky_on = false;
                mosfet_on = true; targetMosfetBrightness = defaultMosfetBrightnessOn230V;
                currentActiveLightSource = MOSFET_STRIP;
                break;
              case MOSFET_STRIP:
                mosfet_on = false; starry_sky_on = false;
                rgbw_on = true; targetRgbwBrightness = defaultRgbwBrightnessOn230V; targetRgbwColor = defaultRgbwColorOn230V;
                currentActiveLightSource = RGBW_STRIP;
                break;
              case RGBW_STRIP:
                mosfet_on = false; rgbw_on = false;
                starry_sky_on = true; targetStarrySkyBrightness = defaultStarrySkyBrightnessOn230V;
                currentActiveLightSource = STARRY_SKY;
                break;
            }
          }
        }
        xSemaphoreGive(starrySkyStateMutex);
        xSemaphoreGive(mosfetStateMutex);
        xSemaphoreGive(rgbwStateMutex);
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
  } else {
    return; // Could not get mutex, skip update
  }

  if (currentRgbwDisplayBrightness == finalTargetBrightness && currentRgbwDisplayColor == finalTargetColor) return;

  // Fading logic...
  if (currentRgbwDisplayBrightness != finalTargetBrightness) {
      if (finalTargetBrightness > currentRgbwDisplayBrightness) {
          currentRgbwDisplayBrightness = min((float)finalTargetBrightness, (float)(currentRgbwDisplayBrightness + (256.0 / (rgbwFadeDuration / 10.0))));
      } else {
          currentRgbwDisplayBrightness = max((float)finalTargetBrightness, (float)(currentRgbwDisplayBrightness - (256.0 / (rgbwFadeDuration / 10.0))));
      }
  }

  // Calculate step for color components
  if (currentRgbwDisplayColor.R != finalTargetColor.R) {
      if (finalTargetColor.R > currentRgbwDisplayColor.R) {
          currentRgbwDisplayColor.R = min((float)finalTargetColor.R, (float)(currentRgbwDisplayColor.R + (256.0 / (rgbwFadeDuration / 10.0))));
      } else {
          currentRgbwDisplayColor.R = max((float)finalTargetColor.R, (float)(currentRgbwDisplayColor.R - (256.0 / (rgbwFadeDuration / 10.0))));
      }
  }
  if (currentRgbwDisplayColor.G != finalTargetColor.G) {
      if (finalTargetColor.G > currentRgbwDisplayColor.G) {
          currentRgbwDisplayColor.G = min((float)finalTargetColor.G, (float)(currentRgbwDisplayColor.G + (256.0 / (rgbwFadeDuration / 10.0))));
      } else {
          currentRgbwDisplayColor.G = max((float)finalTargetColor.G, (float)(currentRgbwDisplayColor.G - (256.0 / (rgbwFadeDuration / 10.0))));
      }
  }
  if (currentRgbwDisplayColor.B != finalTargetColor.B) {
      if (finalTargetColor.B > currentRgbwDisplayColor.B) {
          currentRgbwDisplayColor.B = min((float)finalTargetColor.B, (float)(currentRgbwDisplayColor.B + (256.0 / (rgbwFadeDuration / 10.0))));
      } else {
          currentRgbwDisplayColor.B = max((float)finalTargetColor.B, (float)(currentRgbwDisplayColor.B - (256.0 / (rgbwFadeDuration / 10.0))));
      }
  }
  if (currentRgbwDisplayColor.W != finalTargetColor.W) {
      if (finalTargetColor.W > currentRgbwDisplayColor.W) {
          currentRgbwDisplayColor.W = min((float)finalTargetColor.W, (float)(currentRgbwDisplayColor.W + (256.0 / (rgbwFadeDuration / 10.0))));
      } else {
          currentRgbwDisplayColor.W = max((float)finalTargetColor.W, (float)(currentRgbwDisplayColor.W - (256.0 / (rgbwFadeDuration / 10.0))));
      }
  }

  RgbwColor appliedColor = currentRgbwDisplayColor;
  uint8_t appliedBrightness = currentRgbwDisplayBrightness;

  appliedColor.R = map(appliedColor.R, 0, 255, 0, appliedBrightness);
  appliedColor.G = map(appliedColor.G, 0, 255, 0, appliedBrightness);
  appliedColor.B = map(appliedColor.B, 0, 255, 0, appliedBrightness);
  appliedColor.W = map(appliedColor.W, 0, 255, 0, appliedBrightness);

  // --- Smart Power Limiting ---
  int totalValue = appliedColor.R + appliedColor.G + appliedColor.B + appliedColor.W;
  if (totalValue > MAX_TOTAL_CHANNEL_VALUE) {
    float scale = (float)MAX_TOTAL_CHANNEL_VALUE / (float)totalValue;
    appliedColor.R = (uint8_t)(appliedColor.R * scale);
    appliedColor.G = (uint8_t)(appliedColor.G * scale);
    appliedColor.B = (uint8_t)(appliedColor.B * scale);
    appliedColor.W = (uint8_t)(appliedColor.W * scale);
  }

  // Apply Gamma Correction
  appliedColor.R = gamma8[appliedColor.R];
  appliedColor.G = gamma8[appliedColor.G];
  appliedColor.B = gamma8[appliedColor.B];
  appliedColor.W = gamma8[appliedColor.W];

  for (uint16_t i = 0; i < PixelCount; i++) strip.SetPixelColor(i, appliedColor);
  strip.Show();

  if (currentRgbwDisplayBrightness == finalTargetBrightness &&
      currentRgbwDisplayColor.R == finalTargetColor.R &&
      currentRgbwDisplayColor.G == finalTargetColor.G &&
      currentRgbwDisplayColor.B == finalTargetColor.B &&
      currentRgbwDisplayColor.W == finalTargetColor.W) {
      sendRgbwState();
  }
}

void updateMosfetStripState() {
  unsigned long currentTime = millis();
  if (currentTime - lastMosfetUpdateTime < 10) return;
  lastMosfetUpdateTime = currentTime;
  
  bool on_status;
  uint8_t finalTargetBrightness;

  if (xSemaphoreTake(mosfetStateMutex, (TickType_t)5) == pdTRUE) {
    on_status = mosfet_on;
    finalTargetBrightness = on_status ? targetMosfetBrightness : 0;
    xSemaphoreGive(mosfetStateMutex);
  } else {
    return; // Could not get mutex
  }

  if (currentMosfetDisplayBrightness == finalTargetBrightness) return;

  // Fading logic ... (Identical to original)
  if (finalTargetBrightness > currentMosfetDisplayBrightness) { currentMosfetDisplayBrightness = min((float)finalTargetBrightness, (float)(currentMosfetDisplayBrightness + (256.0 / (mosfetFadeDuration / 10.0)))); }
  else { currentMosfetDisplayBrightness = max((float)finalTargetBrightness, (float)(currentMosfetDisplayBrightness - (256.0 / (mosfetFadeDuration / 10.0)))); }

  // Apply Gamma Correction
  uint8_t correctedBrightness = gamma8[currentMosfetDisplayBrightness];
  uint32_t pwmValue = map(correctedBrightness, 0, 255, 0, (1 << MosfetPwmResolution) - 1);
  ledcWrite(MosfetLedChannel, pwmValue);

  if (currentMosfetDisplayBrightness == finalTargetBrightness) sendMosfetState();
}

void updateStarrySkyState() {
  unsigned long currentTime = millis();
  
  bool on_status;
  uint8_t masterBrightness;
  if (xSemaphoreTake(starrySkyStateMutex, (TickType_t)5) == pdTRUE) {
    on_status = starry_sky_on;
    masterBrightness = targetStarrySkyBrightness;
    xSemaphoreGive(starrySkyStateMutex);
  } else {
    return; // Could not get mutex
  }

  if (masterBrightness == 0 || !on_status) {
    for (int i = 0; i < TLC_NUM_CHIPS * 24; i++) {
      if (stars[i].currentBrightness > 0) { stars[i].currentBrightness = 0; tlc.setPWM(i, 0); }
      stars[i].phase = Star::STAR_OFF;
      stars[i].phaseStartTime = currentTime;
    }
    tlc.write();
    return;
  }
  if (currentTime - lastStarUpdateTime < starUpdateInterval) return;
  lastStarUpdateTime = currentTime;

  // Animation logic (Identical to original)
  for (int i = 0; i < TLC_NUM_CHIPS * 24; i++) {
    Star& star = stars[i];
    // ... (rest of the animation logic is the same)
    switch (star.phase) {
      case Star::STAR_OFF:
        if (currentTime - star.phaseStartTime > star.minActiveDuration) {
          star.phase = Star::STAR_FADE_IN;
          star.fadeStartTime = currentTime;
          star.targetBrightness = star.maxBrightnessPWM;
        }
        break;
      case Star::STAR_FADE_IN: {
        unsigned long elapsed = currentTime - star.fadeStartTime;
        if (star.fadeInDuration > 0) { star.currentBrightness = map(elapsed, 0, star.fadeInDuration, star.minBrightnessPWM, star.maxBrightnessPWM); }
        else { star.currentBrightness = star.maxBrightnessPWM; }
        if (elapsed >= star.fadeInDuration) { star.currentBrightness = star.maxBrightnessPWM; star.phase = Star::STAR_ON; star.phaseStartTime = currentTime; }
        break;
      }
      case Star::STAR_ON:
        if (currentTime - star.phaseStartTime > star.onDuration) {
          star.phase = Star::STAR_FADE_OUT;
          star.fadeStartTime = currentTime;
          star.targetBrightness = star.minBrightnessPWM;
        }
        break;
      case Star::STAR_FADE_OUT: {
        unsigned long elapsed = currentTime - star.fadeStartTime;
        if (star.fadeOutDuration > 0) { star.currentBrightness = map(elapsed, 0, star.fadeOutDuration, star.maxBrightnessPWM, star.minBrightnessPWM); }
        else { star.currentBrightness = star.minBrightnessPWM; }
        if (elapsed >= star.fadeOutDuration) {
          star.currentBrightness = star.minBrightnessPWM;
          star.phase = Star::STAR_OFF;
          star.phaseStartTime = currentTime;
          // Generate new random parameters
          star.minBrightnessPWM = random(0, (int)(0.05 * 4095));
          star.maxBrightnessPWM = random((int)(0.50 * 4095), (int)(1.00 * 4095));
          star.minActiveDuration = random(0, 10) * 1000;
          star.onDuration = random(0, 10) * 1000;
          star.fadeInDuration = random(0, 10) * 1000;
          star.fadeOutDuration = random(0, 10) * 1000;
        }
        break;
      }
    }
    uint16_t finalPWM = map(star.currentBrightness, 0, 4095, 0, map(masterBrightness, 0, 255, 0, 4095));
    
    // Apply Gamma Correction for 12-bit output
    uint16_t correctedPWM = (uint16_t)(pow((float)finalPWM / 4095.0, GAMMA) * 4095.0 + 0.5);
    
    tlc.setPWM(i, correctedPWM);
  }
  tlc.write();
}

void sendRgbwState() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (xSemaphoreTake(rgbwStateMutex, (TickType_t)10) == pdTRUE) {
    StaticJsonDocument<256> doc;
    doc["state"] = rgbw_on ? "ON" : "OFF";
    doc["brightness"] = targetRgbwBrightness;
    JsonObject color = doc.createNestedObject("color");
    color["r"] = targetRgbwColor.R;
    color["g"] = targetRgbwColor.G;
    color["b"] = targetRgbwColor.B;
    color["w"] = targetRgbwColor.W;
    String payload;
    serializeJson(doc, payload);
    client.publish(TOPIC_RGBW_STATE, payload.c_str(), true);
    xSemaphoreGive(rgbwStateMutex);
  }
}

void sendMosfetState() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (xSemaphoreTake(mosfetStateMutex, (TickType_t)10) == pdTRUE) {
    StaticJsonDocument<256> doc;
    doc["state"] = mosfet_on ? "ON" : "OFF";
    doc["brightness"] = targetMosfetBrightness;
    String payload;
    serializeJson(doc, payload);
    client.publish(TOPIC_MOSFET_STATE, payload.c_str(), true);
    xSemaphoreGive(mosfetStateMutex);
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
    client.publish(TOPIC_STARRY_SKY_STATE, payload.c_str(), true);
    xSemaphoreGive(starrySkyStateMutex);
  }
}

void sendDefaultRgbwBrightnessState() {
  if (WiFi.status() != WL_CONNECTED) return;
  StaticJsonDocument<64> doc;
  doc["value"] = defaultRgbwBrightnessOn230V;
  String payload;
  serializeJson(doc, payload);
  client.publish(TOPIC_DEFAULT_RGBW_BRIGHTNESS_STATE, payload.c_str(), true);
}

void sendDefaultMosfetBrightnessState() {
  if (WiFi.status() != WL_CONNECTED) return;
  StaticJsonDocument<64> doc;
  doc["value"] = defaultMosfetBrightnessOn230V;
  String payload;
  serializeJson(doc, payload);
  client.publish(TOPIC_DEFAULT_MOSFET_BRIGHTNESS_STATE, payload.c_str(), true);
}

void sendDefaultStarrySkyBrightnessState() {
  if (WiFi.status() != WL_CONNECTED) return;
  StaticJsonDocument<64> doc;
  doc["value"] = defaultStarrySkyBrightnessOn230V;
  String payload;
  serializeJson(doc, payload);
  client.publish(TOPIC_DEFAULT_STARRY_SKY_BRIGHTNESS_STATE, payload.c_str(), true);
}