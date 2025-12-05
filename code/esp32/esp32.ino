#include <WiFi.h>
#include <ESP32MQTTClient.h>
#include "esp_idf_version.h"
#include <MaxLedControl.h>
#include <cstdint>
#include <cmath>

#define WC_ID "02"
#define WC_PATH "zuerich/wc/"
#define WC_ID_PATH WC_PATH WC_ID

#define TRIG_PIN 4
#define ECHO_PIN 5
#define DIN_PIN 23
#define CLK_PIN 18
#define CS_PIN 2
#define OPENED_THRESHOLD_CM 85

#define DEBUG false

const char *ssid = "GuestWLANPortal";
const char *server = "mqtt://10.10.2.127:1883";


const char *sub_max_time = WC_ID_PATH "/max_time_ms";
const char *pub_time_remaining = WC_ID_PATH "/time_remaining_ms";
const char *pub_in_use = WC_ID_PATH "/in_use";
 
const char *client_id = "wc" WC_ID;


uint64_t max_time_ms = 15 * 60 * 1000; // default 15 minutes
int _alarm = false; // alarm is already taken, so use _alarm
uint64_t start_time_ms;
uint64_t countdown_ms; // milliseconds left
float door_distance_cm;
uint8_t in_use = false;

// sine wave
float sin_freq = 1.f;
float delta = 0.3f;
float t = 0.f;

// password
uint32_t xor_mask = 67676767;

LedControl display = LedControl(DIN_PIN, CLK_PIN, CS_PIN, 1);
ESP32MQTTClient client;

float getWaveformX(){
  float val = sin(sin_freq * t);

  t += delta;

  t = fmod(t, 2.f * PI);

  return val / (PI / 2.f);
}

// plays a PWM signal on the alarm
void playPWM(uint16_t duty, uint16_t cycle, uint16_t time_ms){
  uint64_t start = millis();
  uint64_t end = start + time_ms;

  while (millis() < end){
    uint64_t start_duty = millis();
    uint64_t end_duty = start_duty + duty;
    while (millis() < end_duty){
      digitalWrite(27, HIGH);
      vTaskDelay(1 * portTICK_PERIOD_MS);
      digitalWrite(27, LOW);
      vTaskDelay(1 * portTICK_PERIOD_MS);
    }
    vTaskDelay((1 + cycle - duty) * portTICK_PERIOD_MS);
  }
}

void alarmThread(void* pvParameters){
  (void)(pvParameters);

  while (true){
    while (_alarm){
      float w = getWaveformX();

      uint32_t duty = (int32_t)((w + 1) * 3);

      playPWM(duty, 4, 50);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}


// Xor is its own inverse, so this works for decipher and cipher
uint32_t cipher(void* x){
  return (*(uint32_t*)x) ^ xor_mask;
}

uint32_t packBool8(uint8_t value) {
    uint32_t b = value ? 1 : 0;       // convert bool → 0 or 1
    uint32_t noise = esp_random() & 0x00FFFFFF; // 24 random bits

    return (b << 24) | noise;
}

void mqttPublisher(void* pvParameters){
    (void)pvParameters;

    char msg[32];

    while (1){
      // cipher and MQTT Publish
      if (client.isConnected()) {
          //Serial.printf("raw %f, xored %lu", door_distance_cm, (unsigned long)encoded);
          //snprintf(msg, 32, "%lu", (unsigned long) cipher(&door_distance_cm));
          //client.publish(pub_door_distance, msg);

          snprintf(msg, 32, "%lu", (unsigned long) cipher(&countdown_ms));
          client.publish(pub_time_remaining, msg);

          uint32_t packed = packBool8(in_use);
          snprintf(msg, 32, "%lu", (unsigned long) cipher(&packed));
          client.publish(pub_in_use, msg);
      }
      else{
          if constexpr (DEBUG) Serial.println("disconnected :((((");
          setupWifi();
      }
      
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void countdownDisplayer(void* pvParameters){
  (void)pvParameters;

  char s[32];
  while (1){
    uint64_t elapsed_ms = millis() - start_time_ms;

    if (elapsed_ms > max_time_ms) {
      // countdown reached 0
      _alarm = true;
      strncpy(s, "alarm", 32);
      countdown_ms = 0;
    }
    else {
      _alarm = false;
      countdown_ms = max_time_ms - elapsed_ms;
      uint64_t countdown_s = countdown_ms / 1000;

      uint16_t minutes = countdown_s / 60;
      uint16_t seconds = countdown_s % 60;

      snprintf(s, 32, "%.2d %.2d", minutes, seconds);
    }

    display.clearDisplay(0);
 
    displayString(s);

    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void displayString(char* s) {
  uint16_t len = (uint16_t)strlen(s);

  for (uint16_t i = 0; i < len; i++) {
    display.setChar(0, 8 - i - 1, s[i], false);
  }
  Serial.println(s);
}
 
float getUltrasonicDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
 
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
 
  float cm = duration * 0.034 / 2;  // cm

  // 0 means no signal received
  if (cm == 0.f || cm > 500.f){
    cm = 500.f;
  }
 
  return cm;
}

void setupWifi() {
  if constexpr (DEBUG){
    Serial.print("Connecting to ");
    Serial.print(ssid);
  }
  WiFi.begin(ssid);
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    if constexpr (DEBUG) Serial.print(".");
  }
  
  if constexpr (DEBUG) Serial.println("done!");
}

void setup() {
  Serial.begin(115200);
    
  setupWifi();
  WiFi.setSleep(false);
  client.setURI(server);
  client.setMqttClientName(client_id);
  client.loopStart();
  while (!client.isConnected()) {
    delay(100);
  }

  // publish default max_time
  char msg[32];
  snprintf(msg, 32, "%lu", cipher(&max_time_ms));
  client.publish(sub_max_time, msg);

  pinMode(27, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
 
  display.begin(15);
  display.clear();

  start_time_ms = millis();
  countdown_ms = max_time_ms;

  // create publisher thread
  xTaskCreate(
    mqttPublisher,
    "mqttPublisher",
    4096,
    NULL,
    1,
    NULL
  );
  // create displayer thread
  xTaskCreate(
    countdownDisplayer,
    "countdownDisplayer",
    4096,
    NULL,
    1,
    NULL
  );
  xTaskCreate(
    alarmThread,
    "alarmThread",
    2048,
    NULL,
    1,
    NULL
  );
}
 
void loop() {
  door_distance_cm = getUltrasonicDistanceCm();
 
  if constexpr (DEBUG) Serial.println(door_distance_cm);


  // if door is closed, it is used
  in_use = (uint8_t)(door_distance_cm > OPENED_THRESHOLD_CM);
 
  if (!in_use) {
    // door is open, so not in use, reset timer
    start_time_ms = millis();
  }
 
  //Serial.print("distance: ");
  //Serial.print(door_distance_cm);

  //int b_as_int = in_use ? 1 : 0;
  //Serial.printf("\nbool = %d\n", b_as_int);

  delay(300);
}
 
void onMqttConnect(esp_mqtt_client_handle_t client_handle) {
  client.subscribe(std::string(sub_max_time), [](const std::string &payload) {
    uint32_t ciphered = (uint32_t) std::stoull(payload.c_str(), NULL, 10);
    max_time_ms = (uint64_t) cipher(&ciphered);
    if constexpr (DEBUG){
      Serial.println("hhhhhhhhh");
      Serial.println(ciphered);
      Serial.println(max_time_ms);
    }
  });
}
 
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
esp_err_t handleMQTT(esp_mqtt_event_handle_t event) {
  client.onEventCallback(event);
  return ESP_OK;
}
#else
void handleMQTT(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
  auto *event = static_cast<esp_mqtt_event_handle_t>(event_data);
  client.onEventCallback(event);
}
#endif