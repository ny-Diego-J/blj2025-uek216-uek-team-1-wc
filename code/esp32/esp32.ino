#include <WiFi.h>
#include <ESP32MQTTClient.h>
#include "esp_idf_version.h"
#include <MaxLedControl.h>
#include <cstdint>
#include <cmath>

// compile time constants

#define DEBUG false

#define BAUD_RATE 115200

#define WC_ID "02"
#define WC_PATH "zuerich/wc/"
#define WC_ID_PATH WC_PATH WC_ID

#define OPENED_THRESHOLD_CM 85
#define DISPLAY_BRIGHTNESS 15

// password
#define XOR_MASK 67676767

#define TRIG_PIN 4
#define ECHO_PIN 5
#define DIN_PIN 23
#define CLK_PIN 18
#define CS_PIN 2
#define ALARM_PIN 27

#define PUBLISH_BUF_SIZE 32
// all are in ms
#define DISTANCE_TIMEOUT 300
#define DISPLAY_TIMEOUT 50
#define ALARM_TIMEOUT 100
#define PUBLISH_TIMEOUT 1000
#define ALARM_CYCLE 50
#define ALARM_MAX_DUTY 4
#define DEFAULT_MAX_TIME (15 * 60 * 1000) /* 15 minutes */
// in us
#define PULSE_TIMEOUT 30000

#define SINE_FREQ 1.f
#define SINE_DELTA 0.03f

// in bytes
#define PUBLISH_THREAD_STACK_SIZE 4096
#define DISPLAY_THREAD_STACK_SIZE 4096
#define ALARM_THREAD_STACK_SIZE 2048


// global variables
const char *ssid = "GuestWLANPortal";
const char *server = "mqtt://10.10.2.127:1883";


const char *sub_max_time = WC_ID_PATH "/max_time_ms";
const char *pub_time_remaining = WC_ID_PATH "/time_remaining_ms";
const char *pub_in_use = WC_ID_PATH "/in_use";
 
const char *client_id = "wc" WC_ID;


uint64_t max_time_ms = DEFAULT_MAX_TIME;
int _alarm = false; // alarm is already taken, so use _alarm
uint64_t start_time_ms;
uint64_t countdown_ms; // milliseconds left
float door_distance_cm;
uint8_t in_use = false;

float sine_x = 0.f;

LedControl display = LedControl(DIN_PIN, CLK_PIN, CS_PIN, 1);
ESP32MQTTClient client;

float getWaveformX(){
  float val = sin(SINE_FREQ * sine_x);

  sine_x += SINE_DELTA;

  if (sine_x >= 2.f * PI){
    sine_x -= 2.f * PI;
  }

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
      digitalWrite(ALARM_PIN, HIGH);
      vTaskDelay(1 * portTICK_PERIOD_MS);
      digitalWrite(ALARM_PIN, LOW);
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

      uint32_t duty = (int32_t)((w + 1) * (ALARM_MAX_DUTY - 1));

      playPWM(duty, ALARM_MAX_DUTY, ALARM_CYCLE);
    }
    vTaskDelay(ALARM_TIMEOUT / portTICK_PERIOD_MS);
  }
}


// Xor is its own inverse, so this works for decipher and cipher
uint32_t cipher(void* x){
  // reinterpret cast to uint32 and then xor
  return (*(uint32_t*)x) ^ XOR_MASK;
}

// adds random bytes to a uint8_t boolean
uint32_t packBool8(uint8_t value) {
    uint32_t b = value ? 1 : 0;       // convert bool → 0 or 1
    uint32_t noise = esp_random() & 0x00FFFFFF; // 24 random bits

    return (b << 24) | noise;
}

void mqttPublisher(void* pvParameters){
    (void)pvParameters;

    char msg[PUBLISH_BUF_SIZE];

    while (1){
      // cipher and MQTT Publish
      if (client.isConnected()) {
          snprintf(msg, PUBLISH_BUF_SIZE, "%lu", (unsigned long) cipher(&countdown_ms));
          client.publish(pub_time_remaining, msg);

          uint32_t packed = packBool8(in_use);
          snprintf(msg, PUBLISH_BUF_SIZE, "%lu", (unsigned long) cipher(&packed));
          client.publish(pub_in_use, msg);
      }
      else{
          if constexpr (DEBUG) Serial.println("disconnected :((((");
          setupWifi();
      }
      
      vTaskDelay(PUBLISH_TIMEOUT / portTICK_PERIOD_MS);
    }
}

void countdownDisplayer(void* pvParameters){
  (void)pvParameters;

  char s[PUBLISH_BUF_SIZE];
  while (1){
    uint64_t elapsed_ms = millis() - start_time_ms;

    if (elapsed_ms > max_time_ms) {
      // countdown reached 0
      _alarm = true;
      strncpy(s, "alarm", PUBLISH_BUF_SIZE);
      countdown_ms = 0;
    }
    else {
      _alarm = false;
      countdown_ms = max_time_ms - elapsed_ms;
      uint64_t countdown_s = countdown_ms / 1000;

      uint16_t minutes = countdown_s / 60;
      uint16_t seconds = countdown_s % 60;

      snprintf(s, PUBLISH_BUF_SIZE, "%.2d %.2d", minutes, seconds);
    }

    display.clearDisplay(0);
 
    displayString(s);

    vTaskDelay(DISPLAY_TIMEOUT / portTICK_PERIOD_MS);
  }
}

void displayString(char* s) {
  uint16_t len = (uint16_t)strlen(s);

  for (uint16_t i = 0; i < len; i++) {
    display.setChar(0, 8 - i - 1, s[i], false);
  }
  if constexpr (DEBUG) Serial.println(s);
}
 
float getUltrasonicDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
 
  long duration = pulseIn(ECHO_PIN, HIGH, PULSE_TIMEOUT);
 
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
  Serial.begin(BAUD_RATE);
    
  setupWifi();
  WiFi.setSleep(false);
  client.setURI(server);
  client.setMqttClientName(client_id);
  client.loopStart();

  while (!client.isConnected()) {
    delay(100);
  }

  // publish default max_time
  char msg[PUBLISH_BUF_SIZE];
  snprintf(msg, PUBLISH_BUF_SIZE, "%lu", cipher(&max_time_ms));
  client.publish(sub_max_time, msg);

  pinMode(ALARM_PIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
 
  display.begin(DISPLAY_BRIGHTNESS);
  display.clear();

  start_time_ms = millis();
  countdown_ms = max_time_ms;

  // create publisher thread
  xTaskCreate(
    mqttPublisher,
    "mqttPublisher",
    PUBLISH_THREAD_STACK_SIZE,
    NULL,
    1,
    NULL
  );
  // create displayer thread
  xTaskCreate(
    countdownDisplayer,
    "countdownDisplayer",
    DISPLAY_THREAD_STACK_SIZE,
    NULL,
    1,
    NULL
  );
  xTaskCreate(
    alarmThread,
    "alarmThread",
    ALARM_THREAD_STACK_SIZE,
    NULL,
    1,
    NULL
  );
}
 
void loop() {
  door_distance_cm = getUltrasonicDistanceCm();
 
  if constexpr (DEBUG) Serial.println(door_distance_cm);

  int door_closed = door_distance_cm > OPENED_THRESHOLD_CM;
  // if door is closed, it is in use
  in_use = (uint8_t)door_closed;
 
  if (!in_use) {
    // door is open, so not in use, reset timer
    start_time_ms = millis();
  }

  if constexpr (DEBUG){
    Serial.print("distance: ");
    Serial.print(door_distance_cm);

    int b_as_int = in_use ? 1 : 0;
    Serial.printf("\nin_use = %d\n", b_as_int);
  }

  delay(DISTANCE_TIMEOUT);
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