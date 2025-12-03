#include <WiFi.h>
#include <ESP32MQTTClient.h>
#include "esp_idf_version.h"
#include <MaxLedControl.h>
#include <cstdint>

const char *ssid = "GuestWLANPortal";
const char *server = "mqtt://10.10.2.127:1883";

const char *sub_topic = "zuerich/wc/01/max_time";        // empfängt
const char *pub_topic = "zuerich/wc/01/info";            // sendet
const char *pub_topic2 = "zuerich/wc/01/time_remining";  // sendet

const char *client_id = "lasarpointer";

ESP32MQTTClient client;

bool laserHit = false;
#define TRIG_PIN 4
#define ECHO_PIN 5
#define DIN_PIN 23
#define CLK_PIN 18
#define CS_PIN 2
uint64_t timer_ms = 1 * 60 * 1000 + 1000;

uint64_t start_time;

LedControl display = LedControl(DIN_PIN, CLK_PIN, CS_PIN, 1);


void setup() {
  Serial.begin(115200);

  setup_wifi();
  WiFi.setSleep(false);

  pinMode(27, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);


  client.setURI(server);
  client.setMqttClientName(client_id);
  client.loopStart();
  display.begin(15);
  display.clear();
  start_time = millis();
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}
void displayString(const String &s) {
  for (uint16_t i = 0; i < (uint16_t)s.length(); i++) {
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

  float distance_cm = duration * 0.034 / 2;  // cm

  return distance_cm;
}

String stringLeftPad(const String &s, const uint16_t len, const char pad) {
  uint16_t pad_len = len - (uint16_t)s.length();

  String pad_str;
  for (uint16_t i = 0; i < pad_len; i++) {
    pad_str += pad;
  }

  return pad_str + s;
}

void loop() {
  uint64_t elapsed_ms = millis() - start_time;
  uint64_t countdown_ms = 0;
  String s;
  if (elapsed_ms > timer_ms) {
    digitalWrite(27, HIGH);
    Serial.print("alarm");
    s = "alarm";
  } else {
    digitalWrite(27, LOW);
    countdown_ms = timer_ms - elapsed_ms;
    uint64_t countdown_s = countdown_ms / 1000;

    uint16_t minutes = countdown_s / 60;
    uint16_t seconds = countdown_s % 60;

    s += stringLeftPad(String(minutes), 2, '0');
    s += ' ';
    s += stringLeftPad(String(seconds), 2, '0');
  }

  Serial.println(s);



  displayString(s);

  float distance_cm = getUltrasonicDistanceCm();

  Serial.println(distance_cm);

  if (distance_cm < 10) {
    start_time = millis();
  }
  // Trigger-Puls
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Echo messen
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);

  // Distanz berechnen (falls der Sensor Ultraschall nutzt)
  float distance = duration * 0.034 / 2;  // cm

  Serial.print("Distanz: ");
  Serial.print(distance);
  Serial.println(" cm");

  // MQTT Publish
  if (client.isConnected()) {
    String msg = String(distance_cm);
    client.publish(pub_topic, msg.c_str());
    client.publish(pub_topic2, String(countdown_ms).c_str());
  }



  delay(100);
}
void setup_wifi() {
  Serial.print("Connecting to ");
  Serial.print(ssid);
  WiFi.begin(ssid);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);


    Serial.print(".");
  }
  Serial.println("done!");
}

void onMqttConnect(esp_mqtt_client_handle_t client_handle) {
  client.subscribe(*sub_topic, [](const std::string &payload) {
    int timer_ms = String(payload.c_str()).toInt();
    Serial.println(timer_ms);
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
