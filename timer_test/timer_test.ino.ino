#include <MaxLedControl.h>
#include <cstdint>
#include <cmath>

#define DIN_PIN 23
#define CLK_PIN 18
#define CS_PIN  2
#define TRIG_PIN 4
#define ECHO_PIN 5
constexpr uint64_t TIMER_MS = 15 * 60 * 1000;

uint64_t start_time;

LedControl display = LedControl(DIN_PIN, CLK_PIN, CS_PIN, 1);


void setup() {
    Serial.begin(115200);
    display.begin(15);
    display.clear();
    start_time = millis();
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
}

void displayString(const String& s){
    for (uint16_t i = 0; i < (uint16_t)s.length(); i++) {
        display.setChar(0, 8 - i - 1, s[i], false);
    }
}

float getUltrasonicDistanceCm(){
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 30000);

    float distance_cm = duration * 0.034 / 2; // cm

    if (distance_cm == 0.f || distance_cm > 400.f){
        distance_cm = NAN; //invalid
    }

    return distance_cm;
}

String stringLeftPad(const String& s, const uint16_t len, const char pad){
    uint16_t pad_len = len - (uint16_t)s.length();

    String pad_str;
    for (uint16_t i = 0; i < pad_len; i++){
        pad_str += pad;
    }

    return pad_str + s;
}

void loop() {
    uint64_t elapsed_ms = millis() - start_time;

    String s;
    if (elapsed_ms > TIMER_MS){
   
    }
    else{
        uint64_t countdown_ms = TIMER_MS - elapsed_ms;
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

    if (distance_cm < 10){
        start_time = millis();
    }
}