#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <ESP32Encoder.h>

/*
 * Sender.cpp - ESP-NOW Fixed-Speed Remote Control for Stepper Motor
 * Author: HB9IIU
 *
 * Description:
 * This program reads a rotary encoder to detect the direction of rotation and sends a fixed-speed
 * velocity command to a remote ESP32 via ESP-NOW. The rotation speed is predefined (Slow, Medium, Fast)
 * and selected using a capacitive touch sensor. An automatic stop (velocity = 0) is sent if the encoder
 * is idle for a short time to prevent the motor from continuing to rotate.
 *
 * Features:
 * - Uses rotary encoder to detect rotation direction.
 * - Sends fixed velocity values (+/-) depending on selected mode:
 *      - Slow   → ±4
 *      - Medium → ±12
 *      - Fast   → ±24
 * - Velocity is only sent when:
 *      1. Encoder is moved (sends ±velocity)
 *      2. Encoder becomes idle (sends 0 once)
 * - Capacitive touch sensor (TTP223) cycles through speed modes.
 * - Mode indication via three LEDs.
 * - Status LED blinks fast when connected, slow if not.
 *
 * Hardware Connections:
 * - Encoder A: GPIO32
 * - Encoder B: GPIO33
 * - Touch Sensor: GPIO39
 * - Status LED: GPIO27
 * - Mode LEDs:
 *     - Slow   → GPIO14
 *     - Medium → GPIO12
 *     - Fast   → GPIO13
 *
 * Notes:
 * - Match the Wi-Fi channel with the receiver (default: channel 1).
 * - Use FindMACAddress.cpp to discover receiver MAC if needed.
 */

const uint8_t receiverMAC[] = {0x9C, 0x9C, 0x1F, 0x1E, 0xE0, 0x54};

// Pin assignments
const int encoderPinA = 32;
const int encoderPinB = 33;
const int touchPin    = 39;
const int ledStatus   = 27;
const int ledFine     = 14;
const int ledNormal   = 12;
const int ledFast     = 13;

// Fixed speed values
const int slowSpeed   = 1;
const int mediumSpeed = 12;
const int fastSpeed   = 24;

enum Mode { SLOW, MEDIUM, FAST };
Mode currentMode = SLOW;

typedef struct {
  int velocity;
} Message;

Message data;

ESP32Encoder encoder;
int64_t lastPosition = 0;
int direction = 0;

int lastTouchState = LOW;
unsigned long lastTouchTime = 0;
const unsigned long debounceDelay = 200;
bool connectionOK = false;

unsigned long lastMoveTime = 0;
const unsigned long idleTimeout = 200; // milliseconds
bool sentZero = true;

TaskHandle_t ledTaskHandle;

void onSend(const uint8_t *mac, esp_now_send_status_t status) {
  connectionOK = (status == ESP_NOW_SEND_SUCCESS);
}

void updateLEDs() {
  digitalWrite(ledFine,   currentMode == SLOW   ? HIGH : LOW);
  digitalWrite(ledNormal, currentMode == MEDIUM ? HIGH : LOW);
  digitalWrite(ledFast,   currentMode == FAST   ? HIGH : LOW);
}

void handleTouchSensor() {
  int touchState = digitalRead(touchPin);
  if (touchState == HIGH && lastTouchState == LOW && millis() - lastTouchTime > debounceDelay) {
    lastTouchTime = millis();
    currentMode = static_cast<Mode>((currentMode + 1) % 3);
    Serial.print("Mode changed to: ");
    Serial.println(currentMode == SLOW ? "SLOW" : currentMode == MEDIUM ? "MEDIUM" : "FAST");
    updateLEDs();
  }
  lastTouchState = touchState;
}

void blinkLEDTask(void *param) {
  while (true) {
    digitalWrite(ledStatus, HIGH);
    vTaskDelay(pdMS_TO_TICKS(connectionOK ? 75 : 1000));
    digitalWrite(ledStatus, LOW);
    vTaskDelay(pdMS_TO_TICKS(connectionOK ? 75 : 1000));
  }
}

int getFixedSpeed() {
  switch (currentMode) {
    case SLOW:   return slowSpeed;
    case MEDIUM: return mediumSpeed;
    case FAST:   return fastSpeed;
  }
  return 0;
}

void setup() {
  Serial.begin(115200);

  pinMode(touchPin, INPUT);
  pinMode(ledStatus, OUTPUT);
  pinMode(ledFine, OUTPUT);
  pinMode(ledNormal, OUTPUT);
  pinMode(ledFast, OUTPUT);
  updateLEDs();

  WiFi.mode(WIFI_STA);
  Serial.println(WiFi.macAddress());
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
    return;
  }
  esp_now_register_send_cb(onSend);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  encoder.attachHalfQuad(encoderPinA, encoderPinB);
  encoder.setCount(0);
  lastPosition = 0;

  xTaskCreatePinnedToCore(blinkLEDTask, "BlinkLED", 1000, NULL, 1, &ledTaskHandle, 0);
}

void loop() {
  handleTouchSensor();

  // Detect encoder movement
  int64_t currentPosition = encoder.getCount();
  int delta = currentPosition - lastPosition;

  if (delta != 0) {
    direction = (delta > 0) ? 1 : -1;
    lastPosition = currentPosition;
    lastMoveTime = millis();
    sentZero = false;

    data.velocity = direction * getFixedSpeed();
    esp_now_send(receiverMAC, (uint8_t *)&data, sizeof(data));
    Serial.printf("Encoder moved. Sent velocity: %d\n", data.velocity);
  }

  // If idle, send velocity = 0 once
  if (!sentZero && (millis() - lastMoveTime > idleTimeout)) {
    data.velocity = 0;
    esp_now_send(receiverMAC, (uint8_t *)&data, sizeof(data));
    Serial.println("Encoder idle. Sent velocity: 0");
    sentZero = true;
  }

  delay(10); // Polling loop
}
