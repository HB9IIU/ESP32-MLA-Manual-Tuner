#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

/*
 * Receiver.cpp - Velocity-Controlled Stepper Motor with ESP-NOW and Status LED
 * Author: HB9IIU
 *
 * Purpose:
 * This program receives fixed signed velocity values from a remote ESP32 via ESP-NOW
 * and controls a stepper motor accordingly. It includes motor timeout logic to disable
 * the driver after inactivity and provides visual feedback using a status LED.
 *
 * Features:
 * - Receives signed velocity (steps per unit time) from the sender.
 * - Drives the stepper motor continuously while velocity ≠ 0.
 * - Disables the motor after 2 seconds of no movement (velocity = 0).
 * - Status LED behavior:
 *     - Flickers during stepping.
 *     - Blinks slowly when idle.
 *     - Lights solid when the motor is enabled but not stepping.
 *
 * Hardware Connections:
 * - Step Pin (stepPin): GPIO18 → TMC2208 STEP
 * - Direction Pin (dirPin): GPIO19 → TMC2208 DIR
 * - Enable Pin (enPin): GPIO21 → TMC2208 EN (active LOW)
 * - Status LED: GPIO23 (with 220Ω resistor to GND)
 *
 * Notes:
 * - Match the Wi-Fi channel with the sender (default: channel 1).
 * - Maximum allowed velocity is clamped for safety.
 */

const int stepPin    = 18;
const int dirPin     = 19;
const int enPin      = 21;
const int statusLED  = 23;

volatile int currentVelocity = 0;
unsigned long lastStepTime   = 0;
unsigned long lastActivityTime = 0;

const int minDelay    = 600;    // Fastest speed (µs between steps)
const int maxDelay    = 15000;  // Slowest speed (µs between steps)
const int maxVelocity = 50;     // Clamp max abs velocity
const int deadband    = 0;      // Threshold to treat as zero
const unsigned long idleTimeout = 2000; // ms before disabling motor

bool motorEnabled = false;
bool ledState = false;

void setMotorEnabled(bool enable) {
  digitalWrite(enPin, enable ? LOW : HIGH);
  motorEnabled = enable;

  if (enable) {
    digitalWrite(statusLED, HIGH); // LED solid while enabled
  } else {
    // Blink LED once to indicate disable
    digitalWrite(statusLED, LOW);
    delay(200);
    digitalWrite(statusLED, HIGH);
    delay(200);
    digitalWrite(statusLED, LOW);
  }
}

void IRAM_ATTR moveOneStep(bool direction) {
  digitalWrite(dirPin, direction ? HIGH : LOW);
  digitalWrite(stepPin, HIGH);
  digitalWrite(statusLED, HIGH); // Flicker ON
  delayMicroseconds(2);
  digitalWrite(stepPin, LOW);
  digitalWrite(statusLED, LOW);  // Flicker OFF
}

void onDataReceived(const uint8_t *mac, const uint8_t *incomingData, int len) {
  int v = 0;
  if (len >= sizeof(int)) {
    memcpy(&v, incomingData, sizeof(int));
    if (abs(v) <= deadband) {
      currentVelocity = 0;
    } else {
      currentVelocity = constrain(v, -maxVelocity, maxVelocity);
    }
    Serial.printf("Received velocity: %d\n", currentVelocity);
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enPin, OUTPUT);
  pinMode(statusLED, OUTPUT);
  digitalWrite(statusLED, LOW);

  setMotorEnabled(false); // Start with motor off

  WiFi.mode(WIFI_STA);
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());

  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE); // Match sender
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
    return;
  }

  esp_now_register_recv_cb(onDataReceived);
  Serial.println("Receiver ready.");

  // Blink LED to show ready
  for (int i = 0; i < 5; i++) {
    digitalWrite(statusLED, HIGH);
    delay(200);
    digitalWrite(statusLED, LOW);
    delay(200);
  }
}

void loop() {
  int velocity = currentVelocity;

  if (velocity != 0) {
    if (!motorEnabled) setMotorEnabled(true);

    unsigned long now = micros();
    int speed = abs(velocity);
    int stepDelay = constrain(10000 / speed, minDelay, maxDelay);

    if (now - lastStepTime >= (unsigned long)stepDelay) {
      lastStepTime = now;
      moveOneStep(velocity > 0);
      lastActivityTime = millis();
    }
  } else {
    if (motorEnabled && millis() - lastActivityTime > idleTimeout) {
      Serial.println("No activity. Disabling motor.");
      setMotorEnabled(false);
    }

    // Slow blink while idle
    if (!motorEnabled) {
      digitalWrite(statusLED, ledState);
      ledState = !ledState;
      delay(500);
    } else {
      // Solid ON when enabled but not moving
      digitalWrite(statusLED, HIGH);
      delay(10);
    }
  }
}
