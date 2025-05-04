/*
 * Receiver.cpp - Stepper Motor Control with ESP-NOW
 * Author: HB9IIU
 *
 * Purpose:
 * This program is designed for remotely controlling a stepper motor using ESP-NOW communication.
 * It receives encoder delta values from a sender unit and moves the motor accordingly.
 * Additionally, it includes visual feedback via an LED and a motor timeout for power saving.
 *
 * Features:
 * - Receives encoder delta values via ESP-NOW.
 * - Moves the stepper motor based on the received values.
 * - Uses a status LED to indicate activity:
 *   - Flickers during motor movement.
 *   - Blinks periodically when idle.
 *   - Lights solid when the motor is enabled.
 * - Disables the motor after a period of inactivity to save power.
 *
 * Connections (ESP32 GPIO):
 * - Step Pin (stepPin): GPIO18 - Connect to the STEP pin on the DRV8825.
 * - Direction Pin (dirPin): GPIO19 - Connect to the DIR pin on the DRV8825.
 * - Enable Pin (enPin): GPIO21 - Connect to the EN pin on the DRV8825.
 * - Status LED (statusLED): GPIO23 - Connect the positive leg of the LED to pin 23 through a resistor (220Ω recommended), and the negative leg to GND.
 *
 * DRV8825 Configuration:
 * - Set the microstepping to 1/32 using the DIP switches:
 *   - M0 = HIGH
 *   - M1 = LOW
 *   - M2 = HIGH
 *   Refer to the DRV8825 datasheet or module documentation for detailed setup.
 *
 * Notes:
 * - The maximum encoder delta (step limit) is set to 2000 to filter out noise or erroneous data.
 * - Motor timeout is set to 2 seconds (idleTimeout), after which the motor is disabled if no activity is detected.
 */

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h> // Required for esp_wifi_set_channel and esp_wifi_set_promiscuous

// Stepper motor pins
const int stepPin = 18; // GPIO18 - STEP pin on DRV8825
const int dirPin = 19;  // GPIO19 - DIR pin on DRV8825
const int enPin = 21;   // GPIO21 - EN pin on DRV8825 (active LOW)

// LED pin
const int statusLED = 23; // GPIO23 - Status LED to indicate states

// Motor parameters
const int delayBetweenSteps = 80;       // Microseconds delay for speed control
const int maxStepLimit = 2000;          // Maximum step delta to filter out noise
const unsigned long idleTimeout = 2000; // 2 seconds of inactivity before disabling motor

typedef struct
{
    int encoderDelta; // Only the delta is received
} Message;

Message data;

unsigned long lastActivityTime = 0; // Last time the motor moved
bool motorEnabled = true;           // Track motor state (enabled/disabled)

// Function to enable or disable the stepper motor
void setMotorEnabled(bool enable)
{
    digitalWrite(enPin, enable ? LOW : HIGH); // LOW = Enabled, HIGH = Disabled
    motorEnabled = enable;

    // Update LED to indicate motor state
    if (enable)
    {
        digitalWrite(statusLED, HIGH); // Solid ON for active state
    }
    else
    {
        // Blink once to indicate motor disabled
        digitalWrite(statusLED, LOW);
        delay(200);
        digitalWrite(statusLED, HIGH);
        delay(200);
        digitalWrite(statusLED, LOW);
    }
}

// Function to move the stepper motor
void moveStepper(int steps)
{
    if (steps == 0)
        return;

    // Ensure motor is enabled
    if (!motorEnabled)
        setMotorEnabled(true);

    // Determine direction
    digitalWrite(dirPin, steps > 0 ? HIGH : LOW);

    // Move the stepper motor with LED flickering
    for (int i = 0; i < abs(steps); i++)
    {
        // Stepper pulse
        digitalWrite(stepPin, HIGH);
        digitalWrite(statusLED, HIGH); // LED ON during the step
        delayMicroseconds(delayBetweenSteps);
        digitalWrite(stepPin, LOW);
        digitalWrite(statusLED, LOW); // LED OFF after the step
        delayMicroseconds(delayBetweenSteps);
    }

    // Update last activity time
    lastActivityTime = millis();
}

void onDataReceived(const uint8_t *mac, const uint8_t *incomingData, int len)
{
    memcpy(&data, incomingData, sizeof(data));
    Serial.printf("Received Encoder Delta: %d\n", data.encoderDelta);

    // Sanitize received delta
    if (abs(data.encoderDelta) > maxStepLimit)
    {
        Serial.println("Warning: Delta exceeds max step limit. Ignored.");
        return;
    }

    // Move the stepper motor
    moveStepper(data.encoderDelta);
}

void setup()
{
    Serial.begin(115200);

    // Initialize stepper motor pins
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(enPin, OUTPUT);
    setMotorEnabled(false); // Start with motor disabled

    // Initialize LED pin
    pinMode(statusLED, OUTPUT);
    digitalWrite(statusLED, LOW); // Start with LED OFF

    // Initialize WiFi in STA mode
    WiFi.mode(WIFI_STA);
    Serial.print("MAC Address: ");
    Serial.println(WiFi.macAddress());

    // Set fixed WiFi channel
    int channel = 1;
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);

    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK)
    {
        Serial.println("ESP-NOW initialization failed!");
        return;
    }
    Serial.println("ESP-NOW initialized successfully.");

    // Register callback for data reception
    esp_now_register_recv_cb(onDataReceived);

    // Print initialization message
    Serial.println("Receiver initialized. Waiting for data...");
    
    // Blink the LED 5 times to indicate the receiver is ready
    for (int i = 0; i < 5; i++)
    {
        digitalWrite(statusLED, HIGH);
        delay(200);
        digitalWrite(statusLED, LOW);
        delay(200);
    }
}

void loop()
{
    // Check if motor should be disabled due to inactivity
    if (motorEnabled && millis() - lastActivityTime > idleTimeout)
    {
        Serial.println("No activity. Disabling motor.");
        setMotorEnabled(false);
    }

    // Blink LED in waiting state
    if (!motorEnabled)
    {
        digitalWrite(statusLED, HIGH);
        delay(500);
        digitalWrite(statusLED, LOW);
        delay(500);
    }
}
