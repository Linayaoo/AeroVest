#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include "MAX30105.h"
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include "heartRate.h"

// DHT sensor setup
#define DHTPIN 4          // DHT11 sensor data pin
#define DHTTYPE DHT11     // DHT sensor type
DHT dht(DHTPIN, DHTTYPE);

// Fan motor control setup
#define MOTOR_PIN_ENA 26   // PWM pin for Fan 1
#define MOTOR_PIN_IN1 14   // Input pin 1 for motor driver (Fan 1)
#define MOTOR_PIN_IN2 12   // Input pin 2 for motor driver (Fan 1)
#define MOTOR_PIN_ENB 25   // PWM pin for Fan 2
#define MOTOR_PIN_IN3 27   // Input pin 1 for motor driver (Fan 2)
#define MOTOR_PIN_IN4 13   // Input pin 2 for motor driver (Fan 2)
#define TEMPERATURE_THRESHOLD_LOW 30.0
#define TEMPERATURE_THRESHOLD_MAX 37.0 // Temperature thresholds to adjust motor speed

// Define PWM channels
const int pwmChannel1 = 0;
const int pwmChannel2 = 1;
const int pwmFreq = 5000;
const int pwmResolution = 8;

// Define pin numbers for LEDs (adjust these pins as per your ESP32 setup)
const int greenLED = 18;
const int redLED = 19;

// Define the pin connected to the button and buzzer
const int buttonPin = 2;
const int buzzerPin1 = 5;
const int emergencyTime = 1000; // Define the hold time for the button in milliseconds (1 second)
bool buzzerOn = false; // Flag to control the state of the buzzer
unsigned long buttonPressTime = 0; // Variable to track the button press time

// Define new button pin for controlling the fan
const int fanButtonPin = 3; // New button pin for controlling the fan

// Define fan states
enum FanState {
  FAN_OFF,
  FAN_LOW,
  FAN_MEDIUM,
  FAN_MAX,
  FAN_AUTONOMOUS
};

FanState fanState = FAN_OFF; // Initialize the fan state to OFF

// MAX30105 sensor setup
MAX30105 particleSensor;

const byte RATE_SIZE = 15; // Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE];    // Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; // Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;

uint8_t broadcastAddress[] = {0xC8, 0xF0, 0x9E, 0xF3, 0xDF, 0x44};

typedef struct struct_message {
  float a;
  float b;
  float c;
} struct_message;

struct_message myData;

esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *macAddr, esp_now_send_status_t status) {
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  // Serial setup
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  
  // DHT sensor setup
  dht.begin();

  // Fan motor pins setup
  pinMode(MOTOR_PIN_IN1, OUTPUT);
  pinMode(MOTOR_PIN_IN2, OUTPUT);
  pinMode(MOTOR_PIN_IN3, OUTPUT);
  pinMode(MOTOR_PIN_IN4, OUTPUT);

  // Configure PWM functionalities
  ledcSetup(pwmChannel1, pwmFreq, pwmResolution);
  ledcAttachPin(MOTOR_PIN_ENA, pwmChannel1);
  ledcSetup(pwmChannel2, pwmFreq, pwmResolution);
  ledcAttachPin(MOTOR_PIN_ENB, pwmChannel2);

  // Heart rate sensor setup

  if (!particleSensor.begin(Wire, 400000)) {
    Serial.println("MAX30105 was not found. Please check wiring/power.");
    while (1);
  }
  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(27); // Manually set the Red LED brightness here
  particleSensor.setPulseAmplitudeGreen(0); // Turn off Green LED

  // Pin setups
  pinMode(greenLED, OUTPUT);
  pinMode(redLED, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP); // Set the button pin as input with internal pull-up resistor
  pinMode(buzzerPin1, OUTPUT);      // Set the buzzer pin as output

  // New button pin setup for controlling the fan
  pinMode(fanButtonPin, INPUT_PULLUP);

  Serial.println("Place your finger on the sensor with steady pressure.");
}

void loop() {
   static unsigned long lastSendTime = 0;
  static unsigned long lastButtonPressTime = 0;
  unsigned long currentTime = millis();

  // Read temperature and humidity
  float humidity = dht.readHumidity();      // Read humidity
  float temperature = dht.readTemperature(); // Read temperature in Celsius

  // Check if any reads failed and exit early (to try again).
  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("Failed to read temperature or humidity from DHT sensor!");
  } else {
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" °C");

    int motorSpeed = 0;

    if (fanState == FAN_AUTONOMOUS) {
      if (temperature >= TEMPERATURE_THRESHOLD_LOW) {
        motorSpeed = map(temperature, TEMPERATURE_THRESHOLD_LOW, TEMPERATURE_THRESHOLD_MAX, 85, 255);
        motorSpeed = constrain(motorSpeed, 85, 255);
      }
    } else {
      switch (fanState) {
        case FAN_LOW:
          motorSpeed = 85;
          break;
        case FAN_MEDIUM:
          motorSpeed = 170;
          break;
        case FAN_MAX:
          motorSpeed = 255;
          break;
        default:
          motorSpeed = 0;
          break;
      }
    }

    // Set fan speed using ledcWrite for PWM
    ledcWrite(pwmChannel1, motorSpeed); // Fan 1
    ledcWrite(pwmChannel2, motorSpeed); // Fan 2

    // Set fan direction
    if (motorSpeed > 0) {
      digitalWrite(MOTOR_PIN_IN1, HIGH);
      digitalWrite(MOTOR_PIN_IN2, LOW);
      digitalWrite(MOTOR_PIN_IN3, HIGH);
      digitalWrite(MOTOR_PIN_IN4, LOW);
    } else {
      digitalWrite(MOTOR_PIN_IN1, LOW);
      digitalWrite(MOTOR_PIN_IN2, LOW);
      digitalWrite(MOTOR_PIN_IN3, LOW);
      digitalWrite(MOTOR_PIN_IN4, LOW);
    }

    if (fanState != FAN_AUTONOMOUS) {
      Serial.print("Motor Speed: ");
      Serial.println(motorSpeed == 255 ? "Max" : (motorSpeed >= 170 ? "Med" : (motorSpeed >= 85 ? "Low" : "Off")));
    }
  }

  // Check if the fan button is pressed to toggle the fan state
  if (digitalRead(fanButtonPin) == LOW && (currentTime - lastButtonPressTime > 200)) {  // Debouncing and cycling states
    lastButtonPressTime = currentTime;

    // Cycle through fan states
    switch (fanState) {
      case FAN_OFF:
        fanState = FAN_LOW;
        Serial.println("Fan set to LOW");
        break;
      case FAN_LOW:
        fanState = FAN_MEDIUM;
        Serial.println("Fan set to MEDIUM");
        break;
      case FAN_MEDIUM:
        fanState = FAN_MAX;
        Serial.println("Fan set to MAX");
        break;
      case FAN_MAX:
        fanState = FAN_AUTONOMOUS;
        Serial.println("Fan set to AUTONOMOUS");
        break;
      case FAN_AUTONOMOUS:
        fanState = FAN_OFF;
        Serial.println("Fan turned OFF");
        break;
    }
  }

  // Heart rate and SpO2 measurement
  long irValue = particleSensor.getIR();

  if (checkForBeat(irValue) == true) {
    // We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    // Ensure beatsPerMinute does not go below 50 BPM
    if (beatsPerMinute < 72) {
      beatsPerMinute = 72; // Cap it at 50 BPM
    }

    if (beatsPerMinute < 255 && beatsPerMinute > 20) {
      rates[rateSpot++] = (byte)beatsPerMinute; // Store this reading in the array
      rateSpot %= RATE_SIZE;                    // Wrap variable

      // Take average of readings
      beatAvg = 0;
      for (byte x = 0; x < RATE_SIZE; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }

  // Display SpO2
  long redValue = particleSensor.getRed();
  float ratio = (float)redValue / irValue;
  float spo2 = 110 - 25 * ratio;
  if (spo2 > 100) spo2 = 100;
  if (spo2 < 0) spo2 = 0;
  
  // LED control based on BPM
  if (irValue > 50000) {
    if (beatsPerMinute >= 1 && beatsPerMinute <= 120) {
      digitalWrite(greenLED, HIGH); // Turn on green LED
      digitalWrite(redLED, LOW);    // Turn off red LED
    } else if (beatsPerMinute > 120) {
      digitalWrite(greenLED, LOW);  // Turn off green LED
      digitalWrite(redLED, HIGH);   // Turn on red LED
    }
  } else {
    // No finger detected, turn off LEDs
    digitalWrite(greenLED, LOW);
    digitalWrite(redLED, LOW);
  }
  
  // Print heart rate and SpO2 data
  Serial.print("IR=");
  Serial.print(irValue);
  Serial.print(", Red=");
  Serial.print(redValue);
  Serial.print(", BPM=");
  Serial.print(beatsPerMinute);
  Serial.print(", Avg BPM=");
  Serial.print(beatAvg);
  Serial.print(", SpO2=");
  Serial.print(spo2, 1); // Output spo2 with one decimal point
  
  if (irValue < 50000) {
    Serial.print(" No finger?");
  }
  
  Serial.println();

  // Check if the fan button is pressed to toggle the fan state
  if (digitalRead(fanButtonPin) == LOW && (currentTime - lastButtonPressTime > 200)) {
    lastButtonPressTime = currentTime;

    // Cycle through fan states
    switch (fanState) {
      case FAN_OFF:
        fanState = FAN_LOW;
        Serial.println("Fan set to LOW");
        break;
      case FAN_LOW:
        fanState = FAN_MEDIUM;
        Serial.println("Fan set to MEDIUM");
        break;
      case FAN_MEDIUM:
        fanState = FAN_MAX;
        Serial.println("Fan set to MAX");
        break;
      case FAN_MAX:
        fanState = FAN_AUTONOMOUS;
        Serial.println("Fan set to AUTONOMOUS");
        break;
      case FAN_AUTONOMOUS:
        fanState = FAN_OFF;
        Serial.println("Fan turned OFF");
        break;
    }
  }

  // Button and buzzer control
  int buttonState = digitalRead(buttonPin); // Read the state of the button

  if (buttonState == LOW) {  // If button is pressed (active low)
    if (buttonPressTime == 0) {
      buttonPressTime = millis(); // Record the time when the button is pressed
    } else if (millis() - buttonPressTime >= emergencyTime) { // If the button is held for 1 second
      buzzerOn = !buzzerOn; // Toggle the state of the buzzer
      if (buzzerOn) {
        // Turn on the buzzer
        digitalWrite(buzzerPin1, HIGH); // Turn on the buzzer
        Serial.println("EMERGENCY!! FIRST AID NEEDED IMMEDIATELY");
        //send EMERGENCY signal
        myData.a = -1;
        myData.b = -1;
        myData.c = -1;
        esp_now_send(broadcastAddress, (uint8_t*) &myData, sizeof(myData)); 
      } else {
        // Turn off the buzzer
        digitalWrite(buzzerPin1, LOW); // Turn off the buzzer
        Serial.println("An emergency response was made");
      }
      buttonPressTime = 0; // Reset the button press time
    }
  } else {
    buttonPressTime = 0; // Reset the button press time if the button is released
  }

  if (currentTime - lastSendTime >= 1000) {
    lastSendTime = currentTime;
    myData.a = temperature;
    myData.b = beatsPerMinute; // Assuming beatAvg holds the average heart ratez
    myData.c = spo2;

    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));

    if (result == ESP_OK) {
      Serial.println("Sent with success");
    } else {
      Serial.println("Error sending the data");
    }
  }
}
