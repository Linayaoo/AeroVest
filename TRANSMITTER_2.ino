#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <Adafruit_Sensor.h>
#include <DHT.h>

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

// Heart rate sensor setup
MAX30105 particleSensor;

const byte RATE_SIZE = 4; // Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; // Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; // Time at which the last beat occurred
float beatsPerMinute;
int beatAvg;

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
bool fanOn = false; // Flag to control the state of the fan

uint8_t broadcastAddress[] = {0xC8, 0xF0, 0x9E, 0xF1, 0xB2, 0xA4};

typedef struct struct_message {
  float a;
  float b;
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
  pinMode(greenLED, OUTPUT);
  pinMode(redLED, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP); // Set the button pin as input with internal pull-up resistor
  pinMode(buzzerPin1, OUTPUT);      // Set the buzzer pin as output

  // New button pin setup for controlling the fan
  pinMode(fanButtonPin, INPUT_PULLUP);

  // Initialize heart rate sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30105 was not found. Please check wiring/power.");
    while (1);
  }

  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x0A);
  particleSensor.setPulseAmplitudeGreen(0);
}

void loop() {
  static unsigned long lastPrintTime = 0;
  static unsigned long lastSendTime = 0;
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

    if (temperature >= TEMPERATURE_THRESHOLD_LOW && fanOn) {
      // Adjust the mapping range as needed for your application
      motorSpeed = map(temperature, TEMPERATURE_THRESHOLD_LOW, TEMPERATURE_THRESHOLD_MAX, 85, 255); // Example range: 30°C to 37°C

      // Ensure the PWM value is within bounds
      motorSpeed = constrain(motorSpeed, 85, 255);
    }

    // Set fan speed using ledcWrite for PWM only if the fan is ON
    if (fanOn) {
      ledcWrite(pwmChannel1, motorSpeed); // Fan 1
      ledcWrite(pwmChannel2, motorSpeed); // Fan 2

      // Set fan direction
      digitalWrite(MOTOR_PIN_IN1, HIGH);
      digitalWrite(MOTOR_PIN_IN2, LOW);
      digitalWrite(MOTOR_PIN_IN3, HIGH);
      digitalWrite(MOTOR_PIN_IN4, LOW);

      Serial.print("Motor Speed: ");
      Serial.println(motorSpeed == 255 ? "Max" : (motorSpeed >= 170 ? "Med" : "Low"));
    } else {
      // Turn off the fan if it's not supposed to be running
      ledcWrite(pwmChannel1, 0);
      ledcWrite(pwmChannel2, 0);

      // Ensure the motor pins are in a known state
      digitalWrite(MOTOR_PIN_IN1, LOW);
      digitalWrite(MOTOR_PIN_IN2, LOW);
      digitalWrite(MOTOR_PIN_IN3, LOW);
      digitalWrite(MOTOR_PIN_IN4, LOW);
    }
  }

  // Check if the fan button is pressed to toggle the fan state
  if (digitalRead(fanButtonPin) == LOW) {  // If button is pressed (active low)
    delay(50);  // Debouncing delay
    if (digitalRead(fanButtonPin) == LOW) {  // Check button state again
      fanOn = !fanOn; // Toggle the fan state

      if (fanOn) {
        Serial.println("Fan turned ON");
      } else {
        Serial.println("Fan turned OFF");
      }
    }
  }

  if (currentTime - lastSendTime >= 1000) {
    lastSendTime = currentTime;
    myData.a = temperature;
    myData.b = beatAvg;

    esp_now_send(broadcastAddress, (uint8_t*)&myData, sizeof(myData));
  }

  // Heart rate sensor
  long irValue = particleSensor.getIR();

  if (irValue > 50000) {
    if (checkForBeat(irValue) == true) {
      long delta = millis() - lastBeat;
      lastBeat = millis();
      beatsPerMinute = 60 / (delta / 1000.0);

      if (beatsPerMinute < 255 && beatsPerMinute > 20) {
        rates[rateSpot++] = (byte)beatsPerMinute;
        rateSpot %= RATE_SIZE;
        beatAvg = 0;
        for (byte x = 0; x < RATE_SIZE; x++) {
          beatAvg += rates[x];
        }
        beatAvg /= RATE_SIZE;
      }

      digitalWrite(greenLED, LOW);
      digitalWrite(redLED, LOW);

      if (beatsPerMinute >= 1 && beatsPerMinute <= 100) {
        digitalWrite(greenLED, HIGH);
      } else if (beatsPerMinute > 120) {
        digitalWrite(redLED, HIGH);
      }
    }
  } else {
    digitalWrite(greenLED, LOW);
    digitalWrite(redLED, LOW);
  }

  if (currentTime - lastPrintTime >= 100) {
    lastPrintTime = currentTime;
    Serial.print("IR=");
    Serial.print(irValue);
    Serial.print(", BPM=");
    Serial.print(beatsPerMinute);
    Serial.print(", Avg BPM=");
    Serial.print(beatAvg);

    if (irValue < 50000) {
      Serial.print(" No finger?");
    }

    Serial.println();
  }

// Button and buzzer control
  int buttonState = digitalRead(buttonPin);
  if (buttonState == LOW) {
    if (buttonPressTime == 0) {
      buttonPressTime = millis();
    } else if (millis() - buttonPressTime >= emergencyTime) {
      buzzerOn = !buzzerOn;
      if (buzzerOn) {
        digitalWrite(buzzerPin1, HIGH);
        Serial.println("EMERGENCY!! FIRST AID NEEDED IMMEDIATELY");

        // Send emergency signal via ESP-NOW
        myData.a = -1; // Set a special value for emergency
        myData.b = -1; // Set a special value for emergency
        esp_now_send(broadcastAddress, (uint8_t*)&myData, sizeof(myData));
      } else {
        digitalWrite(buzzerPin1, LOW);
        Serial.println("An emergency response was made");
      }
      buttonPressTime = 0;
    }
  } else {
    buttonPressTime = 0;
  }
}