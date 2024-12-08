/*#if (!PLATFORMIO)
  // Enable Arduino-ESP32 logging in Arduino IDE
  #ifdef CORE_DEBUG_LEVEL
    #undef CORE_DEBUG_LEVEL
  #endif
  #ifdef LOG_LOCAL_LEVEL
    #undef LOG_LOCAL_LEVEL
  #endif

  #define CORE_DEBUG_LEVEL 4
  #define LOG_LOCAL_LEVEL CORE_DEBUG_LEVEL
#endif*/

#include <WiFi.h>
#include <ArduinoWebsockets.h>
#include "secrets.h"

// use first channel of 16 channels (started from zero)
#define LEDC_CHANNEL     0

// use 13 bit precission for LEDC timer
#define LEDC_TIMER_BITS  8

// use 56000 Hz as a LEDC base frequency
#define LEDC_FREQ     56000

// fade LED PIN (replace with LED_BUILTIN constant for built-in LED)
#define LED_PIN            D3

using namespace websockets;

typedef struct __attribute__((packed)) Feedback {
  uint8_t color;
  uint16_t battery;
} Feedback;

unsigned long reconnectDelay = 1000;
unsigned long lastReconnectTime = 0;

const uint8_t sensor1Pin = D8;
const uint8_t sensor2Pin = D9;
const uint8_t blueLedPin = D0;
const uint8_t orangeLedPin = D2;
const uint8_t greenLedPin = D1;
const uint8_t basketSelectionPin = D10;
const uint8_t batteryVoltagePin = 6;

unsigned long time_ms = 0;
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 1000;

int sensorSequenceState = 0;
int sensor1State = LOW;
int sensor2State = LOW;
int lastSensor1State = LOW;
int lastSensor2State = LOW;
int sensorsState = 0;
int lastSensorsState = 0;
//int sensorsStateChange = 0;
bool ballDetected = false;
bool timerUpdate = false;
bool isMagentaBasket = false;

uint32_t batteryMilliVolts = 0;

Feedback feedback = {.color = 0, .battery = 0};

// Valid sensors sequence: 00 -> 01 -> 11 -> 10 -> 00 (0 -> 1 -> 3 -> 2 -> 0)
/*
       new
       0  1  2  3
old 0  0  1  0  0
    1  0  0  0  2
    2  4  0  0  2
    3  0  1  3  0
*/

int QEM[16] = {
   0,  1,  0,  0,
   0,  0,  0,  2,
   4,  0,  0,  2,
   0,  1,  3,  0
};


WebsocketsClient client;

hw_timer_t * timer = NULL;

void onTimer(){
  timerUpdate = true;
  digitalWrite(blueLedPin, !digitalRead(blueLedPin));
}

void startTimer() {
  timer = timerBegin(1000000);
  timerAttachInterrupt(timer, &onTimer);
  timerAlarm(timer, 1000000, true, 0);  //1000 ms
}

void endTimer() {
  timerEnd(timer);
  timer = NULL; 
}

void onMessageCallback(WebsocketsMessage message) {
  Serial.print("Got Message: ");
  Serial.println(message.data());
}

void onEventsCallback(WebsocketsEvent event, String data) {
  if (event == WebsocketsEvent::ConnectionOpened) {
    //digitalWrite(orangeLedPin, LOW);
    Serial.println("Connnection Opened");
  } else if (event == WebsocketsEvent::ConnectionClosed) {
    //digitalWrite(orangeLedPin, HIGH);
    Serial.println("Connnection Closed");
  } else if (event == WebsocketsEvent::GotPing) {
    Serial.println("Got a Ping!");
  } else if (event == WebsocketsEvent::GotPong) {
    Serial.println("Got a Pong!");
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(sensor1Pin, INPUT);
  pinMode(sensor2Pin, INPUT);
  pinMode(blueLedPin, OUTPUT);
  pinMode(orangeLedPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);
  pinMode(basketSelectionPin, INPUT);
  pinMode(batteryVoltagePin, INPUT);

  // Setup timer and attach timer to a led pin
  ledcAttach(LED_PIN, LEDC_FREQ, LEDC_TIMER_BITS);

  digitalWrite(blueLedPin, HIGH);

  uint32_t duty = 128;

  // write duty to LEDC
  ledcWrite(LED_PIN, duty);

  // Connect to wifi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  // Wait some time to connect to wifi
  for (int i = 0; i < 10 && WiFi.status() != WL_CONNECTED; i++) {
    Serial.print(".");
    delay(1000);
  }

  digitalWrite(blueLedPin, LOW);

  startTimer();

  Serial.println("\nConnected to WiFi");

  // Setup Callbacks
  client.onMessage(onMessageCallback);
  client.onEvent(onEventsCallback);

  // Connect to server
  client.connect(WEBSOCKETS_SERVER);

  // Send a ping
  client.ping();

  isMagentaBasket = digitalRead(basketSelectionPin) == HIGH;
  feedback.color = isMagentaBasket;
}

void loop() {
  time_ms = millis();

  sensor1State = digitalRead(sensor1Pin);
  sensor2State = digitalRead(sensor2Pin);
  sensorsState = (sensor2State << 1) | sensor1State;

  /*if (sensorsState != lastSensorsState) {
    lastDebounceTime = time_ms;

    sensorSequenceState = QEM[(lastSensorsState) << 2 | sensorsState];

    if (sensorSequenceState == 4) {
      sensorSequenceState = 0;
      ballDetected = true;
    }

    Serial.printf("state %d %d\n", sensorSequenceState, sensorsState);


  }*/


  if (sensorsState != lastSensorsState) {
    switch (sensorSequenceState) {
      case 0:
        if (sensorsState == 1) {
          sensorSequenceState = 1;
        } else {
          sensorSequenceState = 0;
        }
        break;
      case 1:
        if (sensorsState == 3) {
          sensorSequenceState = 2;
        } else {
          sensorSequenceState = 0;
        }
        break;
      case 2:
        if (sensorsState == 2) {
          sensorSequenceState = 3;
        } else if (sensorsState == 1) {
          sensorSequenceState = 1;
        } else {
          sensorSequenceState = 0;
        }
        break;
      case 3:
        if (sensorsState == 0) {
          ballDetected = true;
          sensorSequenceState = 0;
        } else if (sensorsState == 3) {
          sensorSequenceState = 2;
        } else {
          sensorSequenceState = 0;
        }
        
        break;
    }
  }

  if (sensor1State == HIGH) {
    digitalWrite(greenLedPin, HIGH);
  } else {
    digitalWrite(greenLedPin, LOW);
  }

  if (sensor2State == HIGH) {
    digitalWrite(orangeLedPin, HIGH);
  } else {
    digitalWrite(orangeLedPin, LOW);
  }

  /*if (sensorSequenceState != 0 &&  !(time_ms - lastDebounceTime) > debounceDelay) {
    sensorSequenceState = 0;
    Serial.println("clear state");
  }*/

  lastSensor1State = sensor1State;
  lastSensor2State = sensor2State;
  lastSensorsState = sensorsState;

  if (timerUpdate) {
    timerUpdate = false;
    batteryMilliVolts = analogReadMilliVolts(batteryVoltagePin) * 2;
    feedback.battery = batteryMilliVolts;
    Serial.printf("battery %d\n", batteryMilliVolts);
  }

  if (ballDetected) {
    ballDetected = false;
    Serial.println("ball");

    /*if (isMagentaBasket) {
      client.send("magenta");
    } else {
      client.send("blue");
    }*/

    client.sendBinary((char *)&feedback, sizeof feedback);
  }

  if (!client.available() && (time_ms - lastReconnectTime) > reconnectDelay) {
    lastReconnectTime = time_ms;
    Serial.println("Reconnecting websocket");
    client.connect(WEBSOCKETS_SERVER);
  }

  client.poll();
}
