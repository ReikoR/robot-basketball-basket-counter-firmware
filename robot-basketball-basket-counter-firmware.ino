#include <WiFi.h>
#include <ArduinoWebsockets.h>
#include "secrets.h"

using namespace websockets;

typedef struct __attribute__((packed)) Feedback {
  uint8_t color;
  uint16_t battery;
} Feedback;

typedef struct Sensor {
  const uint8_t pin;
  bool state;
} Sensor;

unsigned long reconnectDelay = 1000;
unsigned long lastReconnectTime = 0;

const uint8_t irLedPin = D3;
const uint8_t sensor1Pin = D8;
const uint8_t sensor2Pin = D9;
const uint8_t blueLedPin = D0;
const uint8_t orangeLedPin = D2;
const uint8_t greenLedPin = D1;
const uint8_t basketSelectionPin = D10;
const uint8_t batteryVoltagePin = 6;

Sensor sensor1 = {.pin = sensor1Pin, .state = LOW};
Sensor sensor2 = {.pin = sensor2Pin, .state = LOW};

const uint32_t irLedFrequency = 56000;
const uint8_t irLedTimerBits = 8;
const uint32_t irLedDuty = 128;

unsigned long time_ms = 0;

int sensorSequenceState = 0;
int sensorsState = 0;
int lastSensorsState = 0;
bool ballDetected = false;
bool timerUpdate = false;
bool isMagentaBasket = false;

uint32_t batteryMilliVolts = 0;

Feedback feedback = {.color = 0, .battery = 0};

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

void ARDUINO_ISR_ATTR sensorHandler(void *arg) {
  Sensor* s = static_cast<Sensor*>(arg);
  s->state = digitalRead(s->pin);

  sensorsState = (sensor2.state << 1) | sensor1.state;

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

  lastSensorsState = sensorsState;
}

void setup() {
  Serial.begin(115200);

  pinMode(sensor1Pin, INPUT);
  attachInterruptArg(sensor1Pin, sensorHandler, &sensor1, CHANGE);

  pinMode(sensor2Pin, INPUT);
  attachInterruptArg(sensor2Pin, sensorHandler, &sensor2, CHANGE);

  pinMode(blueLedPin, OUTPUT);
  pinMode(orangeLedPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);
  pinMode(basketSelectionPin, INPUT);
  pinMode(batteryVoltagePin, INPUT);

  // Setup timer and attach timer to a led pin
  ledcAttach(irLedPin, irLedFrequency, irLedTimerBits);

  digitalWrite(blueLedPin, HIGH);

  // write duty to LEDC
  ledcWrite(irLedPin, irLedDuty);

  // Connect to wifi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  // Wait some time to connect to wifi
  for (int i = 0; /*i < 10 && */WiFi.status() != WL_CONNECTED; i++) {
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

  digitalWrite(greenLedPin, sensor1.state);
  digitalWrite(orangeLedPin, sensor2.state);

  if (timerUpdate) {
    timerUpdate = false;
    batteryMilliVolts = analogReadMilliVolts(batteryVoltagePin) * 2;
    feedback.battery = batteryMilliVolts;
    Serial.printf("battery %d\n", batteryMilliVolts);
  }

  if (!client.available() && (time_ms - lastReconnectTime) > reconnectDelay) {
    lastReconnectTime = time_ms;
    Serial.println("Reconnecting websocket");
    client.connect(WEBSOCKETS_SERVER);
  }

  client.poll();
}
