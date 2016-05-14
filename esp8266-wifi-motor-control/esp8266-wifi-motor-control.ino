#include <math.h>

#include <ESP8266WiFi.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <Ticker.h>
#include <Wire.h>

#include "HMC5883L.h"
#include "I2Cdev.h"



extern "C" {
#include "user_interface.h"
}

#define _min(a,b) ((a)<(b)?(a):(b))
#define _max(a,b) ((a)>(b)?(a):(b))

#define JSON_READ_BUFFER_SIZE  450
#define JSON_PARSE_BUFFER_SIZE 360

/* Compass */
#define NODE_SDA_PIN       D2
#define NODE_SCL_PIN       D3
// class default I2C address is 0x1E
// specific I2C addresses may be passed as a parameter here
// this device only supports one I2C address (0x1E)
HMC5883L mag;
int16_t mx, my, mz;

/* Interrupt */

int tickerInterval = 40;
Ticker ticker;
void tickerCallback(void);

/* WIFI */
//const char* ssid     = "UCInet Mobile Access";
//const char* ssid     = "ResNet Mobile Access";
//const char* password = "";

//const char* ssid     = "belkin.2.4";
//const char* password = "beallcenter24";

const char* ssid     = "PETEroiFi";
const char* password = "00000000";

const char* host = "0693c115.ngrok.io";
//const char* host = "dhcp-10-8-019-072.mobile.reshsg.uci.edu";
//const char* path = "/";
const char* path = "/~PETEroid/json/output.json";
//const char* streamId   = "....................";
//const char* privateKey = "....................";

const char* robotName = "red";
const char* enemyName = "green";
const char* friendName = "yellow";

byte mac[6];
bool isReceived = false;
bool wifiTriggered = false;
long wifiConnectTime, wifiRequestTime, wifiWaitTime;

#define WIFI_UPLOAD_DATA_PIN D1

/* SPI Port Expander */
#define PORT_LATCH_PIN D4
#define PORT_DATA_PIN  D7
#define PORT_CLOCK_PIN D5

/* Movement */
#define MOTOR_A_EN  D8 // pwm pin
#define MOTOR_A_IN1 0  // shift register bit
#define MOTOR_A_IN2 1
#define MOTOR_B_EN  D6
#define MOTOR_B_IN1 2
#define MOTOR_B_IN2 3

unsigned char motorOutput;

typedef struct Vec2 {
  float x;
  float y;
} Vec2;

Vec2 velocity, location, target, acceleration, enemyLocation, friendLocation;
Vec2 rotationFromTo = {0, 0};

float mass = 1.0;
float rotation = 0;
float uploadRotation = 0;
float lastRotation = 0;
const int maxMotorOutput = 600;
int minMotorOutput = 128;
int unitMotorOutput = 128;
const int rotateMotorOutput = maxMotorOutput;
const int moveMotorOutput = maxMotorOutput;
int quadrant;
int turnDir;
int lastTurnDir = 1;

const int rotationTimeout = 2000;
const int rotationTrial = 3;

bool isInScreen, isEnemyInScreen, isFriendInScreen;
bool isOddTrial = true;
bool isXBound, isYBound;
const int northAngle = 229;
const int eastAngle = (northAngle + 90) % 360;
const int southAngle = (eastAngle + 90) % 360;
const int westAngle = (southAngle + 90) % 360;

int motorAOutput = 0;
int motorBOutput = 0;

const int boundX = 240;
const int boundY = 180;

int maxSpeedMagnitude = ceil(maxMotorOutput / unitMotorOutput);
int maxForceMagnitude = maxSpeedMagnitude;

bool isRotating = false;
bool isForward = false;

void resetMotor();


void(* resetFunc) (void) = 0;

void printMac (void) {
  WiFi.macAddress(mac);
  Serial.print("MAC: ");
  Serial.print(mac[5], HEX);
  Serial.print(":");
  Serial.print(mac[4], HEX);
  Serial.print(":");
  Serial.print(mac[3], HEX);
  Serial.print(":");
  Serial.print(mac[2], HEX);
  Serial.print(":");
  Serial.print(mac[1], HEX);
  Serial.print(":");
  Serial.println(mac[0], HEX);
}

/* Compass */

void initCompass (void) {
  Wire.begin(D3, D2);
  Serial.println("Initializing I2C devices...");
  mag.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(mag.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");
}

void loopCompass (void) {
  // read raw heading measurements from device
  mag.getHeading(&mx, &my, &mz);

  // To calculate heading in degrees. 0 degree indicates North
  float heading = atan2(my, mx);
  if (heading < 0)
    heading += 2 * M_PI;
  //  Serial.print("heading:\t");
  //  Serial.println(heading * 180/M_PI);
  rotation = heading * 180 / M_PI;
}

//void timer1Callback(void) {
//  os_intr_lock();
//  Serial.println("1");
//  os_intr_unlock();
//}

void tickerCallback (void) {
  // disable ISR
  os_intr_lock();
  if (wifiTriggered) {
//    Serial.println(millis());
    if (millis() - wifiConnectTime > 300 || millis() - wifiRequestTime > 300) {
      switch (random(10)) {
        case 0:
          turnMotor(true);
          break;
        case 1:
          turnMotor(false);
          break;
        case 2:
          resetMotor();
          break;
      }
    }
  } else {
//    timerMPUTicked = true;
    //  loopMPU();
    loopCompass();
  
    if (isRotating) {
      float degDiff = fabs(rotation - rotationFromTo.y);
      if (degDiff > 180)
        degDiff = 360 - degDiff;
      //    Serial.print("deg: ");
      //    Serial.println(degDiff);
      if (degDiff < 5) {
        resetMotor();
        isRotating = false;
      }
    }
  }

  // enable ISR
  os_intr_unlock();
}

void stopTicker() {
  ticker.detach();
}

void startTicker(long interval) {
  // enable the timer interrupt for the compass
  ticker.attach_ms(interval, tickerCallback);
  
//  os_timer_setfn(&timer1, , NULL);
//  os_timer_arm(&timer1, timerMPUInterval, REPEAT);
//  ticker.attach_ms(timerMPUInterval, timer1Callback);
}

/* Wifi */

void initWifi(void) {
  // We start by connecting to a WiFi network

  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    Serial.print(".");
  }

  printMac();

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

/*  Movement  */
Vec2 mul (Vec2 v2, float multipler) {
  return {v2.x * multipler, v2.y * multipler};
}

Vec2 div (Vec2 v2, float multipler) {
  return {v2.x / multipler, v2.y / multipler};
}

Vec2 add (Vec2 v2a, Vec2 v2b) {
  return {v2a.x + v2b.x, v2a.y + v2b.y};
}

Vec2 sub (Vec2 v2a, Vec2 v2b) {
  return {v2a.x - v2b.x, v2a.y - v2b.y};
}

float magnitude (Vec2 v2) {
  return sqrtf(pow(v2.x, 2) + pow(v2.y, 2));
}

float distance (Vec2 v2a, Vec2 v2b) {
  return sqrtf(pow((v2a.x - v2b.x), 2) + pow((v2a.y - v2b.y), 2));
}

Vec2 normalize (Vec2 v2) {
  float mag = magnitude(v2);
  if (mag == 0) {
    return {0, 0};
  }
  return {v2.x / mag, v2.y / mag};
}

void printVec (const char* str, Vec2 v2) {
  if (str) {
    Serial.print(str);
  }
  Serial.print(v2.x);
  Serial.print(", ");
  Serial.println(v2.y);
}

void addForce (Vec2 v2) {
  printVec("force: ", v2);
  acceleration = add(acceleration, div(v2, mass));
}

Vec2 limit (Vec2 v2) {
  if (magnitude(v2) > maxForceMagnitude) {
    return mul(normalize(v2), maxForceMagnitude);
  }

  return v2;
}


Vec2 steer (Vec2 desired) {
  Vec2 s = sub(mul(normalize(desired), maxSpeedMagnitude), velocity);
  return limit(s);
}

Vec2 seekTarget (Vec2 target) {
  Vec2 desired = sub(target, location);
  return steer(desired);
}

Vec2 separate (Vec2 l) {
  float sd = 1000;
  float d = distance(l, location);
  Vec2 sum = {0, 0};
  int count = 0;
  Serial.print("dist: ");
  Serial.println(d);
  if (d < sd) {
    Vec2 diff = sub(l, location);
    sum = add(div(normalize(diff), d), sum);
    count++;
  }

  if (count > 0) {
    //    sum = mul(normalize(div(sum, count)), maxSpeedMagnitude);
    return steer(sum);
  } else {
    return sum;
  }
}

void updatePort(unsigned char value) {
  digitalWrite(PORT_LATCH_PIN, LOW);
  shiftOut(PORT_DATA_PIN, PORT_CLOCK_PIN, LSBFIRST, value);
  digitalWrite(PORT_LATCH_PIN, HIGH);
}

void controlMotor (int EN_PIN, int IN1_PIN, int IN2_PIN, int dir, int power) {
  if (dir == 1) {
    //    digitalWrite(IN1_PIN, HIGH);
    motorOutput |= (1 << IN1_PIN);
    //    digitalWrite(IN2_PIN, LOW);
    motorOutput &= ~(1 << IN2_PIN);
    analogWrite(EN_PIN, power);
  } else if (dir == -1) {
    //    digitalWrite(IN1_PIN, LOW);
    motorOutput &= ~(1 << IN1_PIN);
    //    digitalWrite(IN2_PIN, HIGH);
    motorOutput |= (1 << IN2_PIN);
    analogWrite(EN_PIN, power);
  } else {
    //    digitalWrite(IN1_PIN, HIGH);
    motorOutput |= (1 << IN1_PIN);
    //    digitalWrite(IN2_PIN, HIGH);
    motorOutput |= (1 << IN2_PIN);
    analogWrite(EN_PIN, 0);
  }
  updatePort(motorOutput);
}

void turnMotor (bool clockwise) {
  Serial.println("turn motor");
  if (clockwise) {
    lastTurnDir = 1;
    controlMotor (MOTOR_A_EN, MOTOR_A_IN1, MOTOR_A_IN2, 1, rotateMotorOutput);
    controlMotor (MOTOR_B_EN, MOTOR_B_IN1, MOTOR_B_IN2, -1, rotateMotorOutput);
  } else {
    lastTurnDir = -1;
    controlMotor (MOTOR_A_EN, MOTOR_A_IN1, MOTOR_A_IN2, -1, rotateMotorOutput);
    controlMotor (MOTOR_B_EN, MOTOR_B_IN1, MOTOR_B_IN2, 1, rotateMotorOutput);
  }
}

void moveAMotor (int dir, int out) {
  motorAOutput = out;
  controlMotor (MOTOR_A_EN, MOTOR_A_IN1, MOTOR_A_IN2, dir, out);
}

void moveBMotor (int dir, int out) {
  motorBOutput = out;
  controlMotor (MOTOR_B_EN, MOTOR_B_IN1, MOTOR_B_IN2, dir, out);
}

void moveMotor(int dir, int aOut, int bOut) {
  moveAMotor(dir, aOut);
  moveBMotor(dir, bOut);
}

bool rotateTo (float desired) {
  float diff = rotation - desired;
  //  Serial.println(diff);
  if (diff < -2) {
    return rotateBy(diff * -1);
  } else if (diff > 2) {
    return rotateBy(360 - diff);
  }
}

void moveForward(long duration) {
  lastRotation = rotation;
  int selfCorrectInterval = 300;
  do {
    moveMotor(1, moveMotorOutput, moveMotorOutput);
    delay(selfCorrectInterval);
    rotateTo(lastRotation);
    duration -= selfCorrectInterval;
  } while (duration > 0);

  resetMotor();
}

void moveBackward(long duration) {
  lastRotation = rotation;
  int selfCorrectInterval = 300;
  do {
    moveMotor(-1, moveMotorOutput, moveMotorOutput);
    delay(selfCorrectInterval);
    rotateTo(lastRotation);
    duration -= selfCorrectInterval;
  } while (duration > 0);

  resetMotor();
}

bool rotateBy(float deg) {
  Serial.println("rotate by");
  long startTime = millis();
  int trial = rotationTrial;
  rotationFromTo = {rotation, ((int)(rotation + deg) % 360)};
  //  printVec("rotate from to: ", rotationFromTo);
  isRotating = true;
  //  controlMotor (MOTOR_A_EN, MOTOR_A_IN1, MOTOR_A_IN2, 1, rotateMotorOutput);
  //  controlMotor (MOTOR_B_EN, MOTOR_B_IN1, MOTOR_B_IN2, -1, rotateMotorOutput);
  turnMotor(deg > 180);

  // finish rotate or timeout
  while (isRotating) {
    yield();
    if (millis() - startTime > rotationTimeout) {
      // try to fix
      trial--;
      switch (trial) {
        case 1:
          moveMotor(1, moveMotorOutput, moveMotorOutput);
          break;
        case 2:
          moveMotor(-1, moveMotorOutput, moveMotorOutput);
          break;
        default:
          return false;  
      }
      Serial.print(rotationTrial - trial);
      Serial.println("-th fix");
      delay(300);
      turnMotor(deg > 180);
      startTime = millis();
    }
  }
  Serial.println("end rotate by");
  return true;
}

void updateRotation() {

}

void resetMotor () {
  Serial.println("reset motor");
  isRotating = false;
  isForward = false;
  controlMotor (MOTOR_A_EN, MOTOR_A_IN1, MOTOR_A_IN2, 0, 128);
  controlMotor (MOTOR_B_EN, MOTOR_B_IN1, MOTOR_B_IN2, 0, 128);
  //  turnDir = 0;
}

void updateMotor () {
  // from velocity to motor behaviour
  int a, b;
  float ratio = 1 - fabs(atan2(velocity.y, velocity.x) * 2 / PI - 1);
  Serial.print("ratio: ");
  Serial.println(ratio);
  float mag = magnitude(velocity);
  if (velocity.y < 0) {
    if (velocity.x > 0) {
      // turn right
      b = unitMotorOutput;
      a = maxMotorOutput;
    } else {
      // turn left
      a = unitMotorOutput;
      b = maxMotorOutput;
    }
  } else {
    if (velocity.x > 0) {
      a = _min(mag, maxSpeedMagnitude) / maxSpeedMagnitude * maxMotorOutput;
      b = a * ratio;
    } else {
      b = _min(mag, maxSpeedMagnitude) / maxSpeedMagnitude * maxMotorOutput;
      a = b * ratio;
    }
  }

  controlMotor (MOTOR_A_EN, MOTOR_A_IN1, MOTOR_A_IN2, 1, a);
  controlMotor (MOTOR_B_EN, MOTOR_B_IN1, MOTOR_B_IN2, 1, b);

  Serial.print("A: ");
  Serial.print(a);
  Serial.print("\tB: ");
  Serial.println(b);
}

bool isRotationInBetween (float a, float b) {
  if (a >= 270)
    b += 360;
  return rotation <= _max(a, b) && rotation >= _min(a, b);
}

int getQuadrant () {
  if (isRotationInBetween(westAngle, northAngle)) {
    return 4;
  } else if (isRotationInBetween(southAngle, westAngle)) {
    return 3;
  } else if (isRotationInBetween(northAngle, eastAngle)) {
    return 1;
  } else if (isRotationInBetween(eastAngle, southAngle)) {
    return 2;
  }

  Serial.println("Error. should not reach here");
  return 0;
}

bool isRobotOutBound () {
  return location.x <= (boundX * -1) || location.x >= boundX || location.y <= (boundY * -1) || location.y >= boundY;
}

bool boundBy () {
  Serial.println("boundby");
  isXBound = location.x <= (boundX * -1) || location.x >= boundX;
  isYBound = location.y <= (boundY * -1) || location.y >= boundY;

  quadrant = getQuadrant();
  if (isXBound && isYBound) {
    //    turnMotor(lastTurnDir > 0);
    bool bottomLeft = location.x <= (boundX * -1) && location.y <= (boundY * -1);
    bool bottomRight = location.x >= boundX && location.y <= (boundY * -1);
    bool topRight = location.x >= boundX && location.y >= boundY;
    bool topLeft = location.x <= (boundX * -1) && location.y >= boundY;

    if (bottomLeft) {
      switch (quadrant) {
        case 3:
          rotateBy(180);
          break;
        case 4:
          rotateBy(90);
          break;
        case 2:
          rotateBy(270);
          break;
      }
    } else if (bottomRight) {
      switch (quadrant) {
        case 2:
          rotateBy(180);
          break;
        case 3:
          rotateBy(90);
          break;
        case 1:
          rotateBy(270);
          break;
      }
    } else if (topLeft) {
      switch (quadrant) {
        case 4:
          rotateBy(180);
          break;
        case 1:
          rotateBy(90);
          break;
        case 3:
          rotateBy(270);
          break;
      }
    } else if (topRight) {
      switch (quadrant) {
        case 1:
          rotateBy(180);
          break;
        case 2:
          rotateBy(90);
          break;
        case 4:
          rotateBy(270);
          break;
      }
    }

  } else if (isXBound || isYBound) {
    //Check coordinate

    if (isXBound) {
      //Q4
      if (quadrant == 4) {
        turnDir = 1;
      }
      //Q3
      if (quadrant == 3) {
        turnDir = -1;
      }
      //Q1
      if (quadrant == 1) {
        turnDir = -1;
      }
      //Q2
      if (quadrant == 2) {
        turnDir = 1;
      }
    }

    if (isYBound) {
      //Q2
      if (quadrant == 2) {
        turnDir = -1;
      }

      //Q3
      if (quadrant == 3) {
        turnDir = 1;
      }

      //Q4
      if (quadrant == 4) {
        turnDir = -1;
      }

      //Q1
      if (quadrant == 1) {
        turnDir = 1;
      }
    }

    //    turnMotor(turnDir > 0);
    rotateBy(turnDir > 0 ? 90 : 270);
  }

  return isXBound || isYBound;
}

double getDirectionFrom (Vec2 from, bool isPointing) {
  double shiftAngle = 360 - (atan2 (from.y - location.y, from.x - location.x) * 180 / PI - 90) +
                      northAngle + (isPointing ? 0 : 180);
  while (shiftAngle > 360) {
    shiftAngle -= 360;
  }

  while (shiftAngle < 0) {
    shiftAngle += 360;
  }

  return shiftAngle;
}

void updateMovement () {
  velocity = add(velocity, acceleration);
  updateMotor();
  acceleration = {0, 0};
}

void updateLocationAndVelocity (Vec2 l, Vec2 v, Vec2 e, Vec2 f) {
  Serial.println("update location");
  location = l;
  velocity = v;
  enemyLocation = e;
  friendLocation = f;
  printVec("l: ", location);
  printVec("v: ", velocity);
  printVec("e: ", enemyLocation);
  printVec("f: ", friendLocation);
  //  isReceived = true;
}

/* SPI Port Expander */

void initPort() {
  Serial.println("init Port");
  pinMode(PORT_LATCH_PIN, OUTPUT);
  pinMode(PORT_DATA_PIN, OUTPUT);
  pinMode(PORT_CLOCK_PIN, OUTPUT);
}

void loopPort() {
  //  Serial.println("loop Port");
  //  int power = 800;
  //  controlMotor (MOTOR_A_EN, MOTOR_A_IN1, MOTOR_A_IN2, 0, power);
  //  delay(500);
  //  controlMotor (MOTOR_B_EN, MOTOR_B_IN1, MOTOR_B_IN2, 0, power);
  //  delay(500);
  //  controlMotor (MOTOR_A_EN, MOTOR_A_IN1, MOTOR_A_IN2, 1, power);
  //  delay(500);
  //  controlMotor (MOTOR_A_EN, MOTOR_A_IN1, MOTOR_A_IN2, -1, power);
  //  delay(500);
  //  controlMotor (MOTOR_B_EN, MOTOR_B_IN1, MOTOR_B_IN2, 1, power);
  //  delay(500);
  //  controlMotor (MOTOR_B_EN, MOTOR_B_IN1, MOTOR_B_IN2, -1, power);
  //  delay(500);
  if (!isRotating) {
    delay(2000);
    Serial.println("Start rotating");
    moveForward(1000);
    rotateBy(90);
//    moveBackward();
    delay(1000);
  }
}

/* Wifi */

void readJson (char* jsonInput) {
  //  Serial.println("json received:");
  //  Serial.println(jsonInput);
  StaticJsonBuffer<JSON_PARSE_BUFFER_SIZE> jsonBufferTest;
  JsonObject& rootTest = jsonBufferTest.parseObject(jsonInput);
  if (rootTest.success()) {
    Serial.println(F("Json parsed successfully."));
    isInScreen = rootTest[robotName]["isOn"];
    isEnemyInScreen = rootTest[enemyName]["isOn"];
    isFriendInScreen = rootTest[friendName]["isOn"];
    updateLocationAndVelocity(
      {rootTest[robotName]["x"], rootTest[robotName]["y"]},
      {0, 0},
      {rootTest[enemyName]["x"], rootTest[enemyName]["y"]},
      {rootTest[friendName]["x"], rootTest[friendName]["y"]});
  } else {
    Serial.println("Json parsing failed.");
  }
}

void printJson (JsonObject& jsonObj) {
  for (JsonObject::iterator it = jsonObj.begin(); it != jsonObj.end(); ++it) {
    Serial.println(it->key);
    if ((it->value).is<JsonObject&>()) {
      JsonObject& obj = (it->value).asObject();
      for (JsonObject::iterator innerIt = obj.begin(); innerIt != obj.end(); ++innerIt) {
        Serial.print(innerIt->key);
        Serial.print(": ");
        Serial.println(innerIt->value.asString());
      }
    } else {
      Serial.println(it->value.asString());
    }
  }
}

void uploadData () {
  /*  */
  Serial.print("connecting to ");
  Serial.println("data.sparkfun.com");

  // Use WiFiClient class to create TCP connections
  WiFiClient client;
  const int httpPort = 80;
  if (!client.connect("data.sparkfun.com", httpPort)) {
    Serial.println("connection failed, restart the programme");
    resetFunc();
  }

  // We now create a URI for the request
  String url = "/input/pw77b7jYOvcXa4xWmjbL?private_key=64nn0nV65AhrAEeBWgZN&rotation=";
  url += String(rotation);
  url += "&speed=";
  url += String(turnDir);
  url += "&x=";
  url += String(location.x);
  url += "&y=";
  url += String(location.y);
  url += "&ma=";
  url += String(motorAOutput);
  url += "&mb=";
  url += String(motorBOutput);
  url += "&q=";
  url += String(quadrant);
  url += "&screen=";
  url += String(isInScreen);
  url += "&tc=";
  url += String(wifiConnectTime);
  url += "&tr=";
  url += String(wifiRequestTime);
  //  url += streamId;
  //  url += "?private_key=";
  //  url += privateKey;
  //  url += "&value=";
  //  url += value;

  Serial.print("Requesting URL: ");
  Serial.println(url);

  // This will send the request to the server
  client.print("GET " + url + " HTTP/1.1\r\n" +
               "Host: " + "data.sparkfun.com" + "\r\n" +
               "Connection: close\r\n\r\n\n");
  unsigned long timeout = millis();
  while (client.available() == 0) {
    if (millis() - timeout > 5000) {
      Serial.println(">>> Client Timeout !");
      client.stop();
      return;
    }
  }

  unsigned long lastRead = millis();

  // Read all the lines of the reply from server and print them to Serial
  while (client.available()) {
    //    String line = client.readStringUntil('\r');
    //    Serial.print(line);
    char c = client.read();
    Serial.print(c);

    lastRead = millis();
  }
}

void fetchJson () {
  stopTicker();
  startTicker(150);
  wifiConnectTime = 0;
  wifiRequestTime = 0;
  wifiTriggered = true;
  
  Serial.print("connecting to ");
  Serial.println(host);

  wifiConnectTime = millis();
  /* time: 1000~1860ms */
  // Use WiFiClient class to create TCP connections
  WiFiClient client;
  const int httpPort = 80;
  if (!client.connect(host, httpPort)) {
    Serial.println("connection failed, restart the programme");
    resetFunc();
  }
  /* END */
  wifiConnectTime = millis() - wifiConnectTime;

  // We now create a URI for the request
  String url = path;
  //  url += streamId;
  //  url += "?private_key=";
  //  url += privateKey;
  //  url += "&value=";
  //  url += value;

  Serial.print("Requesting URL: ");
  Serial.println(url);

  wifiRequestTime = millis();
  /* time: 220~3000ms */
  // This will send the request to the server
  client.print("GET " + url + " HTTP/1.1\r\n" +
               "Host: " + host + "\r\n" +
               "Cache-Control: no-cache, no-store, must-revalidate\r\n" +
               "Pragma: no-cache\r\nExpires: 0\r\n" +
               "Connection: close\r\n\r\n\n");
  unsigned long timeout = millis();
  while (client.available() == 0) {
    if (millis() - timeout > 5000) {
      Serial.println(">>> Client Timeout !");
      client.stop();
      return;
    }
  }
  /* END */
  wifiRequestTime = millis() - wifiRequestTime;

  unsigned long lastRead = millis();
  char jsonInput[JSON_READ_BUFFER_SIZE];
  int jsonInputCount = 0;
  unsigned char jsonInputObjectCount = 0;

  /* time: 50ms */
  // Read all the lines of the reply from server and print them to Serial
  while (client.available()) {
    //    String line = client.readStringUntil('\r');
    //    Serial.print(line);
    char c = client.read();
    Serial.print(c);

    // algo for paring the json
    if (c == '{') {
      //      Serial.println("{");
      jsonInputObjectCount++;
      jsonInput[jsonInputCount++] = c;
    } else if (jsonInputObjectCount > 0) {
      if (c == '}') {
        //        Serial.println("}");
        jsonInputObjectCount--;
      }
      jsonInput[jsonInputCount++] = c;
    }

    lastRead = millis();
  }
  /* END */

  // make an end for the json
  jsonInput[jsonInputCount] = '\0';

  Serial.println();
  Serial.println("closing connection");

  Serial.println(F("Start json reading.\n"));
  readJson(jsonInput);
  Serial.println(F("\nComplete json reading."));

  stopTicker();
  startTicker(tickerInterval);
  wifiTriggered = false;
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  pinMode(MOTOR_A_EN, OUTPUT);
  pinMode(MOTOR_B_EN, OUTPUT);

  motorOutput = 0;
  resetMotor();

  velocity = {0, 0};
  location = {0, 0};
  target = {0, 0};
  acceleration = {0, 0};
  enemyLocation = {0, 0};

  Serial.print("PI: ");
  Serial.println(PI);
  Serial.print("Max speed: ");
  Serial.println(maxSpeedMagnitude);
  Serial.print("Max force: ");
  Serial.println(maxForceMagnitude);

  Serial.println("motion init done");

  ESP.wdtEnable(10000);

  initWifi();
  pinMode(WIFI_UPLOAD_DATA_PIN, INPUT_PULLUP);
  //  digitalWrite(WIFI_UPLOAD_DATA_PIN, HIGH); // pull up

  initPort();
  //  initMPU();
  initCompass();

  ESP.wdtEnable(1000);

  startTicker(tickerInterval);
}

void loop() {
  if (isReceived) {
    Serial.println("activate motor");
    isReceived = false;

    //    addForce(seekTarget(target));
    addForce(separate(enemyLocation));
    updateMovement();

    printVec("v: ", velocity);
  } else {
    //      loopPort();
    //      Serial.println(rotation);
    resetMotor();
    fetchJson();

    if (isInScreen) {
      if (boundBy()) {
        moveForward(1000);
      }

      // get away from enemy
      if (isEnemyInScreen) {
        rotateTo(getDirectionFrom(enemyLocation, false));
        moveForward(1500);
      }

      if (isFriendInScreen) {
        rotateTo(getDirectionFrom(friendLocation, true));
        moveForward(1000);
      }
    } else {
      // point back to origin and turn
      if (isOddTrial) {
        isOddTrial = false;
        rotateTo(getDirectionFrom({0, 0}, true));
        moveForward(1500);
      } else {
        isOddTrial = true;
        rotateTo(getDirectionFrom({0, 0}, false));
        moveBackward(1500);
      }
    }

    Serial.print("upload? ");
    Serial.println(digitalRead(WIFI_UPLOAD_DATA_PIN));
    if (digitalRead(WIFI_UPLOAD_DATA_PIN) == HIGH) {
      uploadData();
    }
  }
}
