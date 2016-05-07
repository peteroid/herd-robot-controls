/*
 *  This sketch sends data via HTTP GET requests to data.sparkfun.com service.
 *
 *  You need to get streamId and privateKey at data.sparkfun.com and paste them
 *  below. Or just customize this script to talk to other HTTP servers.
 *
 */

#include <ESP8266WiFi.h>
//#include <ESP.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <math.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

extern "C" {
  #include "user_interface.h"
}


#define _min(a,b) ((a)<(b)?(a):(b))

#define JSON_READ_BUFFER_SIZE  450
#define JSON_PARSE_BUFFER_SIZE 360

/* MPU6050 */

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

#define NODE_INTERRUPT_PIN D1
#define NODE_SDA_PIN       D2
#define NODE_SCL_PIN       D4
//#define LED_PIN       13
//bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float lastYaw[7] = {0, 0, 0, 0, 0, 0, 0};
int lastYawIndex = 0;
bool isCalibrated = false;
int minCalibrateIterations = 10;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
//  Serial.println("interrupt!");
}

int timerMPUInterval = 50;
os_timer_t timerMPU;
bool timerMPUTicked = false;
void timerCallback(void *pArg);

/* WIFI */
//const char* ssid     = "UCInet Mobile Access";
const char* ssid     = "ResNet Mobile Access";
const char* password = "";

//const char* ssid     = "PETEroiFi";
//const char* password = "00000000";

const char* host = "4eaf4a66.ngrok.io";
//const char* host = "dhcp-10-8-019-072.mobile.reshsg.uci.edu";
//const char* path = "/";
const char* path = "/~PETEroid/json/output.json";
//const char* streamId   = "....................";
//const char* privateKey = "....................";

const char* robotName = "red";
const char* enemyName = "green";

byte mac[6];
bool isReceived = false;

/* SPI Port Expander */
#define PORT_LATCH_PIN D3
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

typedef struct {
  float x;
  float y;
} Vec2;

Vec2 velocity, location, target, acceleration, enemyLocation;
float mass = 1.0;
float rotation = 0;
Vec2 rotationFromTo = {0, 0};
int maxMotorOutput = 999;
int minMotorOutput = 128;
int unitMotorOutput = 128;
int rotateMotorOutput = 999;

int maxSpeedMagnitude = ceil(maxMotorOutput / unitMotorOutput);
int maxForceMagnitude = maxSpeedMagnitude;

bool isRotating = false;

void resetMotor();



void(* resetFunc) (void) = 0;

void printMac (void) {
  WiFi.macAddress(mac);
  Serial.print("MAC: ");
  Serial.print(mac[5],HEX);
  Serial.print(":");
  Serial.print(mac[4],HEX);
  Serial.print(":");
  Serial.print(mac[3],HEX);
  Serial.print(":");
  Serial.print(mac[2],HEX);
  Serial.print(":");
  Serial.print(mac[1],HEX);
  Serial.print(":");
  Serial.println(mac[0],HEX);
}

/* MPU */

void initMPU (void) {
  Serial.println("\nInit MPU");
  
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin(NODE_SDA_PIN, NODE_SCL_PIN);
//  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  Wire.setClock(5000);

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(NODE_INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    Serial.println(digitalPinToInterrupt(NODE_INTERRUPT_PIN));
    attachInterrupt(digitalPinToInterrupt(NODE_INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

bool loopMPU (void) {
//  Serial.println("Get data from MPU");
  
  if (!dmpReady)
    return false;

  while (!mpuInterrupt && fifoCount < packetSize) {
    yield();
  }
  
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
//    Serial.print(F("FIFO overflow! int: "));
//    Serial.print(mpuIntStatus);
//    Serial.print(", fifo count: ");
//    Serial.print(fifoCount);

    return false;
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    rotation = ypr[0] * 180 / M_PI;
//    Serial.print("ypr\t");
//    Serial.println(ypr[0] * 180 / M_PI);
//    Serial.print("\t");
//    Serial.print(ypr[1] * 180 / M_PI);
//    Serial.print("\t");
//    Serial.println(ypr[2] * 180 / M_PI);
    if (isRotating) {
      float degDiff = fabs(rotation - rotationFromTo.y);
      if (degDiff > 180)
        degDiff = 360 - degDiff;
      Serial.print("deg: ");
      Serial.println(degDiff);
      if (degDiff < 3) {
        resetMotor();
        isRotating = false;
      }
    }

    return true;
  }
}

void timerMPUCallback(void *pArg) {
  loopMPU();
  timerMPUTicked = true;
}

void startTimerMPU(void) {
// enable the timer interrupt for the mpu
  os_timer_setfn(&timerMPU, timerMPUCallback, NULL);
  os_timer_arm(&timerMPU, timerMPUInterval, true);
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

void moveMotor(int dir) {
  controlMotor (MOTOR_A_EN, MOTOR_A_IN1, MOTOR_A_IN2, dir, rotateMotorOutput);
  controlMotor (MOTOR_B_EN, MOTOR_B_IN1, MOTOR_B_IN2, dir, rotateMotorOutput);
}

void moveForward() {
  moveMotor(1);
}

void moveBackward() {
  moveMotor(-1);
}

void rotateBy(float deg) {
  rotationFromTo = {rotation, ((int)(rotation + deg + 180) % 360) - 180};
  printVec("rotate from to: ", rotationFromTo);
  isRotating = true;
  controlMotor (MOTOR_A_EN, MOTOR_A_IN1, MOTOR_A_IN2, 1, rotateMotorOutput);
  controlMotor (MOTOR_B_EN, MOTOR_B_IN1, MOTOR_B_IN2, -1, rotateMotorOutput);
  while(isRotating) {
    yield();
  }
}

void updateRotation() {
  
}

void resetMotor () {
  controlMotor (MOTOR_A_EN, MOTOR_A_IN1, MOTOR_A_IN2, 0, 128);
  controlMotor (MOTOR_B_EN, MOTOR_B_IN1, MOTOR_B_IN2, 0, 128);
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

void updateMovement () {
  velocity = add(velocity, acceleration);
  updateMotor();
  acceleration = {0, 0};
}

void updateLocationAndVelocity (Vec2 l, Vec2 v, Vec2 e) {
  Serial.println("update location");
  location = l;
  velocity = v;
  enemyLocation = e;
  printVec("l: ", location);
  printVec("v: ", velocity);
  printVec("e: ", enemyLocation);
  isReceived = true;
}

//void serialEvent() {
//  if (Serial.available()) {
//    updateLocation({Serial.parseFloat(), Serial.parseFloat()});
//  }
//}

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
    moveForward();
    delay(1000);
    rotateBy(90);
    moveBackward();
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
    Vec2 newLocation = {rootTest[robotName]["x"], rootTest[robotName]["y"]};
//    Vec2 newVelocity = {rootTest[robotName]["vx"], rootTest[robotName]["vy"]};
    Vec2 newVelocity = {0, 0};
    Vec2 newEnemyLocation = {rootTest[enemyName]["x"], rootTest[enemyName]["y"]};
    updateLocationAndVelocity(newLocation, newVelocity, newEnemyLocation);
  } else {
    Serial.println("Json parsing failed.");
  }
}

void printJson (JsonObject& jsonObj) {
  for (JsonObject::iterator it=jsonObj.begin(); it!=jsonObj.end(); ++it) {
    Serial.println(it->key);
    if ((it->value).is<JsonObject&>()) {
      JsonObject& obj = (it->value).asObject();
      for (JsonObject::iterator innerIt=obj.begin(); innerIt!=obj.end(); ++innerIt) {
        Serial.print(innerIt->key);
        Serial.print(": ");
        Serial.println(innerIt->value.asString());
      }
    } else {
      Serial.println(it->value.asString());
    }
  }
}

void fetchJson () {
  Serial.print("connecting to ");
  Serial.println(host);
  
  // Use WiFiClient class to create TCP connections
  WiFiClient client;
  const int httpPort = 80;
  if (!client.connect(host, httpPort)) {
    Serial.println("connection failed, restart the programme");
    resetFunc();
  }
  
  // We now create a URI for the request
  String url = path;
//  url += streamId;
//  url += "?private_key=";
//  url += privateKey;
//  url += "&value=";
//  url += value;
  
  Serial.print("Requesting URL: ");
  Serial.println(url);
  
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

  unsigned long lastRead = millis();
  char jsonInput[JSON_READ_BUFFER_SIZE];
  int jsonInputCount = 0;
  unsigned char jsonInputObjectCount = 0;
  
  // Read all the lines of the reply from server and print them to Serial
  while(client.available()){
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

  // make an end for the json
  jsonInput[jsonInputCount] = '\0';
  
  Serial.println();
  Serial.println("closing connection");
  
  Serial.println(F("Start json reading.\n"));
  readJson(jsonInput);
  Serial.println(F("\nComplete json reading."));
}

void setup() {
  Serial.begin(115200);
  while(!Serial);

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
  initPort();
  initMPU();

  ESP.wdtEnable(1000);

  Serial.println("Start Calibration");
}

void loop() {
  if (isCalibrated) {
    if (isReceived) {
      Serial.println("activate motor");
      isReceived = false;

//    addForce(seekTarget(target));
      addForce(separate(enemyLocation));
      updateMovement();

      printVec("v: ", velocity);
    } else {
      loopPort();
      Serial.println(rotation);
//    fetchJson();
    }
  } else {
    if (lastYawIndex == 10) {
      lastYawIndex = 0;

      if (minCalibrateIterations != 0) {
        minCalibrateIterations--;
      } else {
        int i;
        float sum = 0;
        for (i = 0; i < 10; i++) {
          sum += lastYaw[i];
        }

        sum /= 10;
        Serial.println(sum);
        if (sum < 0.3) {
          isCalibrated = true;
          Serial.print("Calibration done: ");
          Serial.print(rotation);
          Serial.println("Start MPU Timer");
          startTimerMPU();
          return;
        }
      }
    }

// do the calibration
    float lastRotation = rotation;
    if (loopMPU()) {
//      Serial.print(rotation);
//      Serial.print(" : ");
//      Serial.print(lastRotation);
//      Serial.print(" abs: ");
//      Serial.println(fabs(rotation - lastRotation));
      
      lastYaw[lastYawIndex++] = fabs(rotation - lastRotation);
    }
  }
  delay(50);
}
