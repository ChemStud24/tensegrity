#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include "Adafruit_MPR121.h"
#include <SparkFun_TB6612.h>
#include <Encoder.h>
#include <Arduino_LSM6DS3.h>
#include "Adafruit_VL53L1X.h"

int status = WL_IDLE_STATUS;
char ssid[] = "CS-Robots";
char pass[] = "kaQcVTdG4CfpDWqZ";
char PC_IP[] = "172.16.71.4";
char PC_IP_2[] = "172.16.71.35";
unsigned int localPort = 2390;

#define N_Arduino 0
char arduinoIP[] = "172.16.71.78";

int motor_numbers[3][2] = {{1,0},{3,2},{5,4}};
#define Nb_motors 6
int N_motor1 = motor_numbers[N_Arduino][0];
int N_motor2 = motor_numbers[N_Arduino][1];

WiFiUDP Udp;
#define WIRE_PORT Wire

/////////////////////////
// MPR121
/////////////////////////

const int NUM_SENSORS = 4;
float capacitance[NUM_SENSORS];
float tempCap[NUM_SENSORS];

int chargeCurrent = 42;
float chargeTime = 1;

Adafruit_MPR121 cap = Adafruit_MPR121();

/////////////////////////
// Encoders
/////////////////////////

Encoder enc_1(9,10);
Encoder enc_2(11,13);
long enc_counts_1 = 0;
long enc_counts_2 = 0;

/////////////////////////
// IMU
/////////////////////////

float ax, ay, az;
float gx, gy, gz;

/////////////////////////
// Motors
/////////////////////////

#define AIN1 17
#define AIN2 15
#define PWMA 16
#define STBY 6
#define BIN1 5
#define BIN2 4
#define PWMB 3

Motor motor1 = Motor(AIN1, AIN2, PWMA, 1, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, 1, STBY);

/////////////////////////
// VL53L1X
/////////////////////////

#define IRQ_PIN 2
#define XSHUT_1 7
#define XSHUT_2 8

Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(XSHUT_1, IRQ_PIN);
Adafruit_VL53L1X vl53_2 = Adafruit_VL53L1X(XSHUT_2, IRQ_PIN);

int16_t distance = -1;
int16_t distance_2 = -1;

String sensorDataString;

//////////////////////////////////////////////////////////////
// CONFIG FUNCTIONS
//////////////////////////////////////////////////////////////

void config_with_settings(Adafruit_MPR121 board) {
  board.writeRegister(MPR121_AUTOCONFIG0, 0x00);
  board.writeRegister(MPR121_CONFIG1, byte(chargeCurrent));
  board.writeRegister(MPR121_CONFIG2, time2Reg(chargeTime));
}

byte time2Reg(float T) {
  if (T == 0.5) return 0b00100000;
  byte n = log(2 * T) / log(2) + 1;
  return n << 5;
}

//////////////////////////////////////////////////////////////
// VL53 INIT FUNCTION
//////////////////////////////////////////////////////////////

void initVL53() {
  Serial.println("Init");  
  pinMode(XSHUT_1, OUTPUT);
  pinMode(XSHUT_2, OUTPUT);

  digitalWrite(XSHUT_1, LOW);
  digitalWrite(XSHUT_2, LOW);
  delay(10);

  // Sensor 1
  digitalWrite(XSHUT_1, HIGH);
  delay(10);
  while(!vl53.begin(0x29, &Wire)){
    Serial.println("Stuck here");
  }
  vl53.setTimingBudget(50);
  vl53.startRanging();

  // Sensor 2
  digitalWrite(XSHUT_2, HIGH);
  delay(10);
  while(!vl53_2.begin(0x29, &Wire)){
    Serial.print("Stuck here");
  }
  vl53_2.setTimingBudget(50);
  vl53_2.startRanging();
  Serial.println("Exit");
}

//////////////////////////////////////////////////////////////
// RESET FUNCTION
//////////////////////////////////////////////////////////////

void resetMPR121() {

  motor1.drive(0);
  motor2.drive(0);

  delay(50);

  WIRE_PORT.end();
  delay(50);
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
  delay(50);

  cap.writeRegister(MPR121_SOFTRESET, 0x63);
  delay(100);

  if (!cap.begin(0x5A)) return;

  config_with_settings(cap);
  delay(100);

  // Re-init VL53 sensors correctly
  initVL53();
}

//////////////////////////////////////////////////////////////
// SETUP
//////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(115200);

  while (WiFi.status() == WL_NO_SHIELD);

  while (status != WL_CONNECTED) {
    IPAddress ip, dns, gateway, netmask;
    ip.fromString(arduinoIP);
    dns.fromString("8.8.8.8");
    gateway.fromString("172.16.71.65");
    netmask.fromString("255.255.255.224");

    WiFi.config(ip, dns, gateway, netmask);
    status = WiFi.begin(ssid, pass);
    Serial.println(status);
    delay(1000);
  }

  Udp.begin(localPort);

  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);

  if (!IMU.begin()) while (1);

  if (!cap.begin(0x5A)) while (1);
  config_with_settings(cap);

  // ✅ Correct VL53 init
  initVL53();
}

//////////////////////////////////////////////////////////////
// LOOP
//////////////////////////////////////////////////////////////

void loop() {
  Serial.println("HERE!");
  if (status != WL_CONNECTED) return;

  bool mpr_error = false;

  for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
    int raw = cap.filteredData(sensor);
    if (raw <= 0) { mpr_error = true; break; }

    capacitance[sensor] = chargeCurrent * chargeTime * 1024.0 / raw / 3.3;

    if (abs(capacitance[sensor] - 0.20) < 0.01) {
      mpr_error = true;
      break;
    }
  }

  if (mpr_error) {
    resetMPR121();
    return;
  }

  tempCap[0] = capacitance[3];
  tempCap[1] = capacitance[0];
  tempCap[2] = capacitance[1];
  tempCap[3] = capacitance[2];

  for (int i = 0; i < NUM_SENSORS; i++)
    capacitance[i] = tempCap[i];

  enc_counts_1 = enc_1.read();
  enc_counts_2 = enc_2.read();

  if (IMU.accelerationAvailable())
    IMU.readAcceleration(ax, ay, az);

  if (IMU.gyroscopeAvailable())
    IMU.readGyroscope(gx, gy, gz);

  if (vl53.dataReady()) {
    distance = vl53.distance();
    vl53.clearInterrupt();
  }

  if (vl53_2.dataReady()) {
    distance_2 = vl53_2.distance();
    vl53_2.clearInterrupt();
  }

  sensorDataString = String(N_Arduino) + " " +
                     String(capacitance[0]) + " " +
                     String(capacitance[1]) + " " +
                     String(capacitance[2]) + " " +
                     String(capacitance[3]) + " " +
                     String(enc_counts_1) + " " +
                     String(enc_counts_2) + " " +
                     String(ax) + " " +
                     String(ay) + " " +
                     String(az) + " " +
                     String(gx) + " " +
                     String(gy) + " " +
                     String(gz) + " " +
                     String(distance) + " " +
                     String(distance_2);

  Udp.beginPacket(PC_IP, localPort);
  Udp.write(sensorDataString.c_str());
  Udp.endPacket();

  Udp.beginPacket(PC_IP_2, localPort);
  Udp.write(sensorDataString.c_str());
  Udp.endPacket();

  int packetSize = Udp.parsePacket();

  if (packetSize) {
    char packetBuffer[255];
    Udp.read(packetBuffer, 255);

    double motorPWM[Nb_motors];
    int index = 0, skipped = 0;
    char* token = strtok(packetBuffer, " ");

    while (token != NULL && index < Nb_motors) {
      if (skipped >= 3) motorPWM[index++] = atof(token);
      skipped++;
      token = strtok(NULL, " ");
    }

    motor1.drive((int)(255.0 * motorPWM[N_motor1] / 99.0));
    motor2.drive((int)(255.0 * motorPWM[N_motor2] / 99.0));
  }
}