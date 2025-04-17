#include <SPI.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Arduino_LSM6DS3.h>

int status = WL_IDLE_STATUS;
char ssid[] = "tensegrity"; // your network SSID (name)
char pass[] = "augustin";        // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;                // your network key Index number (needed only for WEP)
char PC_IP[] = "10.42.0.1"; // change to PC IP address

unsigned int localPort = 2390; // local port to listen on

/////////////////////////
//Configuration Arduino//
/////////////////////////

#define N_Arduino 0 // To change in function of which Arduino is used

char packetBuffer[255];  // buffer to hold incoming packet
#define OFFSET 3  //Offset start of message
String sensorDataString;

WiFiUDP Udp;

#define WIRE_PORT Wire // Your desired Wire port. 

// Arduino Nano on-board IMU
float ax, ay, az; // accelerometer
float gx, gy, gz; // gyroscope

const int offsetA = 1;
const int offsetB = 1;

void setup() {
  
  // Check for the presence of the WiFi shield:
  // Serial.begin(115200);
  if (WiFi.status() == WL_NO_SHIELD){
    // Don't continue:
    // Serial.println("No Shield");
    while (true);
  }

  String fv = WiFi.firmwareVersion(); 
  if (fv != "1.5.0") {
    // Serial.println("Firmware");
    // Don't continue:
    while (true);
  }

  // Attempt to connect to WiFi network:
  while (status != WL_CONNECTED){
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);
    // status = WiFi.begin(ssid);
    // Wait 1 second for connection:
    // Serial.println("Connecting...");
    delay(1000);
  }

  Udp.begin(localPort);

  if (!IMU.begin()) {
    // Serial.println("Failed to initialize IMU!");
    sensorDataString ="Failed to initialize IMU!";
    // Udp.beginPacket("10.42.0.1", 2390); // Replace with the Python code IP and port
    Udp.beginPacket(PC_IP, 2390); // Replace with the Python code IP and port
    Udp.write(sensorDataString.c_str());
    Udp.endPacket();
    while (1);
  }
}

void loop() {

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);
  }

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);
  }

  sensorDataString = String(ax) + " "
                  + String(ay) + " "
                  + String(az) + " "
                  + String(gx) + " "
                  + String(gy) + " "
                  + String(gz);

  Udp.beginPacket(PC_IP, 2390); // Replace with the Python code IP and port
  Udp.write(sensorDataString.c_str());
  Udp.endPacket();
}