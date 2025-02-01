#include <SPI.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
// #include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include "Adafruit_MPR121.h" // For MPR 121
#include <SparkFun_TB6612.h> // For DC motor Driver

int status = WL_IDLE_STATUS;
char ssid[] = "tensegrity"; // your network SSID (name)
char pass[] = "augustin";        // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;                // your network key Index number (needed only for WEP)

unsigned int localPort = 2390; // local port to listen on

/////////////////////////
//Configuration Arduino//
/////////////////////////

#define Nb_motors 6 // To change in function of the number of motor used
#define N_motor1 1 // To change in function of motor used
#define N_motor2 3 // To change in function of motor used
#define N_Arduino 1 // To change in function of which Arduino is used 

char packetBuffer[255];  // buffer to hold incoming packet
#define OFFSET 3  //Offset start of message
String sensorDataString;

WiFiUDP Udp;

#define WIRE_PORT Wire // Your desired Wire port. 

#define AD0_VAL 0 // 1 if ICM20948 address is 0x69, 0 if address is 0x68

ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object
double q[4];

const int NUM_SENSORS = 4; // How many sensors will you use?
float capacitance[NUM_SENSORS];
float temp[NUM_SENSORS];

bool configMode = false;
int chargeCurrent = 42; // uA In order to compute capacitance from MPR121 
float chargeTime = 1; // us   In order to compute capacitance from MPR121 

/* You can find those 2 values through the following functions in order to config the MPR121  : 

void print_autoconfig() {
  // read the results of the autoconfiguration
  byte cdc = cap.readRegister8(MPR121_CHARGECURR_0);
  byte cdt = cap.readRegister8(MPR121_CHARGETIME_1);

  // 3.2.1 Print the autoconfiguration results for the user
  Serial.println("Results of the Autoconfiguration:");
    
  // print the current result
  Serial.print("int chargeCurrent = ");
  Serial.print(Reg2Current(cdc));
  Serial.println("; // uA");

  // print the time result
  Serial.print("int chargeTime = ");
  Serial.print(Reg2Time(cdt));
  Serial.println("; // us");

  // reminder instructions
  Serial.println("Fill in this information at the top of the code.");
}

// this converts the CDC register value into a current in uA
int Reg2Current(byte currentRegister) {
  // get the last 6 bits of the current register and convert to int
  return int(currentRegister);
}

// this converts the INDIVIDUAL CDT register of a SINGLE electrode into a time in microseconds
float Reg2Time(byte timeRegister) {
  // get the leftmost 3 bits of the register
  int n = timeRegister & 0b00000111;
  // use the formala on the datasheet to calculate time
  return 0.5*pow(2,n-1);
}
*/

Adafruit_MPR121 cap = Adafruit_MPR121();

#define AIN1 17
#define AIN2 15
#define PWMA 16
#define STBY 6

#define BIN1 5
#define BIN2 4
#define PWMB 3

const int offsetA = 1;
const int offsetB = 1;

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);
void setup() {
  
  // Check for the presence of the WiFi shield:
  Serial.begin(115200);
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
    Serial.println("Connecting...");
    delay(1000);
  }

  Udp.begin(localPort);
  // Serial.println("Here I am");

  /////////////////////
  //Configuration ICM//
  /////////////////////
  
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);

  // bool found = false;
  // while (!found){
  //   // Serial.println("Finding IMU...");
  //   // Initialize the ICM-20948
  //   // If the DMP is enabled, .begin performs a minimal startup. We need to configure the sample mode etc. manually.
    
  //   if (myICM.status != ICM_20948_Stat_Ok) {

  //     myICM.begin(WIRE_PORT, AD0_VAL);
  //     // Serial.println("I got here");

  //     sensorDataString = "ICM20948 NOT FOUND";
  //     // Serial.println("IMU NOT FOUND");

  //     // Send the sensor data through UDP
  //     Udp.beginPacket("10.42.0.1", 2390); // Replace with the Python code IP and port
  //     Udp.write(sensorDataString.c_str());
  //     Udp.endPacket();
  //     delay(1000);
  //   } 
  //   else{
  //     found = true;
  //     // Serial.println("IMU FOUND");
  //     sensorDataString = "ICM20948 has been FOUND";
  //     // Serial.println("IMU NOT FOUND");

  //     // Send the sensor data through UDP
  //     Udp.beginPacket("10.42.0.1", 2390); // Replace with the Python code IP and port
  //     Udp.write(sensorDataString.c_str());
  //     Udp.endPacket();
  //   }
  // }

  // bool initialized = false;
  // while(!initialized){
  //   Serial.println("Initializing IMU...");
  //   bool success = true; // Use success to show if the DMP configuration was successful
  //   Serial.println(success);
  //   // Initialize the DMP. initializeDMP is a weak function. You can overwrite it if you want to e.g. to change the sample rate
  //   while (!(myICM.initializeDMP() == ICM_20948_Stat_Ok)) {
  //     Serial.println(myICM.statusString());
  //     Serial.println(ICM_20948_Stat_Ok);
  //     Serial.println(myICM.initializeDMP());
  //     delay(500);
  //   }
  //   // Enable the DMP orientation sensor
  //   success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok);
  //   success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat9, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  //   Serial.println(success);
  //   // Enable the FIFO
  //   success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);
  //   Serial.println(success);
  //   // Enable the DMP
  //   success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);
  //   Serial.println(success);
  //   // Reset DMP
  //   success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);
  //   Serial.println(success);
  //   // Reset FIFO
  //   success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);
  //   Serial.println(success);
  //   // Check success
  //   if (!success){
  //     sensorDataString ="ICM20948 NOT INITIALIZED";
  //     Udp.beginPacket("10.42.0.1", 2390); // Replace with the Python code IP and port
  //     Udp.write(sensorDataString.c_str());
  //     Udp.endPacket();
  //   }
  //   else{
  //     initialized = true;
  //     Serial.println("IMU Initialized.");
  //     sensorDataString ="ICM20948 has been INITIALIZED";
  //     Udp.beginPacket("10.42.0.1", 2390); // Replace with the Python code IP and port
  //     Udp.write(sensorDataString.c_str());
  //     Udp.endPacket();
  //   }
  // }
  
  ////////////////////////
  //Configuration MPR121//
  ////////////////////////

  // 1.Check for MPR121
  // Default address is 0x5A, if tied to 3.3V its 0x5B
  // If tied to SDA its 0x5C and if SCL then 0x5D
  if (!cap.begin(0x5A)) {
    while (1)
      Serial.println("MPR121 NOT FOUND");
      sensorDataString ="MPR121 NOT FOUND";
      Udp.beginPacket("10.42.0.1", 2390); // Replace with the Python code IP and port
      Udp.write(sensorDataString.c_str());
      Udp.endPacket();
  }
  Serial.println("Configuring MPR121...");
  // 2. configure the settings
  if (configMode) {
    // run the autoconfiguration
    autoconfig_init();    
  } else {
      // disable autoconfig and use the current and time settings above
      config_with_settings(cap);
  }
}

void loop() {
  /////////////////////
  //Retrieve ICM Data//
  /////////////////////

  // icm_20948_DMP_data_t data;
  // myICM.readDMPdataFromFIFO(&data);

  // if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)){ // Was valid data available?
  //   if ((data.header & DMP_header_bitmap_Quat9) > 0){ // We have asked for orientation data so we should receive Quat9
  //     // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
  //     // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
  //     // The quaternion data is scaled by 2^30.
  //     //SERIAL_PORT.printf("Quat9 data is: Q1:%ld Q2:%ld Q3:%ld Accuracy:%d\r\n", data.Quat9.Data.Q1, data.Quat9.Data.Q2, data.Quat9.Data.Q3, data.Quat9.Data.Accuracy);
  //     // Scale to +/- 1

  //     q[1] = ((double)data.Quat9.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
  //     q[2] = ((double)data.Quat9.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
  //     q[3] = ((double)data.Quat9.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
  //     q[0] = sqrt(1.0 - ((q[1] * q[1]) + (q[2] * q[2]) + (q[3] * q[3])));
  //   }
  // }
  // if (myICM.status != ICM_20948_Stat_FIFOMoreDataAvail){ // If more data is available then we should read it right away - and not delay
  //   delay(1);
  // }

  ////////////////////////
  //Retrieve MPR121 Data//
  ////////////////////////

  if(!configMode){
    // 4.3 Read from MPR121
    for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
      int mpr_given = cap.filteredData(sensor);
      capacitance[sensor]= chargeCurrent*chargeTime*1024.0/mpr_given/3.3;
    }
  }

  // re-map the capacitance data because Will did a dum-dum when designing the motherboard
  // delete this when you do a better job with the motherboard design
  temp[0] = capacitance[3];
  temp[1] = capacitance[0];
  temp[2] = capacitance[1];
  temp[3] = capacitance[2];
  for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
    capacitance[sensor] = temp[sensor];
  }


  String q0String = String(q[0], 5); // Convert q[0] to a string with 5 decimal places
  String q1String = String(q[1], 5); // Convert q[1] to a string with 5 decimal places
  String q2String = String(q[2], 5); // Convert q[2] to a string with 5 decimal places
  String q3String = String(q[3], 5); // Convert q[3] to a string with 5 decimal places

  sensorDataString = String(N_Arduino) + " " 
                    + q0String.substring(0, 7) + " " 
                    + q1String.substring(0, 7) + " "
                    + q2String.substring(0, 7) + " "
                    + q3String.substring(0, 7) + " "
                    + String(capacitance[0]) + " " 
                    + String(capacitance[1]) + " " 
                    + String(capacitance[2]);

  //////////////////////
  //Retrieve motor PWM//
  //////////////////////

  int packetSize = Udp.parsePacket();

  if (packetSize) {
    // Receive data into packetBuffer
    Udp.read(packetBuffer, 255);
    double motorPWM[Nb_motors];
    int numDoubles = 0;
    int skippedValues = 0;
    char* endPtr; // Used by strtod to check for parsing errors
    // Parse the string and convert doubles to an array
    char* token = strtok((char*)packetBuffer, " ");
    while (token != NULL && numDoubles < Nb_motors) {
      if (skippedValues < OFFSET) {
        skippedValues++;
      } else {
        motorPWM[numDoubles++] = strtod(token, &endPtr);
      }
      token = strtok(NULL, " ");
    }
    motor1.drive((int) (255.0*motorPWM[N_motor1]/99.0));
    motor2.drive((int) (255.0*motorPWM[N_motor2]/99.0));
  }
  // Send the sensor data through UDP
  Udp.beginPacket("10.42.0.1", 2390); // Replace with the Python code IP and port
  Udp.write(sensorDataString.c_str());
  Udp.endPacket();

}

  ////////////////////
  //Helper functions//
  ////////////////////
void autoconfig_init() {
  // Auto-configure Charge-time and charge-current for MPR121
  cap.writeRegister(MPR121_AUTOCONFIG0, 0x00001011);

  // Specify the search boundaries and target for the auto-configuration. correct values for Vdd = 3.3V are 200, 180, 130.
  cap.writeRegister(MPR121_UPLIMIT, 200);     // ((Vdd - 0.7)/Vdd) * 256
  cap.writeRegister(MPR121_TARGETLIMIT, 180); // UPLIMIT * 0.9
  cap.writeRegister(MPR121_LOWLIMIT, 130);    // UPLIMIT * 0.65

  // wait for autoconfiguration to finish
  delay(500);
}

void config_with_settings(Adafruit_MPR121 board) {
  // Disable auto-configuration to specify I and T manually.
  board.writeRegister(MPR121_AUTOCONFIG0, 0x00); // Disable Auto-configuration
  // Global settings
  board.writeRegister(MPR121_CONFIG1, byte(chargeCurrent)); // [7-6] Default first filter iterations = 6 (default); [5-0] CDC = X ua (000001 = 1 ua, 111111 = 63 ua)
  board.writeRegister(MPR121_CONFIG2, time2Reg(chargeTime)); // [7-5] CDT 0.5 us (default) to 32 us. EX: 001 = 0.5 us; 020 = 1 us; 111 = 32 us
}

// this takes in a charging time and outputs the correct GLOBAL configuration register setting
byte time2Reg(float T) {
  // the only valid floating-point value
  if (T == 0.5) {
    return 0b00100000;
  } else {
    // the (inverse) formula from the data sheet
    byte n = log(2 * T) / log(2) + 1;
    return n << 5;
  }
}