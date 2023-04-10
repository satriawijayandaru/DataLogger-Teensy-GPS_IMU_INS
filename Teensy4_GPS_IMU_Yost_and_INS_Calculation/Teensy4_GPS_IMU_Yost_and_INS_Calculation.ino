#include <TinyGPS++.h>

#define ser1 Serial
#define GPS Serial2
#define IMU Serial3

double dt;
unsigned long previousMillis;

//IMU
String incomingCSV[20];
String incomingString;
int currentCSVIndex = 0;
int maxCSVIndex;
char incomingByte;
int incomingInt;
unsigned long previousMillisIMU = 0;
float imuX, imuY, imuZ;
float G = 9.80665;

//INS Calculation
float gx, gy, gz;
float x, y, z;
float k = 0.98;

//GPS
float lat, lon, alt;

void setup() {
  GPS.begin(115200);
  IMU.begin(115200);
}

void loop() {
  getImu(100);
}

void insCalc() {
  unsigned long currentMillis = millis();
  dt = (currentMillis - previousMillis) / 1000.0;

  gx = imuX * 9.8;
  gy = imuY * 9.8;
  gz = imuZ * 9.8;

  x = k * (x + gx * dt) + (1 - k) * lat;
  y = k * (y + gy * dt) + (1 - k) * lon;
  z = k * (z + gz * dt) + (1 - k) * alt;
  
  ser1.print("DT = ");
  ser1.println(dt);
  ser1.print("lat INS = ");
  ser1.println(x);
  ser1.print("lng INS = ");
  ser1.println(y);
  ser1.print("alt INS = ");
  ser1.println(z);
  ser1.println();
  previousMillis = currentMillis;
}

void getImu(int interval) {
  if (IMU.available()) {
    serialIN(IMU.read(), 0);
  }
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillisIMU >= interval) {
    previousMillisIMU = currentMillis;
//    IMU.println(":34");
    IMU.println(":41");
  }
}

void dataPreparation() {
  imuX   = incomingCSV[2].toFloat() / G;
  imuY   = incomingCSV[0].toFloat() / G;
  imuZ   = incomingCSV[1].toFloat() / G;
  //  imuX   = radToDeg(incomingCSV[2].toFloat());
  //  imuY   = radToDeg(incomingCSV[0].toFloat());
  //  imuZ   = radToDeg(incomingCSV[1].toFloat());

  Serial.print("X = ");
  Serial.println(imuX, 3);
  Serial.print("Y = ");
  Serial.println(imuY, 3);
  Serial.print("Z = ");
  Serial.println(imuZ, 3);
  insCalc();
}

float radToDeg(float data) {
  return data * (180 / PI);
}

void serialIN(char incomingSerial, bool debugMode) {

  incomingByte = incomingSerial;
  incomingString += incomingByte;
  if (incomingByte == ',') {
    int str_len = incomingString.length() - 1;
    incomingCSV[currentCSVIndex] = incomingString.substring(0, str_len);
    if (debugMode == 1) {
      ser1.print("Incoming ");
      ser1.print(currentCSVIndex);
      ser1.print(" = ");
      ser1.println(incomingCSV[currentCSVIndex].toFloat() * 90.0);
    }
    currentCSVIndex++;
    incomingString = "";
  }
  if (incomingByte == '\n') {
    int str_len = incomingString.length() - 1;
    incomingCSV[currentCSVIndex] = incomingString.substring(0, str_len);
    maxCSVIndex = currentCSVIndex;
    if (debugMode == 1) {
      ser1.print("Incoming ");
      ser1.print(currentCSVIndex);
      ser1.print(" = ");
      ser1.println(incomingCSV[currentCSVIndex].toFloat() * 90.0);
      ser1.print("max Index  = ");
      ser1.println(maxCSVIndex);
      ser1.println();
    }
    dataPreparation();
    incomingString = "";
    currentCSVIndex = 0;
  }
}
