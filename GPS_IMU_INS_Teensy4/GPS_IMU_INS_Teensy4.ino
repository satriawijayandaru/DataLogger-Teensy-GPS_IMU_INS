#include <TinyGPS++.h>

#define ser1 Serial
#define GPS Serial2
#define IMU Serial3

int debugEn = 1;

int led = 13;
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
float liniarAccelX, liniarAccelY, liniarAccelZ;
float accelX, accelY, accelZ;
float compassX, compassY, compassZ;
float G = 9.80665;
int dataReq = 0;
int imuDataType;

//INS Calculation
float gx, gy, gz;
float x, y, z;
float k = 0.98;

//GPS
float lat, lon, alt;
static const uint32_t GPSBaud = 9600;
char coordinateLat[10];
char coordinateLng[10];
char gpsAlt[5];
char latStr[10];
char speedGPS[10];
TinyGPSPlus gps;
double prevLat, prevLon;
double currLat, currLon;
double totalDistance = 0;

void setup() {
  pinMode(led, OUTPUT);
  GPS.begin(9600);
  IMU.begin(115200);
}

void loop() {
  getImu(20);
  getGPSData();
}

void insCalc() {
  unsigned long currentMillis = millis();
  dt = (currentMillis - previousMillis) / 1000.0;

  gx = liniarAccelX * 9.8;
  gy = liniarAccelY * 9.8;
  gz = liniarAccelZ * 9.8;

  x = k * (x + gx * dt) + (1 - k) * lat;
  y = k * (y + gy * dt) + (1 - k) * lon;
  z = k * (z + gz * dt) + (1 - k) * alt;

  ser1.print("DT = ");
  ser1.println(dt);
  ser1.print("lat INS = ");
  ser1.println(x, 4);
  ser1.print("lng INS = ");
  ser1.println(y, 4);
  ser1.print("alt INS = ");
  ser1.println(z, 4);
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
    dataReq++;
    if (dataReq == 1) {
      IMU.println(":41");
      imuDataType = 41;
    }
    if (dataReq == 2) {
      IMU.println(":34");
      imuDataType = 34;
    }
    if (dataReq == 3) {
      IMU.println(":40");
      imuDataType = 40;
      dataReq = 0;
    }
  }
}

void dataPreparation() {
  digitalWrite(led, HIGH);
  if (imuDataType == 41) {
    liniarAccelX   = incomingCSV[2].toFloat() / G;
    liniarAccelY   = incomingCSV[0].toFloat() / G;
    liniarAccelZ   = incomingCSV[1].toFloat() / G;
  }
  if (imuDataType == 34) {
    accelX = incomingCSV[2].toFloat();
    accelY = incomingCSV[0].toFloat();
    accelZ = incomingCSV[1].toFloat();
  }
  if (imuDataType == 40) {
    compassX = incomingCSV[2].toFloat();
    compassY = incomingCSV[0].toFloat();
    compassZ = incomingCSV[1].toFloat();
  }
  float heading = atan2(compassY, compassX) * 180.0 / PI;
  float speed = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);
  digitalWrite(led, LOW);
  ser1.println("LinearAc,Acel,Magn");
  ser1.print("X = "); ser1.print(liniarAccelX); ser1.print(","); ser1.print(accelX); ser1.print(","); ser1.print(compassX); ser1.println();
  ser1.print("Y = "); ser1.print(liniarAccelY); ser1.print(","); ser1.print(accelY); ser1.print(","); ser1.print(compassY); ser1.println();
  ser1.print("Z = "); ser1.print(liniarAccelZ); ser1.print(","); ser1.print(accelZ); ser1.print(","); ser1.print(compassZ); ser1.println();
  ser1.print("Heading = "); ser1.println(heading);
  ser1.print("Speed   = "); ser1.println(speed);

  

  insCalc();
  ser1.println();
}

float radToDeg(float data) {
  return data * (180 / PI);
}

void getGPSData() {
  while (GPS.available() > 0) {
    gps.encode(GPS.read());
    if (gps.location.isUpdated()) {
      dtostrf(gps.location.lat(), 7, 7, coordinateLat);
      dtostrf(gps.location.lng(), 7, 7, coordinateLng);
      dtostrf(gps.altitude.meters(), 2, 2, gpsAlt);
      dtostrf(gps.speed.mps(), 5, 5, speedGPS);

      currLat = gps.location.lat();
      currLon = gps.location.lng();
//       calculateDistance();


      if (debugEn == 1) {
        Serial.print("Latitude  = "); Serial.println(gps.location.lat(), 9);
        Serial.print("Longitude = "); Serial.println(gps.location.lng(), 9);
        Serial.print("Altitude  = "); Serial.println(gpsAlt);
        Serial.print("Speed m/s = "); Serial.println(speedGPS);
        Serial.print("Speed kph = "); Serial.println(gps.speed.kmph());
        Serial.print("Distance  = "); Serial.println(totalDistance);
        Serial.println("=======================================");
      }
    }
  }
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
