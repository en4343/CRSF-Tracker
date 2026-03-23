/*================================================================================================= 
    Antenna Tracker - "Community Edition" (Final Flight Ready)
=================================================================================================*/

#include <Arduino.h>
#include "config.h"                      
#include <Wire.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include <Preferences.h> 

// --- SENSOR & DISPLAY LIBRARIES ---
#if USE_COMPASS
  #include <Adafruit_BNO08x.h>
#endif
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> 

// =======================================================================================
// SYSTEM CONFIGURATION
// =======================================================================================
#define OLED_RESET -1 

// =======================================================================================
// GLOBAL STATE VARIABLES
// =======================================================================================

// --- Telemetry (Drone) Data ---
volatile float droneLat = 0;
volatile float droneLon = 0;
volatile float droneAlt = 0;
volatile int droneSats = 0;
volatile bool linkConnected = false;
unsigned long lastPacketTime = 0;
int currentChannel = 1;
bool channelLocked = false;

// --- Ground Station (Box) Data ---
float boxLat = 0;
float boxLon = 0;
float boxAlt = 0;
bool boxGPSFixed = false;
int boxSats = 0;
float trackerHeading = 0;
bool compassGood = false;
bool gpsGood = false;

// --- Calibration & Tracking State ---
int panOffset = 0; 
float altOffset = 0; 
bool homeEstablished = false; 
bool calibrationDone = false; 
int trimOffsetVal = 0; 
int currentTrim = 0;   

// --- Hardware Objects ---
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);
SFE_UBLOX_GNSS myGNSS;
HardwareSerial gpsSerial(2);

#if USE_COMPASS
  Adafruit_BNO08x bno08x(BNO_RST);
  sh2_SensorValue_t sensorValue;
#endif

Preferences preferences; 
Servo azServo;            
Servo elServo;   
bool servosAwake = false;
uint8_t ledState = LOW; 
uint32_t millisLED = 0;

#ifndef CRSF_FRAMETYPE_GPS
  #define CRSF_FRAMETYPE_GPS 0x02
#endif

// --- Geographic Structures ---
struct Location { float lat; float lon; float alt; float hdg; float alt_ag; };
struct Location hom = { 0,0,0,0,0};   
struct Location cur = { 0,0,0,0,0};   
struct Vector { float az; float el; int32_t dist; };
struct Vector hc_vector  = { 90, 0, 0};

// FORWARD DECLARATIONS
void pointServos(uint16_t az, uint16_t el);
void getAzEl(struct Location &home, struct Location &current);
void PerformCalibration();
void ClearFailsafe();
void CheckFailsafe(); 
void BlinkLed(uint16_t period);
void ServiceTheStatusLed();
void ReadLocalGPS(); 
void ReadCompass();
void WakeServos();

// =======================================================================================
// LOGGING & DISPLAY
// =======================================================================================
void LogScreenPrintln(String s, String s2 = "") {
  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE); 
  display.println("CRSF Tracker");
  display.println("----------------");
  
  display.setTextSize(2);
  display.println(s);
  if (s2 != "") { 
      display.setTextSize(1); 
      display.println(s2); 
  }
  
  display.setTextSize(1);
  
  display.setCursor(0, 45);
  display.print("Tracker: "); display.print(boxSats);
  if(boxGPSFixed) display.print(" [FIX]");

  display.setCursor(0, 55);
  display.print("Drone:   "); display.print(droneSats);
  
  display.display();
}

void UpdateDisplay(int targetAz, int trimVal) {
  static unsigned long timer = 0;
  if (millis() - timer > 200) {
    timer = millis();
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    float rawDiff = cur.alt - hom.alt;
    float trueDiff = rawDiff - altOffset; 
    String mode = "REL"; 
    if (trueDiff < -100) { trueDiff = cur.alt; mode = "AGL"; }
    
    display.print("Alt:"); display.print(trueDiff, 0);
    display.print("m ["); display.print(mode); display.println("]");

    display.print("Dist:"); display.print(hc_vector.dist); display.println("m");

    int tiltAngle = hc_vector.el; 
    if (tiltAngle < 0) tiltAngle = 0;
    display.print("TiltCMD: "); display.println(tiltAngle); 

    float trueHeading = trackerHeading + panOffset;
    while (trueHeading >= 360.0) trueHeading -= 360.0;
    while (trueHeading < 0.0) trueHeading += 360.0;

    display.print("Hdg:"); display.print(trueHeading, 0);
    
    if (hc_vector.dist < MIN_TRACKING_DIST) {
        display.println(" [LOCK]");
    } else {
        display.print(" Trim:"); 
        if(trimVal > 0) display.print("+");
        else if (trimVal == 0 && !USE_TRIM_KNOB) display.print("OFF");
        display.print(trimVal);
        display.println("");
    }
    display.display();
  }
}

// =======================================================================================
// HARDWARE CONTROL
// =======================================================================================
void WakeServos() {
    if (!servosAwake) {
        azServo.writeMicroseconds(PAN_CENTER_PWM);
        elServo.writeMicroseconds(TILT_HORIZON_PWM);
        
        azServo.attach(azPWM_Pin, minAzPWM, maxAzPWM); 
        elServo.attach(elPWM_Pin, minElPWM, maxElPWM);
        
        servosAwake = true;
    }
}

// =======================================================================================
// ESP-NOW TELEMETRY RECEIVER (Dynamic V3 & V4 Parser)
// =======================================================================================
void OnDataRecv(const esp_now_recv_info_t * info, const uint8_t *data, int data_len) {
  int offset = -1;
  
  for (int i = 4; i < 15; i++) {
      if (data[i] == CRSF_FRAMETYPE_GPS && data_len >= i + 16) { 
          offset = i;
          break; 
      }
  }

  if (offset != -1) {
    lastPacketTime = millis();
    linkConnected = true;
    
    if (!channelLocked) {
      channelLocked = true;
      Serial.println("ELRS LINK OK - LOCKED ON PACKET!");
    }

    long latitudeBytes = 0; long longitudeBytes = 0;
    for (int i = 0; i < 4; ++i) latitudeBytes |= (data[(offset + 4) - i] << (i * 8));
    for (int i = 0; i < 4; ++i) longitudeBytes |= (data[(offset + 8) - i] << (i * 8));

    droneLat = latitudeBytes / 10000000.0;
    droneLon = longitudeBytes / 10000000.0;
    
    uint16_t altitudeRaw = (data[offset + 13] << 8) | data[offset + 14];
    int altitude1 = altitudeRaw - 1000;
    droneAlt = (altitude1 > 32767 || altitude1 < -32768) ? (altitude1 - 4294967296) : altitude1;
    
    droneSats = data[offset + 15];
  }
}

// =======================================================================================
// SENSORS (GPS & COMPASS)
// =======================================================================================
void ReadLocalGPS() {
  if (myGNSS.getGnssFixOk()) {
      boxGPSFixed = true;
      boxSats = myGNSS.getSIV();
      boxLat = myGNSS.getLatitude() / 10000000.0;
      boxLon = myGNSS.getLongitude() / 10000000.0;
      
      if (!calibrationDone) {
          boxAlt = myGNSS.getAltitude() / 1000.0; 
          hom.alt = boxAlt;
      }
      hom.lat = boxLat;
      hom.lon = boxLon;
      homeEstablished = true;
  } else {
      boxGPSFixed = false;
      boxSats = myGNSS.getSIV(); 
  }
}

void ReadCompass() {
#if USE_COMPASS
    if (!compassGood) return;
    
    if (bno08x.wasReset()) bno08x.enableReport(SH2_ROTATION_VECTOR, 50000); 
    
    while (bno08x.getSensorEvent(&sensorValue)) {
        if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
            float qr = sensorValue.un.rotationVector.real;
            float qi = sensorValue.un.rotationVector.i;
            float qj = sensorValue.un.rotationVector.j;
            float qk = sensorValue.un.rotationVector.k;

            float sqr = sq(qr); float sqi = sq(qi);
            float sqj = sq(qj); float sqk = sq(qk);
            
            float siny_cosp = 2.0 * (qr * qk + qi * qj);
            float cosy_cosp = 1.0 - 2.0 * (sqj + sqk);
            trackerHeading = atan2(siny_cosp, cosy_cosp) * 180.0 / PI;

            if (trackerHeading < 0) trackerHeading += 360.0;

            trackerHeading = 360.0 - trackerHeading; 
            if (trackerHeading >= 360.0) trackerHeading -= 360.0;
        }
    }
#endif
}

// =======================================================================================
// BUTTON & CALIBRATION
// =======================================================================================
void checkHomeButton() {
    static unsigned long pressStart = 0;
    static bool isPressed = false;

    if (digitalRead(PIN_RESET_HOME) == LOW) { 
        if (!isPressed) {
            isPressed = true;
            pressStart = millis();
        }
        if (millis() - pressStart > 5000) {
            display.clearDisplay(); display.setCursor(0,0);
            display.setTextSize(2); display.setTextColor(SSD1306_WHITE); 
            display.println("RELEASE TO"); display.println("  CLEAR"); display.display();
        }
    } 
    else {
        if (isPressed) {
            unsigned long duration = millis() - pressStart;
            isPressed = false;
            
            if (duration > 5000) ClearFailsafe();
            else if (duration > 1000) {
                if (!homeEstablished || boxSats < MIN_SATS) LogScreenPrintln("Tracker GPS", "Need Sats");
                else if (!linkConnected) LogScreenPrintln("No Link", "Cannot Cal");
                else if (droneSats < MIN_SATS) LogScreenPrintln("Drone GPS", "Need Sats");
                else PerformCalibration();
            }
        }
    }
}

void ClearFailsafe() {
    preferences.begin("anttrack", false);
    preferences.clear(); 
    preferences.end();
    panOffset = 0; 
    calibrationDone = false;
    LogScreenPrintln("FAILSAFE", "CLEARED");
    delay(2000);
}

void PerformCalibration() {
    LogScreenPrintln("Calibrating...");
    cur.lat = droneLat; cur.lon = droneLon;
    getAzEl(hom, cur); 
    
    #if USE_COMPASS
        panOffset = hc_vector.az - trackerHeading;
    #else
        trackerHeading = 0; 
        panOffset = hc_vector.az;
    #endif

    while (panOffset < 0) panOffset += 360;
    while (panOffset >= 360) panOffset -= 360;
    
    altOffset = cur.alt - hom.alt;

    #if USE_TRIM_KNOB
        long total = 0;
        for(int i=0; i<10; i++) {
            total += analogRead(TRIM_POT_PIN);
            delay(10);
        }
        trimOffsetVal = total / 10;
    #else
        trimOffsetVal = 0;
        currentTrim = 0;
    #endif
    
    calibrationDone = true;
    
// Save calibration to non-volatile memory
    uint32_t currentEpoch = myGNSS.getUnixEpoch(); // Grab live satellite time
    
    preferences.begin("anttrack", false);
    preferences.putFloat("offset", (float)panOffset);
    preferences.putFloat("altOffset", (float)altOffset); 
    preferences.putFloat("lat", hom.lat);
    preferences.putFloat("lon", hom.lon);
    preferences.putUInt("epoch", currentEpoch); 
    preferences.end();
    
    WakeServos();
    LogScreenPrintln("Locked & Zeroed!", "Tracker Ready");
}

void CheckFailsafe() {
    if (!boxGPSFixed) return; // Must wait for ground GPS to get live time/location

    preferences.begin("anttrack", true);
    float savedLat = preferences.getFloat("lat", 0);
    float savedLon = preferences.getFloat("lon", 0);
    float savedOffset = preferences.getFloat("offset", 0);
    uint32_t savedEpoch = preferences.getUInt("epoch", 0);
    preferences.end();
    
    if (savedLat == 0 || savedEpoch == 0) return; // No previous data exists

    uint32_t currentEpoch = myGNSS.getUnixEpoch();
    if (currentEpoch == 0) return; // Satellites haven't broadcasted time yet

    // Check if the failsafe has expired
    if ((currentEpoch - savedEpoch) > (FAILSAFE_TIMEOUT * 60)) {
        return; // Memory is too old, force a new manual calibration
    }

    struct Location savedLoc; savedLoc.lat = savedLat; savedLoc.lon = savedLon;
    float distToSavedHome = getDist(hom, savedLoc); 

    // If tracker rebooted within 100m of old home, assume power bump and restore
    if (distToSavedHome < 100) { 
        getAzEl(hom, cur); 
        if (hc_vector.dist > MIN_TRACKING_DIST) { 
            panOffset = (int)savedOffset;
            calibrationDone = true;
            WakeServos();
            LogScreenPrintln("FAILSAFE!", "Restored Cal");
            delay(2000);
        }
    }
}

// =======================================================================================
// SETUP
// =======================================================================================
void setup() {
  Serial.begin(115200);
  delay(100); 
  
  Wire.begin(SDA, SCL);  
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) Serial.println(F("SSD1306 allocation failed"));
  display.clearDisplay(); display.display();

  #if USE_COMPASS
    pinMode(BNO_INT, INPUT_PULLUP);
  #endif
  
  pinMode(PIN_RESET_HOME, INPUT_PULLUP);
  pinMode(StatusLed, OUTPUT); 
  
  // --- INIT COMPASS ---
  #if USE_COMPASS
      LogScreenPrintln("Compass Init...");
      if (!bno08x.begin_I2C(0x4A, &Wire)) {
          LogScreenPrintln("Compass FAIL", "Check Wires!");
          delay(3000);
      } else {
          bno08x.enableReport(SH2_ROTATION_VECTOR, 50000); 
          compassGood = true;
          LogScreenPrintln("Compass OK!");
          delay(1000);
      }
  #else
      LogScreenPrintln("Compass OFF", "Visual Cal Mode");
      delay(1500);
  #endif

  // --- INIT GPS ---
  LogScreenPrintln("GPS Init...");
  gpsSerial.begin(9600, SERIAL_8N1, gps_rxPin, gps_txPin);
  
  if (myGNSS.begin(gpsSerial) == false) {
      gpsSerial.updateBaudRate(38400);
      if (myGNSS.begin(gpsSerial) == false) {
          gpsSerial.updateBaudRate(115200);
          if (myGNSS.begin(gpsSerial) == false) {
              LogScreenPrintln("GPS FAIL", "Check Wires"); delay(2000);
          } else { LogScreenPrintln("GPS OK", "115200bd"); }
      } else { LogScreenPrintln("GPS OK", "38400bd"); }
  } else { LogScreenPrintln("GPS OK", "9600bd"); }

  myGNSS.setI2COutput(COM_TYPE_UBX); 
  myGNSS.setNavigationFrequency(5);  
  
  // --- INIT ESP-NOW WIFI ---
  WiFi.mode(WIFI_STA);
  esp_wifi_set_mac(WIFI_IF_STA, BINDING_MAC);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) ESP.restart();
  esp_now_register_recv_cb(OnDataRecv);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  
  // --- FINISH BOOT ---
  #if USE_TRIM_KNOB
      LogScreenPrintln("Zeroing Knob...");
      delay(500); 
      long total = 0;
      for(int i=0; i<10; i++) {
          total += analogRead(TRIM_POT_PIN);
          delay(10);
      }
      trimOffsetVal = total / 10; 
  #endif
  
  LogScreenPrintln("Ready!");
  delay(500);
}

// =======================================================================================
// MAIN LOOP
// =======================================================================================
void loop() {            
  ReadCompass();
  ReadLocalGPS();

  if (millis() - lastPacketTime > 2000) {
    channelLocked = false; linkConnected = false; 
    currentChannel++; if (currentChannel > 13) currentChannel = 1;
    esp_wifi_set_channel(currentChannel, WIFI_SECOND_CHAN_NONE);
    delay(10); 
  }

  checkHomeButton();

  if (linkConnected) {
      cur.lat = droneLat; cur.lon = droneLon; cur.alt = droneAlt;
      cur.alt_ag = cur.alt - hom.alt; 
      gpsGood = true;       
  }

  // --- FLIGHT TRACKING MODE ---
  if (homeEstablished && calibrationDone && linkConnected) {
      getAzEl(hom, cur); 
      
      float targetAzBase = hc_vector.az - trackerHeading - panOffset;
      
      while (targetAzBase < 0) targetAzBase += 360;
      while (targetAzBase > 360) targetAzBase -= 360;
      
      #if USE_TRIM_KNOB
          int rawKnob = analogRead(TRIM_POT_PIN);
          int knobDiff = rawKnob - trimOffsetVal; 
          currentTrim = knobDiff / 30;

          if (currentTrim < -MAX_TRIM_ANGLE) currentTrim = -MAX_TRIM_ANGLE;
          if (currentTrim > MAX_TRIM_ANGLE)  currentTrim = MAX_TRIM_ANGLE;
      #else
          currentTrim = 0; // Locks trim dead-center if hardware isn't installed
      #endif

      static unsigned long debugTimer = 0;
      if (millis() - debugTimer > 200) { 
         debugTimer = millis();
         UpdateDisplay(0, currentTrim); 
      }

      if ( (hc_vector.dist >= MIN_TRACKING_DIST) || ((int)cur.alt_ag >= MIN_TRACKING_ALT) ) {
          long finalAzLong = (long)targetAzBase + currentTrim;

          while (finalAzLong < 0) finalAzLong += 360;
          while (finalAzLong >= 360) finalAzLong -= 360;
          
          int tiltAngle = hc_vector.el; 
          if (tiltAngle < 0) tiltAngle = 0;
          if (tiltAngle > maxEl) tiltAngle = maxEl; 
          
          pointServos((uint16_t)finalAzLong, tiltAngle);
      }
  } 
  
  // --- PRE-FLIGHT / WAITING MODE ---
  else if (!homeEstablished || boxSats < MIN_SATS) {
     static unsigned long warnTimer = 0;
     if (millis() - warnTimer > 2000) {
        warnTimer = millis();
        LogScreenPrintln("Tracker: " + String(boxSats) + "/" + String(MIN_SATS)); 
     }
  }
  else if (!calibrationDone) {
     if (linkConnected) CheckFailsafe(); 
     static unsigned long warnTimer = 0;
     if (millis() - warnTimer > 1000) { 
        warnTimer = millis();
        if (!linkConnected) LogScreenPrintln("No Link");
        else if (droneSats < MIN_SATS) LogScreenPrintln("Drone: " + String(droneSats) + "/" + String(MIN_SATS)); 
        else LogScreenPrintln("Ready", "Hold 1 sec");
     }
  }
  
  ServiceTheStatusLed();
} 

// =======================================================================================
// MATH & NAVIGATION
// =======================================================================================

float getDist(struct Location &a, struct Location &b) {
  float dLon = (b.lon - a.lon) * PI / 180.0;
  float lat1 = a.lat * PI / 180.0;
  float lat2 = b.lat * PI / 180.0;
  float dLat = (b.lat - a.lat) * PI / 180.0;
  float x = sin(dLat/2) * sin(dLat/2) + sin(dLon/2) * sin(dLon/2) * cos(lat1) * cos(lat2);
  float c = 2 * atan2(sqrt(x), sqrt(1-x));
  return 6371000.0 * c; 
}

void getAzEl(struct Location &hom, struct Location &cur) {
  float dLon = (cur.lon - hom.lon) * PI / 180.0;
  float lat1 = hom.lat * PI / 180.0;
  float lat2 = cur.lat * PI / 180.0;

  float y = sin(dLon) * cos(lat2);
  float x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
  float az = atan2(y, x) * 180.0 / PI;
  if (az < 0) az += 360;
  hc_vector.az = az;

  float dLat = (cur.lat - hom.lat) * PI / 180.0;
  float a = sin(dLat/2) * sin(dLat/2) + sin(dLon/2) * sin(dLon/2) * cos(lat1) * cos(lat2);
  float c = 2 * atan2(sqrt(a), sqrt(1-a));
  hc_vector.dist = 6371000 * c; 

  float rawDiff = cur.alt - hom.alt;
  float diff = rawDiff - altOffset;
  if (diff < -100) diff = cur.alt; 
  if (diff < 0) diff = 0;
  hc_vector.el = atan2(diff, hc_vector.dist) * 180.0 / PI;
}

void pointServos(uint16_t az, uint16_t el) {
  int targetAz = az + PAN_TRIM;
  if (targetAz < 0) targetAz += 360;
  if (targetAz >= 360) targetAz -= 360;

  float servoAngle; 
  if (REVERSE_PAN) {
      if (targetAz <= 90) servoAngle = 90.0 - targetAz;
      else if (targetAz >= 180) servoAngle = 90.0 + (360.0 - targetAz);
      else servoAngle = 0; 
  } 
  else {
      servoAngle = targetAz + SERVO_CENTER; 
      if (servoAngle >= 360) servoAngle -= 360; 
  }

  servoAngle = constrain(servoAngle, 0, 270);
  float usPerDegree = (maxAzPWM - minAzPWM) / 270.0;
  
  float deviation = servoAngle - 90.0;
  int targetPanPWM = PAN_CENTER_PWM + (int)(deviation * usPerDegree);

  if (el < 0) el = 0;
  if (el > maxEl) el = maxEl; 
  
  int targetTiltPWM = map(el, 0, 90, TILT_HORIZON_PWM, 1000); 
  targetTiltPWM = constrain(targetTiltPWM, 1000, 1600);

  static float currentPanPWM = targetPanPWM;   
  static float currentTiltPWM = targetTiltPWM; 
  static unsigned long lastMoveTime = 0;

  if (millis() - lastMoveTime > 20) {
      lastMoveTime = millis();
      
      currentPanPWM += (targetPanPWM - currentPanPWM) * SERVO_SPEED;
      currentTiltPWM += (targetTiltPWM - currentTiltPWM) * SERVO_SPEED;

      azServo.writeMicroseconds((int)currentPanPWM);
      elServo.writeMicroseconds((int)currentTiltPWM);
  }
}

// =======================================================================================
// STATUS LED HANDLERS
// =======================================================================================
void ServiceTheStatusLed() {
  if (linkConnected) {
    if (calibrationDone) digitalWrite(StatusLed, HIGH); 
    else BlinkLed(100); 
  } else {
    BlinkLed(1000);     
  }
}

void BlinkLed(uint16_t period) {
  uint32_t cMillis = millis();
  if (cMillis - millisLED >= period) {
    millisLED = cMillis;
    ledState = !ledState;
    digitalWrite(StatusLed, ledState);
  }
}