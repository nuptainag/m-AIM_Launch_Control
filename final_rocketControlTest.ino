#include <Wire.h>
#include <Servo.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_Sensor.h>
#include <MS5611.h>
#include <math.h>
#include <SD.h>
#include <SPI.h>

// --- Hardware Settings ---
#define BNO08X_RESET 15
Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

MS5611 ms5611(0x77);
float groundAltitude = 0.0;

Servo servo1, servo2, servo3, servo4;
const int PIN_SERVO1 = 2; // North Fin
const int PIN_SERVO2 = 5; // East Fin
const int PIN_SERVO3 = 4; // South Fin
const int PIN_SERVO4 = 3; // West Fin

const int S1_OFFSET = 0;
const int S2_OFFSET = 0;
const int S3_OFFSET = 0;
const int S4_OFFSET = 0;

// --- Tuning Parameters (PID) ---
float Kp = 2.0;
float Ki = 0.05;
float Kd = 0.5;

const float MAX_FIN_DEFLECTION = 25.0;

// --- State Variables & Flight Phasing ---
enum FlightPhase { PAD, BOOST, NAVIGATE };
FlightPhase currentPhase = PAD;

const float LAUNCH_THRESHOLD_G = 1.5;
unsigned long launchTime = 0;
const unsigned long BOOST_DURATION_MS = 3000;

// PID Memory
float integralPitch = 0, integralYaw = 0;
float lastErrorPitch = 0, lastErrorYaw = 0;

// Loop Timing
unsigned long lastInnerLoopTime = 0;
unsigned long lastOuterLoopTime = 0;
unsigned long lastLogTime = 0;
unsigned long lastFlushTime = 0; 

// Sensor Holding Variables
float rollRad = 0, pitchRad = 0, yawRad = 0;
float q_i = 0, q_j = 0, q_k = 0, q_r = 1.0;
float accX_body = 0, accY_body = 0, accZ_body = 0;

// --- IMU FAILSAFE VARIABLES ---
int consecutiveZeroCount = 0;
bool testModeActive = false;

// --- KALMAN FILTER VARIABLES ---
float kf_z = 0.0;
float kf_v = 0.1;
float P[2][2] = { { 1.0, 0.0 }, { 0.0, 1.0 } };
float Q_z = 0.01;
float Q_v = 0.1;
float R_baro = 2.0;

// --- 3D NAVIGATION VARIABLES (OUTER LOOP) ---
float launchLat = 39.007357;
float launchLon = -104.886216;
float currentLat = launchLat;
float currentLon = launchLon;

float targetLat = 39.007287;
float targetLon = -104.886367;
float targetAlt = 10;

float commandedPitchWorld = 0.0;
float commandedYawWorld = 0.0;

// Dead Reckoning Integration Variables
float velNorth_fps = 0.0;
float velEast_fps = 0.0;
float posNorth_ft = 0.0;
float posEast_ft = 0.0;

// --- DATA LOGGING VARIABLES ---
File flightLog;
int log_s1 = 90, log_s2 = 90, log_s3 = 90, log_s4 = 90;

void setup() {
  Serial.begin(115200);
  Serial.println("\n--- ROCKET BOOT SEQUENCE INITIATED ---");

  Wire.begin();
  Wire2.begin();
  Wire.setClock(400000);
  Wire2.setClock(100000);

  Serial.println("1. Initializing SD Card...");
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("   -> SD Card ERROR (Proceeding without logging)");
  } else {
    flightLog = SD.open("flight_log.csv", FILE_WRITE);
    if (flightLog) {
      flightLog.println("Time_ms,Phase,Lat,Lon,Alt_ft,Vel_fps,Pitch_deg,Yaw_deg,Roll_deg,CmdPitch,CmdYaw,S1,S2,S3,S4");
      flightLog.flush();
      Serial.println("   -> SD Card READY");
    }
  }

  Serial.println("2. Initializing MS5611 Barometer...");
  if (!ms5611.begin()) {
    Serial.println("   -> CRITICAL ERROR: MS5611 NOT DETECTED!");
    while (1) delay(10);
  }
  Serial.println("   -> MS5611 READY");

  Serial.println("3. Initializing BNO08x IMU on Wire2...");
  // Back to 0x4A based on your scanner results
  if (!bno08x.begin_I2C(0x4A, &Wire2)) {
    Serial.println("   -> CRITICAL ERROR: BNO08x NOT DETECTED on 0x4A!");
    while (1) delay(10); 
  }
  
  Serial.println("   -> BNO08x FOUND. Starting Sensor Fusion Coprocessor...");
  // Strict check to catch brownouts instantly
  if (!bno08x.enableReport(SH2_ARVR_STABILIZED_RV, 10000)) {
    Serial.println("   -> CRITICAL ERROR: BNO08x BROWNOUT! Sensor refused to start data stream. Check power wiring!");
    while(1) delay(10);
  }
  if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION, 10000)) {
    Serial.println("   -> CRITICAL ERROR: BNO08x BROWNOUT! Sensor refused linear accel stream.");
    while(1) delay(10);
  }
  Serial.println("   -> BNO08x COPOCESSOR RUNNING AND READY");

  Serial.println("4. Calibrating Ground Altitude (Taking 50 samples)...");
  float altSum = 0;
  for (int i = 0; i < 50; i++) {
    ms5611.read();
    float p = ms5611.getPressure();
    if (p > 0) altSum += 44330.0 * (1.0 - pow(p / 1013.25, 0.1903));
    delay(20);
  }
  groundAltitude = (altSum / 50.0) * 3.28084;
  Serial.print("   -> Ground Altitude set to: ");
  Serial.print(groundAltitude);
  Serial.println(" ft");

  Serial.println("5. Attaching Servos...");
  servo1.attach(PIN_SERVO1);
  servo2.attach(PIN_SERVO2);
  servo3.attach(PIN_SERVO3);
  servo4.attach(PIN_SERVO4);
  writeServos(0, 0, 0);
  
  lastInnerLoopTime = millis();
  lastOuterLoopTime = millis();
  lastLogTime = millis();
  lastFlushTime = millis();
  
  Serial.println("--- BOOT COMPLETE ---");
}

void loop() {
  unsigned long now = millis();

  readSensors();
  
  if (now - lastLogTime >= 20) {
    lastLogTime = now;
    logTelemetry(now);
    if (now - lastFlushTime > 2000) {
      if (flightLog) flightLog.flush();
      lastFlushTime = now;
    }
  }

  if (now - lastOuterLoopTime >= 100) {
    lastOuterLoopTime = now;
    if (currentPhase == NAVIGATE) {
      calculateGuidance();
      // VISUAL DEBUG: Proves sensor is alive and tracking target
      Serial.print("Act Yaw: ");
      Serial.print(yawRad * 57.2958);
      Serial.print(" | Cmd Yaw: "); Serial.println(commandedYawWorld);
    }
  }

  float dt = (now - lastInnerLoopTime) / 1000.0;
  if (dt >= 0.01) {
    lastInnerLoopTime = now;
    
    updateKalmanFilter(dt);
    
    if (currentPhase != PAD) updateDeadReckoning(dt);

    checkFlightPhase(now);
    
    // --- IMU FAILSAFE OVERRIDE ---
    if (testModeActive) {
      // Slowly sweep the servos back and forth between -20 and +20 degrees
      float sweepAngle = 20.0 * sin(now / 500.0);
      writeServos(sweepAngle, sweepAngle, 0.0);
    } 
    else if (currentPhase != PAD) {
      runControlLoop(dt);
    }
  }
}

void logTelemetry(unsigned long timestamp) {
  if (flightLog) {
    flightLog.print(timestamp); flightLog.print(",");
    flightLog.print(currentPhase);
    flightLog.print(",");
    flightLog.print(currentLat, 6); flightLog.print(",");
    flightLog.print(currentLon, 6); flightLog.print(",");
    flightLog.print(kf_z); flightLog.print(",");
    flightLog.print(kf_v); flightLog.print(",");
    flightLog.print(pitchRad * 57.2958); flightLog.print(",");
    flightLog.print(yawRad * 57.2958); flightLog.print(",");
    flightLog.print(rollRad * 57.2958); flightLog.print(",");
    flightLog.print(commandedPitchWorld); flightLog.print(",");
    flightLog.print(commandedYawWorld); flightLog.print(",");
    flightLog.print(log_s1); flightLog.print(",");
    flightLog.print(log_s2); flightLog.print(",");
    flightLog.print(log_s3); flightLog.print(",");
    flightLog.println(log_s4);
  }
}

void readSensors() {
  if (bno08x.wasReset()) {
    bno08x.enableReport(SH2_ARVR_STABILIZED_RV, 10000);
    bno08x.enableReport(SH2_LINEAR_ACCELERATION, 10000);
  }

  // The 'while' loop fully drains the sensor queue so it never freezes
  while (bno08x.getSensorEvent(&sensorValue)) {
    switch (sensorValue.sensorId) {
      case SH2_ARVR_STABILIZED_RV:
        {
          q_i = sensorValue.un.arvrStabilizedRV.i;
          q_j = sensorValue.un.arvrStabilizedRV.j;
          q_k = sensorValue.un.arvrStabilizedRV.k;
          q_r = sensorValue.un.arvrStabilizedRV.real;

          if (isnan(q_i) || isnan(q_j) || isnan(q_k) || isnan(q_r)) break;
          
          if (q_i == 0.0 && q_j == 0.0 && q_k == 0.0 && q_r == 0.0) { 
            q_r = 1.0;
          }

          float sqr = q_r * q_r, sqi = q_i * q_i, sqj = q_j * q_j, sqk = q_k * q_k;
          float norm = sqi + sqj + sqk + sqr;
          if (norm < 0.0001) norm = 1.0;
          
          yawRad = atan2(2.0 * (q_i * q_j + q_k * q_r), (sqi - sqj - sqk + sqr));
          pitchRad = asin(constrain(-2.0 * (q_i * q_k - q_j * q_r) / norm, -1.0, 1.0));
          rollRad = atan2(2.0 * (q_j * q_k + q_i * q_r), (-sqi - sqj + sqk + sqr));
          break;
        }
      case SH2_LINEAR_ACCELERATION:
        {
          accX_body = sensorValue.un.linearAcceleration.x;
          accY_body = sensorValue.un.linearAcceleration.y;
          accZ_body = sensorValue.un.linearAcceleration.z;
          
          // IMU FAILSAFE CHECK
          // It is impossible for a working sensor to return exactly 0.00 for all axes
          // in the real world due to gravity and sensor noise. 
          if (accX_body == 0.0 && accY_body == 0.0 && accZ_body == 0.0) {
            consecutiveZeroCount++;
            if (consecutiveZeroCount >= 20) {
              testModeActive = true; 
            }
          } else {
            consecutiveZeroCount = 0; // Reset counter if a valid reading comes through
          }
          
          break;
        }
    }
  }
}

void updateDeadReckoning(float dt) {
  if (isnan(accX_body) || isnan(accY_body) || isnan(accZ_body)) return;
  
  float ax = accX_body * 3.28084;
  float ay = accY_body * 3.28084;
  float az = accZ_body * 3.28084;
  
  float sqi = q_i * q_i, sqj = q_j * q_j, sqk = q_k * q_k;
  
  float accEast = ax * (1 - 2 * sqj - 2 * sqk) + ay * (2 * q_i * q_j - 2 * q_k * q_r) + az * (2 * q_i * q_k + 2 * q_j * q_r);
  float accNorth = ax * (2 * q_i * q_j + 2 * q_k * q_r) + ay * (1 - 2 * sqi - 2 * sqk) + az * (2 * q_j * q_k - 2 * q_i * q_r);
  
  velEast_fps += accEast * dt;
  velNorth_fps += accNorth * dt;

  posEast_ft += velEast_fps * dt;
  posNorth_ft += velNorth_fps * dt;
  
  currentLat = launchLat + (posNorth_ft / 364000.0);
  currentLon = launchLon + (posEast_ft / (364000.0 * cos(launchLat * PI / 180.0)));
}

void updateKalmanFilter(float dt) {
  float accel_fps2 = accZ_body * 3.28084;

  ms5611.read();
  float pressure_mb = ms5611.getPressure();
  
  if (isnan(pressure_mb) || pressure_mb <= 0.0) pressure_mb = 1013.25; 
  
  float baroAlt = (44330.0 * (1.0 - pow(pressure_mb / 1013.25, 0.1903)) * 3.28084) - groundAltitude;
  
  kf_z += (kf_v * dt) + (0.5 * accel_fps2 * dt * dt);
  kf_v += (accel_fps2 * dt);
  
  float y = baroAlt - kf_z;
  float S = P[0][0] + R_baro;
  float K0 = P[0][0] / S;
  float K1 = P[1][0] / S;
  
  kf_z += K0 * y;
  kf_v += K1 * y;

  float P00_temp = P[0][0];
  float P01_temp = P[0][1];
  
  P[0][0] = (1 - K0) * P00_temp;
  P[0][1] = (1 - K0) * P01_temp;
  
  P[1][0] = -K1 * P00_temp + P[1][0];
  P[1][1] = -K1 * P01_temp + P[1][1];
}

// void checkFlightPhase(unsigned long now) {

//   if (currentPhase == PAD) {

//     // DESK TEST: Launch threshold commented out for automatic triggering

//     //if ((accZ_body / 9.81) > LAUNCH_THRESHOLD_G) {

//     delay(20);

//     currentPhase = BOOST;

//     launchTime = now;

//     Serial.println("LAUNCH! Phase: BOOST");

//     //}

//   } else if (currentPhase == BOOST) {

//     if (now - launchTime > BOOST_DURATION_MS) {

//       currentPhase = NAVIGATE;

//       Serial.println("Phase: NAVIGATE");

//     }

//   }

// }


void checkFlightPhase(unsigned long now) {
  // Static variable persists its value across multiple calls to this function
  static int launchSampleCount = 0;
  
  if (currentPhase == PAD) {
    if ((accZ_body / 9.81) > LAUNCH_THRESHOLD_G) {
      launchSampleCount++;
      
      // Trigger launch only after 10 consecutive samples above the threshold
      if (launchSampleCount >= 2) {
        currentPhase = BOOST;
        launchTime = now;
        Serial.println("LAUNCH! Phase: BOOST");
      }
    } else {
      // Reset the counter if acceleration drops below threshold 
      // This ensures we only trigger on 10 *consecutive* high readings, ignoring random bumps
      launchSampleCount = 0;
    }
  } else if (currentPhase == BOOST) {
    if (now - launchTime > BOOST_DURATION_MS) {
      currentPhase = NAVIGATE;
      Serial.println("Phase: NAVIGATE");
    }
  }
}

void calculateGuidance() {
  float lat1 = currentLat * (PI / 180.0);
  float lon1 = currentLon * (PI / 180.0);
  float lat2 = targetLat * (PI / 180.0);
  float lon2 = targetLon * (PI / 180.0);

  float dLon = lon2 - lon1;
  float dLat = lat2 - lat1;
  
  float x = sin(dLon) * cos(lat2);
  float y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
  
  float bearingDeg = atan2(x, y) * (180.0 / PI);
  if (bearingDeg < 0) bearingDeg += 360.0;
  
  commandedYawWorld = bearingDeg;
  
  float a = sin(dLat / 2) * sin(dLat / 2) + cos(lat1) * cos(lat2) * sin(dLon / 2) * sin(dLon / 2);
  a = constrain(a, 0.0, 1.0);
  
  float distance_ft = 20902231.0 * (2 * atan2(sqrt(a), sqrt(1.0 - a)));
  
  if (distance_ft < 5.0) {
    commandedPitchWorld = 0.0;
  } else {
    float elevationDeg = atan2(targetAlt - kf_z, distance_ft) * (180.0 / PI);
    commandedPitchWorld = constrain(90.0 - elevationDeg, 0.0, 20.0);
  }
}

void runControlLoop(float dt) {
  float rollDeg = rollRad * (180.0 / PI);
  float pitchDeg = pitchRad * (180.0 / PI);
  float yawDeg = yawRad * (180.0 / PI);
  
  if (yawDeg < 0) yawDeg += 360.0;

  float targetPitch = (currentPhase == BOOST) ? 0.0 : commandedPitchWorld;
  float targetYaw = (currentPhase == BOOST) ? yawDeg : commandedYawWorld;

  if (isnan(targetPitch) || isinf(targetPitch)) targetPitch = 0.0;
  if (isnan(targetYaw) || isinf(targetYaw)) targetYaw = yawDeg;

  float errorPitchWorld = targetPitch - pitchDeg;
  float errorYawWorld = targetYaw - yawDeg;
  
  while (errorYawWorld > 180.0) errorYawWorld -= 360.0;
  while (errorYawWorld < -180.0) errorYawWorld += 360.0;
  
  float errorPitchBody = errorPitchWorld * cos(rollRad) + errorYawWorld * sin(rollRad);
  float errorYawBody = -errorPitchWorld * sin(rollRad) + errorYawWorld * cos(rollRad);
  
  if (isnan(errorPitchBody) || isinf(errorPitchBody)) errorPitchBody = 0.0;
  if (isnan(errorYawBody) || isinf(errorYawBody)) errorYawBody = 0.0;
  
  if (isnan(integralPitch) || isinf(integralPitch)) integralPitch = 0.0;
  if (isnan(integralYaw) || isinf(integralYaw)) integralYaw = 0.0;
  
  if (isnan(lastErrorPitch) || isinf(lastErrorPitch)) lastErrorPitch = 0.0; 
  if (isnan(lastErrorYaw) || isinf(lastErrorYaw)) lastErrorYaw = 0.0;       
  
  integralPitch += errorPitchBody * dt;
  integralPitch = constrain(integralPitch, -20, 20);
  float derivativePitch = (errorPitchBody - lastErrorPitch) / dt;
  
  float pidPitch = (Kp * errorPitchBody) + (Ki * integralPitch) + (Kd * derivativePitch);
  lastErrorPitch = errorPitchBody;
  
  integralYaw += errorYawBody * dt;
  integralYaw = constrain(integralYaw, -20, 20);
  float derivativeYaw = (errorYawBody - lastErrorYaw) / dt;
  
  float pidYaw = (Kp * errorYawBody) + (Ki * integralYaw) + (Kd * derivativeYaw);
  lastErrorYaw = errorYawBody;
  
  float deflPitchCmd = constrain(pidPitch, -MAX_FIN_DEFLECTION, MAX_FIN_DEFLECTION);
  float deflYawCmd = constrain(pidYaw, -MAX_FIN_DEFLECTION, MAX_FIN_DEFLECTION);
  float outputRoll = 0.0; 

  writeServos(deflPitchCmd, deflYawCmd, outputRoll);
}

void writeServos(float pitchCmd, float yawCmd, float rollCmd) {
  if (isnan(pitchCmd) || isinf(pitchCmd)) pitchCmd = 0.0;
  if (isnan(yawCmd) || isinf(yawCmd)) yawCmd = 0.0;
  if (isnan(rollCmd) || isinf(rollCmd)) rollCmd = 0.0;

  int center = 90;
  int s1 = center + S1_OFFSET + pitchCmd + rollCmd;
  int s2 = center + S2_OFFSET + yawCmd + rollCmd;
  int s3 = center + S3_OFFSET - pitchCmd + rollCmd;
  int s4 = center + S4_OFFSET - yawCmd + rollCmd;
  
  s1 = constrain(s1, 70, 110);
  s2 = constrain(s2, 70, 110);
  s3 = constrain(s3, 70, 110);
  s4 = constrain(s4, 70, 110);
  
  log_s1 = s1;
  log_s2 = s2;
  log_s3 = s3;
  log_s4 = s4;

  servo1.write(s1);
  servo2.write(s2);
  servo3.write(s3);
  servo4.write(s4);
}