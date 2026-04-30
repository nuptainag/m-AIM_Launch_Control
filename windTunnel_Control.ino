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

MS5611 ms5611(0x77); // GY-63 default I2C address
float groundAltitude = 0.0;

Servo servo1, servo2, servo3, servo4;
const int PIN_SERVO1 = 5;  // North Fin -- 2
const int PIN_SERVO2 = 2;  // East Fin -- 5
const int PIN_SERVO3 = 3;  // South Fin --4 
const int PIN_SERVO4 = 4;  // West Fin -- 3

const int S1_OFFSET = 0;
const int S2_OFFSET = 0;
const int S3_OFFSET = 0;
const int S4_OFFSET = 0;

// --- Aerodynamic Constants & LUT ---
const float AIR_DENSITY = 0.001915;   // slugs/ft^3
const float REF_AREA = 25.6 / 144.0;  // ft^2
const int NUM_AOA = 13;
const int NUM_CL = 15;

const float aoa_breakpoints[NUM_AOA] = { -30.00, -25.00, -20.00, -15.00, -10.00, -5.00, 0.00, 5.00, 10.00, 15.00, 20.00, 25.00, 30.00 };
const float cl_req_breakpoints[NUM_CL] = { -1.00, -0.86, -0.71, -0.57, -0.43, -0.29, -0.14, 0.00, 0.14, 0.29, 0.43, 0.57, 0.71, 0.86, 1.00 };

// Rows: Current AoA | Columns: Required CL
const float fin_deflection_lut[NUM_AOA][NUM_CL] = {
  { 22.00, 31.93, 39.88, 47.82, 55.77, 63.72, 71.67, 79.62, 87.57, 95.51, 103.46, 111.41, 119.36, 127.31, 135.26 },
  { 10.86, 19.69, 30.16, 38.67, 47.18, 55.69, 64.20, 72.71, 81.22, 89.73, 98.24, 106.75, 115.26, 123.77, 132.28 },
  { -1.77, 3.50, 12.63, 24.98, 36.85, 47.51, 58.17, 68.84, 79.50, 90.16, 100.82, 111.48, 122.14, 132.80, 143.46 },
  { -16.12, -9.06, -2.01, 5.04, 12.88, 22.86, 32.90, 41.58, 50.26, 58.94, 67.62, 76.30, 84.97, 93.65, 102.33 },
  { -27.44, -20.70, -13.95, -7.21, -0.46, 6.61, 14.67, 25.06, 34.54, 43.12, 51.70, 60.28, 68.87, 77.45, 86.03 },
  { -40.37, -33.40, -26.44, -19.48, -12.52, -5.56, 1.41, 8.84, 17.00, 27.31, 36.21, 44.61, 53.01, 61.41, 69.81 },
  { -53.18, -45.77, -38.36, -30.95, -23.54, -16.13, -8.72, -1.31, 6.11, 13.70, 23.78, 33.65, 42.52, 51.39, 60.25 },
  { -63.49, -56.23, -48.97, -41.71, -34.45, -27.19, -19.94, -12.68, -5.41, 1.93, 10.52, 19.01, 27.54, 36.82, 46.29 }, // Repaired Row 7
  { -73.80, -66.69, -59.59, -52.48, -45.37, -38.26, -31.16, -24.05, -16.94, -9.84, -2.73, 4.38, 12.56, 22.26, 32.34 },
  { -91.65, -84.16, -76.66, -69.17, -61.67, -54.18, -46.68, -39.19, -31.69, -24.20, -16.70, -9.21, -1.71, 5.86, 14.37 },
  { -98.87, -91.87, -84.87, -77.86, -70.86, -63.86, -56.86, -49.85, -42.85, -35.85, -28.84, -21.84, -14.84, -7.84, -0.83 },
  { -113.52, -106.40, -99.28, -92.16, -85.04, -77.92, -70.80, -63.67, -56.55, -49.43, -42.31, -35.19, -28.07, -20.95, -13.82 },
  { -117.52, -110.67, -103.82, -96.98, -90.13, -83.28, -76.43, -69.58, -62.73, -55.89, -49.04, -42.19, -35.34, -28.49, -21.64 }
};

// --- Tuning Parameters (PID) ---
// Pitch & Yaw
float Kp = 3.0;
float Ki = 0.0;
float Kd = 0.1;

// Roll (Decoupled & Tuned separately for low inertia)
float Kp_roll = 5.0; 
float Ki_roll = 0.0;
float Kd_roll = 0.0;

// --- State Variables & Flight Phasing ---
enum FlightPhase { PAD, FLIGHT };
FlightPhase currentPhase = PAD;

const float LAUNCH_THRESHOLD_G = 2.5;
const float GRAVITY_FPS2 = 32.174;
unsigned long launchTime = 0;

// PID Memory
float integralPitch = 0, integralYaw = 0, integralRoll = 0;
float lastErrorPitch = 0, lastErrorYaw = 0, lastErrorRoll = 0;

// Derivative Low-Pass Filter variables
float derivPitchFiltered = 0.0;
float derivYawFiltered = 0.0;
float derivRollFiltered = 0.0;

// Target Orientation
float targetPitchWorld = 0.0;
float targetYawWorld = 0.0;
float targetRollWorld = 0.0;

// Loop Timing
unsigned long lastInnerLoopTime = 0;
unsigned long lastLogTime = 0;

// Sensor Holding Variables
float rollRad = 0, pitchRad = 0, yawRad = 0;
float q_i = 0, q_j = 0, q_k = 0, q_r = 1.0;
float accX_body = 0, accY_body = 0, accZ_body = 0;

// --- KALMAN FILTER VARIABLES (1D for Altitude/Velocity) ---
float kf_z = 0.0;
float kf_v = 0.1;
float P[2][2] = { { 1.0, 0.0 }, { 0.0, 1.0 } };
float Q_z = 0.01;
float Q_v = 0.1;
float R_baro = 2.0;

// --- DATA LOGGING VARIABLES ---
File flightLog;
int log_s1 = 90, log_s2 = 90, log_s3 = 90, log_s4 = 90;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire2.begin();
  Wire.setClock(400000);
  Wire2.setClock(400000);

  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("SD Card initialization failed! Flight proceeding without telemetry.");
  } else {
    flightLog = SD.open("flight_log.csv", FILE_WRITE);
    if (flightLog) {
      flightLog.println("Time_ms,Phase,Alt_ft,Vel_fps,Pitch_deg,Yaw_deg,Roll_deg,S1,S2,S3,S4");
      flightLog.flush();
      Serial.println("SD Card Active. Telemetry logging to flight_log.csv");
    }
  }

  // --- MS5611 Initialization ---
  if (!ms5611.begin()) {
    Serial.println("MS5611 not detected! Check wiring.");
    while (1) delay(10);
  }

  // --- BNO08x Initialization ---
  if (!bno08x.begin_I2C(0x4A, &Wire2)) {
    Serial.println("BNO08x not detected!");
    while (1) delay(10);
  }
  bno08x.enableReport(SH2_ARVR_STABILIZED_RV, 2500);
  bno08x.enableReport(SH2_LINEAR_ACCELERATION, 2500);

  // --- Ground Altitude Calibration ---
  float altSum = 0;
  for (int i = 0; i < 50; i++) {
    ms5611.read();
    float pressure_mb = ms5611.getPressure();
    float currentAltMeters = 44330.0 * (1.0 - pow(pressure_mb / 1013.25, 0.1903));
    altSum += currentAltMeters;
    delay(20);
  }
  groundAltitude = (altSum / 50.0) * 3.28084;
  
  // --- Servo Initialization ---
  servo1.attach(PIN_SERVO1);
  servo2.attach(PIN_SERVO2);
  servo3.attach(PIN_SERVO3);
  servo4.attach(PIN_SERVO4);
  writeServos(0, 0, 0);

  Serial.println("System Ready. Waiting for Launch.");
  lastInnerLoopTime = micros(); 
  lastLogTime = millis();
}

void loop() {
  unsigned long now = millis();

  // 50Hz Data Logging
  if (now - lastLogTime >= 20) {
    lastLogTime = now;
    logTelemetry(now);
  }

  // Inner Loop: 100Hz (PID & Actuation) using micros for precision
  unsigned long nowMicros = micros();
  float dt = (nowMicros - lastInnerLoopTime) / 1000000.0;
  
  if (dt >= 0.0025) {
    lastInnerLoopTime = nowMicros;
    
    readSensors();
    updateKalmanFilter(dt);
    checkFlightPhase(now);
    if (currentPhase == FLIGHT) {
      runControlLoop(dt);
    }
  }
}

void logTelemetry(unsigned long timestamp) {
  if (flightLog) {
    float pitchDeg = pitchRad * (180.0 / PI);
    float yawDeg = yawRad * (180.0 / PI);
    float rollDeg = rollRad * (180.0 / PI);

    flightLog.print(timestamp);
    flightLog.print(",");
    flightLog.print(currentPhase);
    flightLog.print(",");
    flightLog.print(kf_z);
    flightLog.print(",");
    flightLog.print(kf_v);
    flightLog.print(",");
    flightLog.print(pitchDeg);
    flightLog.print(",");
    flightLog.print(yawDeg);
    flightLog.print(",");
    flightLog.print(rollDeg);
    flightLog.print(",");
    flightLog.print(log_s1);
    flightLog.print(",");
    flightLog.print(log_s2);
    flightLog.print(",");
    flightLog.print(log_s3);
    flightLog.print(",");
    flightLog.println(log_s4);

    flightLog.flush();
  }
}

void readSensors() {
  if (bno08x.wasReset()) {
    bno08x.enableReport(SH2_ARVR_STABILIZED_RV, 2500);
    bno08x.enableReport(SH2_LINEAR_ACCELERATION, 2500);
  }

  if (bno08x.getSensorEvent(&sensorValue)) {
    switch (sensorValue.sensorId) {
      case SH2_ARVR_STABILIZED_RV:
        {
          q_i = sensorValue.un.arvrStabilizedRV.i;
          q_j = sensorValue.un.arvrStabilizedRV.j;
          q_k = sensorValue.un.arvrStabilizedRV.k;
          q_r = sensorValue.un.arvrStabilizedRV.real;

          float sqr = q_r * q_r, sqi = q_i * q_i, sqj = q_j * q_j, sqk = q_k * q_k;
          
          rollRad = atan2(2.0 * (q_i * q_j + q_k * q_r), (sqi - sqj - sqk + sqr)); 
          yawRad = asin(-2.0 * (q_i * q_k - q_j * q_r) / (sqi + sqj + sqk + sqr)); 
          pitchRad = atan2(2.0 * (q_j * q_k + q_i * q_r), (-sqi - sqj + sqk + sqr));
          break;
        }
      case SH2_LINEAR_ACCELERATION:
        {
          accX_body = sensorValue.un.linearAcceleration.x;
          accY_body = sensorValue.un.linearAcceleration.y;
          accZ_body = sensorValue.un.linearAcceleration.z;
          break;
        }
    }
  }
}

void updateKalmanFilter(float dt) {
  float accel_fps2 = accZ_body * 3.28084;

  ms5611.read();
  float pressure_mb = ms5611.getPressure();
  float alt_meters = 44330.0 * (1.0 - pow(pressure_mb / 1013.25, 0.1903));
  float baroAlt = (alt_meters * 3.28084) - groundAltitude;

  kf_z = kf_z + (kf_v * dt) + (0.5 * accel_fps2 * dt * dt);
  kf_v = kf_v + (accel_fps2 * dt);

  P[0][0] += dt * (dt * P[1][1] + P[0][1] + P[1][0]) + Q_z;
  P[0][1] += dt * P[1][1];
  P[1][0] += dt * P[1][1];
  P[1][1] += Q_v;

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

void checkFlightPhase(unsigned long now) {
  if (currentPhase == PAD) {
    float gForceZ = accZ_body / 9.81;
    // For bench testing, you can comment this out to force FLIGHT mode
    //if (gForceZ > LAUNCH_THRESHOLD_G) {
    currentPhase = FLIGHT;
    launchTime = now;
    Serial.println("LAUNCH! Phase: FLIGHT (Stabilization Active).");
    
    targetYawWorld = yawRad * (180.0 / PI);
    targetPitchWorld = pitchRad * (180.0 / PI);
    targetRollWorld = rollRad * (180.0 / PI);
    //}
  }
}

float getFinDeflection(float current_aoa, float required_cl) {
  current_aoa = constrain(current_aoa, aoa_breakpoints[0], aoa_breakpoints[NUM_AOA - 1]);
  required_cl = constrain(required_cl, cl_req_breakpoints[0], cl_req_breakpoints[NUM_CL - 1]);

  int i = 0, j = 0;
  while (i < NUM_AOA - 2 && current_aoa > aoa_breakpoints[i + 1]) i++;
  while (j < NUM_CL - 2 && required_cl > cl_req_breakpoints[j + 1]) j++;

  float t_aoa = (current_aoa - aoa_breakpoints[i]) / (aoa_breakpoints[i + 1] - aoa_breakpoints[i]);
  float t_cl = (required_cl - cl_req_breakpoints[j]) / (cl_req_breakpoints[j + 1] - cl_req_breakpoints[j]);

  float y1 = fin_deflection_lut[i][j] * (1 - t_cl) + fin_deflection_lut[i][j + 1] * t_cl;
  float y2 = fin_deflection_lut[i + 1][j] * (1 - t_cl) + fin_deflection_lut[i + 1][j + 1] * t_cl;

  return y1 * (1 - t_aoa) + y2 * t_aoa;
}

void runControlLoop(float dt) {
  float rollDeg = rollRad * (180.0 / PI);
  float pitchDeg = pitchRad * (180.0 / PI);
  float yawDeg = yawRad * (180.0 / PI);
  if (yawDeg < 0) yawDeg += 360.0;

  // Calculate World Frame Errors
  float errorPitchWorld = targetPitchWorld - pitchDeg;
  float errorYawWorld = targetYawWorld - yawDeg;
  float errorRollWorld = targetRollWorld - rollDeg;
  
  while (errorYawWorld > 180.0) errorYawWorld -= 360.0;
  while (errorYawWorld < -180.0) errorYawWorld += 360.0;
  
  while (errorRollWorld > 180.0) errorRollWorld -= 360.0;
  while (errorRollWorld < -180.0) errorRollWorld += 360.0;

  // Rotate Errors into Body Frame
  float errorPitchBody = errorPitchWorld * cos(rollRad) + errorYawWorld * sin(rollRad);
  float errorYawBody = -errorPitchWorld * sin(rollRad) + errorYawWorld * cos(rollRad);

  // --- PID Control with Low-Pass Filtered Derivatives ---
  
  // Pitch
  integralPitch += errorPitchBody * dt;
  integralPitch = constrain(integralPitch, -20, 20);
  float rawDerivPitch = (errorPitchBody - lastErrorPitch) / dt;
  derivPitchFiltered = (0.6 * derivPitchFiltered) + (0.4 * rawDerivPitch); 
  float forcePitchReq = (Kp * errorPitchBody) + (Ki * integralPitch) + (Kd * derivPitchFiltered);
  lastErrorPitch = errorPitchBody;

  // Yaw
  integralYaw += errorYawBody * dt;
  integralYaw = constrain(integralYaw, -20, 20);
  float rawDerivYaw = (errorYawBody - lastErrorYaw) / dt;
  derivYawFiltered = (0.6 * derivYawFiltered) + (0.4 * rawDerivYaw);
  float forceYawReq = (Kp * errorYawBody) + (Ki * integralYaw) + (Kd * derivYawFiltered);
  lastErrorYaw = errorYawBody;

  // Roll
  integralRoll += errorRollWorld * dt;
  integralRoll = constrain(integralRoll, -20, 20);
  float rawDerivRoll = (errorRollWorld - lastErrorRoll) / dt;
  derivRollFiltered = (0.6 * derivRollFiltered) + (0.4 * rawDerivRoll); 
  float forceRollReq = (Kp_roll * errorRollWorld) + (Ki_roll * integralRoll) + (Kd_roll * derivRollFiltered);
  lastErrorRoll = errorRollWorld;

  // --- AERODYNAMIC CONVERSION ---
  float velocity_fps = kf_v;
  
  // FIX: Force a minimum virtual speed of 250 fps to prevent divide-by-zero 
  // saturation on the bench or right off the pad. This restores proportional control.
  if (velocity_fps < 250.0) velocity_fps = 250.0; 
  
  float q = 0.5 * AIR_DENSITY * (velocity_fps * velocity_fps);
  if (q < 0.1) q = 0.1; 
  
  float cl_pitch_req = forcePitchReq / (q * REF_AREA);
  float cl_yaw_req = forceYawReq / (q * REF_AREA);
  float cl_roll_req = forceRollReq / (q * REF_AREA);
  
  // Use -errorBody proxies as relative AoA instead of absolute orientation
  float deflPitchCmd = getFinDeflection(-errorPitchBody, cl_pitch_req);
  float deflYawCmd = getFinDeflection(-errorYawBody, cl_yaw_req);
  float deflRollCmd = getFinDeflection(0.0, cl_roll_req); 

  writeServos(deflPitchCmd, deflYawCmd, deflRollCmd);
}

void writeServos(float pitchCmd, float yawCmd, float rollCmd) {
  int center = 90;

  int s1 = center + S1_OFFSET + pitchCmd + rollCmd;
  int s2 = center + S2_OFFSET + yawCmd + rollCmd;
  int s3 = center + S3_OFFSET - pitchCmd + rollCmd;
  int s4 = center + S4_OFFSET - yawCmd + rollCmd;

  // Widen constraints to ±35 degrees of travel to prevent Roll saturation
  s1 = constrain(s1, 55, 125);
  s2 = constrain(s2, 55, 125);
  s3 = constrain(s3, 55, 125);
  s4 = constrain(s4, 55, 125);

  log_s1 = s1;
  log_s2 = s2;
  log_s3 = s3;
  log_s4 = s4;

  servo1.write(s1);
  servo2.write(s2);
  servo3.write(s3);
  servo4.write(s4);
}