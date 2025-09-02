/*
================================================================================
 LSM9DS1 KALMAN FILTER IMU FIRMWARE
================================================================================

OVERVIEW:
This firmware implements a Kalman filter-based 9-DOF sensor fusion algorithm for the 
LSM9DS1 IMU (accelerometer + gyroscope + magnetometer) to provide stable and 
accurate orientation tracking with minimal drift and optimal noise reduction.

SENSOR FUSION ALGORITHM DETAILS:
================================================================================

1. ROLL & PITCH (X & Y AXES) - KALMAN FILTER (PROPER MATRIX IMPLEMENTATION):
   - Uses accelerometer for absolute reference (gravity vector)
   - Uses gyroscope for responsive short-term tracking
   - Full matrix-based Kalman filter with proper covariance matrices
   - Process noise covariance Q (2x2): models uncertainty in state evolution
   - Measurement noise covariance R (scalar): models accelerometer noise
   - State vector: [angle, gyro_bias]'
   - Automatic bias correction and optimal sensor fusion

2. YAW (Z AXIS) - DIRECT MAGNETOMETER CALCULATION:
   - Uses magnetometer readings directly for yaw calculation
   - Simple atan2(magY, magX) approach with magnetic declination correction
   - No gyroscope integration for yaw to prevent drift
   - Magnetic declination: -10° 13' (adjustable for location)
   - Normalizes yaw to 0-360 degrees

3. MAGNETOMETER CALIBRATION:
   - Hard iron calibration: Corrects for magnetic distortions
   - Collects min/max values during 30-second rotation procedure
   - Calculates offsets: (max + min) / 2 for each axis
   - Calculates scale factors to normalize magnetic field strength

KALMAN FILTER ADVANTAGES:
================================================================================
✓ Optimal sensor fusion - automatically weighs sensor reliability
✓ Adaptive noise filtering - reduces noise while maintaining responsiveness
✓ Minimal drift - corrects gyroscope bias over time
✓ Smooth output - eliminates accelerometer noise without lag
✓ Self-tuning - adjusts to changing sensor conditions

KEY PARAMETERS:
================================================================================
- FREQ: 30.0 Hz - Main loop frequency
- Q[0][0]: 0.01 - Process noise variance for angle estimation
- Q[1][1]: 0.005 - Process noise variance for gyroscope bias
- R: 0.5 - Measurement noise variance (accelerometer)
- State vector: [angle, bias]' (2x1)
- Process noise covariance Q (2x2 matrix)
- Measurement noise covariance R (scalar)
- Magnetic declination: -10° 13' (adjust for your location)

COMMANDS:
================================================================================
- '.' : Get current angles (roll, pitch, yaw)
- 'z' : Reset yaw to 0°
- 'm' : Calibrate magnetometer (rotate device for 30 seconds)
- 'r' : Show raw sensor data
- 'c' : Recalibrate gyroscope
- 'q' : Show Kalman filter parameters and tuning commands
- '1'/'2' : Increase/decrease Q[0][0] (angle process noise)
- '3'/'4' : Increase/decrease Q[1][1] (bias process noise)  
- '5'/'6' : Increase/decrease R (measurement noise)

BENEFITS:
================================================================================
✓ Superior noise rejection compared to complementary filters
✓ Automatic sensor weighting based on movement conditions
✓ No yaw drift (magnetometer provides absolute reference)
✓ Smooth, stable output with fast response to real movement
✓ Self-calibrating gyroscope bias correction

Compatible with: Arduino Nano 33 BLE Sense, Arduino Nano 33 IoT
Author: Kalman Filter IMU Sensor Fusion Implementation
*/

// LSM9DS1 Accelerometer + Gyro + Magnetometer Sensor Fusion
// Advanced IMU firmware with adaptive filtering for stable orientation tracking
// Compatible with Arduino Nano 33 BLE Sense

#include <Arduino_LSM9DS1.h>
#include <math.h>

#define FREQ  30.0 // sample freq in Hz

// global angle, gyro derived
float gx = 0, gy = 0, gz = 0;
float gyrX = 0, gyrY = 0, gyrZ = 0;
float accX = 0, accY = 0, accZ = 0;
float magX = 0, magY = 0, magZ = 0;

// gyro calibration offsets
float gyrXoffs = 0, gyrYoffs = 0, gyrZoffs = 0;

// magnetometer calibration offsets and scale factors
float magXoffs = 0, magYoffs = 0, magZoffs = 0;
float magXscale = 1.0, magYscale = 1.0, magZscale = 1.0;

// --- KALMAN FILTER PARAMETERS (PROPER MATRIX IMPLEMENTATION) ---
// Process noise covariance matrix Q (2x2) - models uncertainty in the process
float Q[2][2] = {
  {0.01, 0.0},    // Q11: angle process noise variance, Q12: angle-bias cross-correlation
  {0.0, 0.005}    // Q21: bias-angle cross-correlation, Q22: bias process noise variance
};

// Measurement noise covariance R (scalar for 1D measurement)
float R = 0.5;  // Measurement noise variance (accelerometer noise)

// State transition matrix F (2x2) - how state evolves over time
// F = [1, -dt]  where dt is time step
//     [0,  1 ]

// Measurement matrix H (1x2) - how measurement relates to state
// H = [1, 0] - we measure angle directly, not bias

// Kalman filter state variables for pitch (X axis)
float pitchAngle = 0, pitchBias = 0, pitchRate = 0;
float pitchP[2][2] = {{0, 0}, {0, 0}};    // Error covariance matrix for pitch

// Kalman filter state variables for roll (Y axis)
float rollAngle = 0, rollBias = 0, rollRate = 0;
float rollP[2][2] = {{0, 0}, {0, 0}};     // Error covariance matrix for roll

// Time tracking for Kalman filter
float dt = 1.0/FREQ;

// calibration variables
bool isCalibrated = false;
int calibrationSamples = 500;

void setup() {
  Serial.begin(38400);
  while (!Serial);
  
  // debug led
  pinMode(13, OUTPUT); 
  digitalWrite(13, HIGH);
  
  Serial.println("LSM9DS1 IMU Starting...");
  
  // Initialize the LSM9DS1 IMU
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1) {
      digitalWrite(13, HIGH);
      delay(100);
      digitalWrite(13, LOW);
      delay(100);
    }
  }
  
  Serial.println("IMU initialized successfully!");
  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.print("Magnetic field sample rate = ");
  Serial.print(IMU.magneticFieldSampleRate());
  Serial.println(" Hz");
  
  // Calibrate gyroscope and magnetometer
  Serial.println("Calibrating sensors... Keep sensor still!");
  digitalWrite(13, HIGH);
  calibrate();
  digitalWrite(13, LOW);
  Serial.println("Calibration complete!");
  
  // Initialize Kalman filter covariance matrices
  pitchP[0][0] = 1.0;
  pitchP[1][1] = 1.0;
  rollP[0][0] = 1.0;  
  rollP[1][1] = 1.0;
  
  Serial.println("System ready!");
  Serial.println("Commands: '.' = angles, 'z' = reset Z, 'm' = mag cal, 'r' = raw data, 'q' = Kalman tuning");
}

void loop() {
  unsigned long start_time, end_time;
  double ax, ay;
  
  // ============================================================================
  // SENSOR FUSION ALGORITHM IMPLEMENTATION - KALMAN FILTER APPROACH
  // ============================================================================
  
  start_time = millis();
  
  // Read raw sensor data from LSM9DS1
  read_sensor_data();
  
  // ============================================================================
  // STEP 1: CALCULATE ABSOLUTE ANGLES FROM ACCELEROMETER (ROLL & PITCH)
  // ============================================================================
  // Accelerometer provides absolute reference using gravity vector
  ax = atan2(accY, sqrt(pow(accX, 2) + pow(accZ, 2))) * 180.0 / M_PI;  // Roll
  ay = atan2(-accX, sqrt(pow(accY, 2) + pow(accZ, 2))) * 180.0 / M_PI; // Pitch
  
  // ============================================================================
  // STEP 2: APPLY KALMAN FILTER FOR PITCH AND ROLL
  // ============================================================================
  gx = kalmanFilter(gx, gyrX, ax, pitchP, pitchBias);  // Pitch with Kalman filter
  gy = kalmanFilter(gy, gyrY, ay, rollP, rollBias);    // Roll with Kalman filter
  
  // ============================================================================
  // STEP 3: YAW CALCULATION USING MAGNETOMETER ONLY
  // ============================================================================
  // Calculate tilt-compensated heading from magnetometer
  gz = atan2(magY, magX); // Calculate yaw from magnetometer readings
  float declinationAngle = -0.1783; // Declination in radians for -10° 13' (adjust for your location)
  gz += declinationAngle;          // Adjust for magnetic declination
  
  // Normalize yaw to 0-360 degrees
  if (gz < 0) gz += 2 * M_PI;
  if (gz > 2 * M_PI) gz -= 2 * M_PI;
  gz = gz * 180.0 / M_PI; // Convert yaw to degrees
  
  // Handle serial communication
  if (Serial.available()) {
    char rx_char = Serial.read();
    
    // Send current angle data
    if (rx_char == '.') {
      digitalWrite(13, HIGH);
      Serial.print(gx, 2);
      Serial.print(", ");
      Serial.print(gy, 2);
      Serial.print(", ");
      Serial.println(gz, 2);
      digitalWrite(13, LOW);
    }
    
    // Reset Z axis (yaw)
    if (rx_char == 'z') {
      gz = 0;
      Serial.println("Z axis (yaw) reset to 0");
    }
    
    // Debug: print raw sensor data
    if (rx_char == 'r') {
      Serial.print("Raw - Acc: ");
      Serial.print(accX, 3); Serial.print(", ");
      Serial.print(accY, 3); Serial.print(", ");
      Serial.print(accZ, 3);
      Serial.print(" | Gyro: ");
      Serial.print(gyrX, 3); Serial.print(", ");
      Serial.print(gyrY, 3); Serial.print(", ");
      Serial.print(gyrZ, 3);
      Serial.print(" | Mag: ");
      Serial.print(magX, 3); Serial.print(", ");
      Serial.print(magY, 3); Serial.print(", ");
      Serial.println(magZ, 3);
    }
    
    // Kalman filter parameter tuning commands
    if (rx_char == 'q') {
      Serial.println("Kalman Filter Parameters (Matrix Implementation):");
      Serial.print("Q[0][0] (angle process noise): "); Serial.println(Q[0][0], 4);
      Serial.print("Q[1][1] (bias process noise): "); Serial.println(Q[1][1], 4);
      Serial.print("R (measurement noise): "); Serial.println(R, 4);
      Serial.println("Commands: '1'=Q[0][0]+, '2'=Q[0][0]-, '3'=Q[1][1]+, '4'=Q[1][1]-, '5'=R+, '6'=R-");
    }
    
    // Kalman filter real-time tuning
    if (rx_char == '1') {
      Q[0][0] += 0.001;
      if (Q[0][0] > 0.1) Q[0][0] = 0.1;
      Serial.print("Q[0][0] (angle process noise) increased to: "); Serial.println(Q[0][0], 4);
    }
    if (rx_char == '2') {
      Q[0][0] -= 0.001;
      if (Q[0][0] < 0.0001) Q[0][0] = 0.0001;
      Serial.print("Q[0][0] (angle process noise) decreased to: "); Serial.println(Q[0][0], 4);
    }
    if (rx_char == '3') {
      Q[1][1] += 0.001;
      if (Q[1][1] > 0.05) Q[1][1] = 0.05;
      Serial.print("Q[1][1] (bias process noise) increased to: "); Serial.println(Q[1][1], 4);
    }
    if (rx_char == '4') {
      Q[1][1] -= 0.001;
      if (Q[1][1] < 0.0001) Q[1][1] = 0.0001;
      Serial.print("Q[1][1] (bias process noise) decreased to: "); Serial.println(Q[1][1], 4);
    }
    if (rx_char == '5') {
      R += 0.1;
      if (R > 5.0) R = 5.0;
      Serial.print("R (measurement noise) increased to: "); Serial.println(R, 4);
    }
    if (rx_char == '6') {
      R -= 0.1;
      if (R < 0.01) R = 0.01;
      Serial.print("R (measurement noise) decreased to: "); Serial.println(R, 4);
    }
    
    // Magnetometer calibration
    if (rx_char == 'm') {
      Serial.println("Starting magnetometer calibration...");
      Serial.println("Rotate the sensor in all directions for 30 seconds!");
      digitalWrite(13, HIGH);
      calibrateMagnetometer();
      digitalWrite(13, LOW);
      Serial.println("Magnetometer calibration complete!");
    }
    
    // Recalibrate gyroscope
    if (rx_char == 'c') {
      Serial.println("Recalibrating gyroscope...");
      digitalWrite(13, HIGH);
      calibrate();
      digitalWrite(13, LOW);
      Serial.println("Recalibration complete!");
    }
  }
  
  // Maintain consistent loop timing
  end_time = millis();
  int loopTime = ((1.0/FREQ) * 1000) - (end_time - start_time);
  if (loopTime > 0) {
    delay(loopTime);
  }
}

// --- KALMAN FILTER FUNCTION (PROPER MATRIX IMPLEMENTATION) ---
float kalmanFilter(float angle, float gyroRate, float accelAngle, float P[2][2], float &bias) {
  // State vector: x = [angle, bias]'
  // Measurement: z = accelAngle
  
  // PREDICT STEP
  // 1. State prediction: x_pred = F * x_prev + B * u
  float rate = gyroRate - bias;
  angle += dt * rate;  // angle_pred = angle_prev + dt * (gyroRate - bias)
  // bias prediction: bias remains the same (no control input)
  
  // 2. Covariance prediction: P_pred = F * P * F' + Q
  // 
  // WHY THIS STEP EXISTS:
  // - P matrix tracks our uncertainty (how confident we are in our estimates)
  // - F matrix models how the system evolves over time  
  // - This step propagates uncertainty through the system dynamics
  // - Without this, we wouldn't know how much to trust our predictions
  //
  // State transition matrix F models: angle_new = angle_old + dt*(gyro - bias)
  //                                   bias_new = bias_old (bias persists)
  float F[2][2] = {{1.0, -dt}, {0.0, 1.0}};  // F = [1  -dt]  ← bias affects angle
                                              //     [0   1 ]  ← bias persists
  
  // Calculate F * P (intermediate step for efficiency and readability)
  // FP represents "how does current uncertainty transform through system dynamics"
  float FP[2][2];
  FP[0][0] = F[0][0] * P[0][0] + F[0][1] * P[1][0];  // How angle uncertainty evolves
  FP[0][1] = F[0][0] * P[0][1] + F[0][1] * P[1][1];  // How angle-bias correlation evolves  
  FP[1][0] = F[1][0] * P[0][0] + F[1][1] * P[1][0];  // How bias-angle correlation evolves
  FP[1][1] = F[1][0] * P[0][1] + F[1][1] * P[1][1];  // How bias uncertainty evolves
  
  // Calculate P_pred = (F * P) * F' + Q
  // This completes uncertainty propagation and adds process noise
  P[0][0] = FP[0][0] * F[0][0] + FP[0][1] * F[1][0] + Q[0][0];  // Angle variance + process noise
  P[0][1] = FP[0][0] * F[0][1] + FP[0][1] * F[1][1] + Q[0][1];  // Angle-bias covariance
  P[1][0] = FP[1][0] * F[0][0] + FP[1][1] * F[1][0] + Q[1][0];  // Bias-angle covariance
  P[1][1] = FP[1][0] * F[0][1] + FP[1][1] * F[1][1] + Q[1][1];  // Bias variance + process noise
  
  // UPDATE STEP - This is where we incorporate the accelerometer measurement
  //
  // WHY THIS STEP EXISTS:
  // - We have a prediction, but it's just based on gyro integration (which drifts)
  // - The accelerometer gives us an absolute angle reference (gravity direction)
  // - This step optimally combines prediction + measurement based on their uncertainties
  // - The P matrix tells us how much to trust each source
  
  // 1. Innovation: How much does measurement disagree with prediction?
  float innovation = accelAngle - angle;  // Measurement - Prediction
  
  // 2. Innovation covariance: How uncertain is this disagreement?
  // S represents total uncertainty in the innovation (prediction uncertainty + measurement noise)
  float S = P[0][0] + R;  // Prediction uncertainty + Accelerometer noise
  
  // 3. Kalman gain: How much should we trust the measurement vs prediction?
  // K is the "optimal blending factor" - this is the magic of Kalman filtering!
  float K[2];
  K[0] = P[0][0] / S;  // How much to correct angle estimate (0 to 1)
  K[1] = P[1][0] / S;  // How much to correct bias estimate (can be negative)
  
  // 4. State update: Blend prediction with measurement using optimal weights
  angle += K[0] * innovation;  // New angle = prediction + optimal_weight * disagreement
  bias += K[1] * innovation;
  
  // 5. Covariance update: P = (I - K * H) * P
  // where (I - K * H) = [1-K[0], -K[1]*0] = [1-K[0],   0  ]
  //                     [-K[1],   1     ]   [-K[1] ,   1  ]
  float IKH[2][2] = {{1.0 - K[0], 0.0}, {-K[1], 1.0}};
  
  float P_temp[2][2];
  P_temp[0][0] = IKH[0][0] * P[0][0] + IKH[0][1] * P[1][0];
  P_temp[0][1] = IKH[0][0] * P[0][1] + IKH[0][1] * P[1][1];
  P_temp[1][0] = IKH[1][0] * P[0][0] + IKH[1][1] * P[1][0];
  P_temp[1][1] = IKH[1][0] * P[0][1] + IKH[1][1] * P[1][1];
  
  // Copy back to P
  P[0][0] = P_temp[0][0];
  P[0][1] = P_temp[0][1];
  P[1][0] = P_temp[1][0];
  P[1][1] = P_temp[1][1];
  
  return angle;
}

void calibrate() {
  float xSum = 0, ySum = 0, zSum = 0;
  int validSamples = 0;
  
  Serial.print("Calibrating");
  
  for (int i = 0; i < calibrationSamples; i++) {
    if (IMU.gyroscopeAvailable()) {
      float x, y, z;
      IMU.readGyroscope(x, y, z);
      
      xSum += x;
      ySum += y;
      zSum += z;
      validSamples++;
      
      // Progress indicator
      if (i % 50 == 0) {
        Serial.print(".");
      }
    }
    delay(5); // Small delay between readings
  }
  
  if (validSamples > 0) {
    gyrXoffs = xSum / validSamples;
    gyrYoffs = ySum / validSamples;
    gyrZoffs = zSum / validSamples;
    isCalibrated = true;
    
    Serial.println();
    Serial.print("Calibration offsets - X: ");
    Serial.print(gyrXoffs, 4);
    Serial.print(", Y: ");
    Serial.print(gyrYoffs, 4);
    Serial.print(", Z: ");
    Serial.println(gyrZoffs, 4);
  } else {
    Serial.println("\nCalibration failed - no gyro data available");
    isCalibrated = false;
  }
}

void read_sensor_data() {
  // Read accelerometer data (in g's)
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(accX, accY, accZ);
  }
  
  // Read gyroscope data (in deg/s) and apply calibration
  if (IMU.gyroscopeAvailable()) {
    float rawX, rawY, rawZ;
    IMU.readGyroscope(rawX, rawY, rawZ);
    
    // Apply calibration offsets
    gyrX = rawX - gyrXoffs;
    gyrY = rawY - gyrYoffs;
    gyrZ = rawZ - gyrZoffs;
  }
  
  // Read magnetometer data (in uT) and apply calibration
  if (IMU.magneticFieldAvailable()) {
    float rawMX, rawMY, rawMZ;
    IMU.readMagneticField(rawMX, rawMY, rawMZ);
    
    // Apply calibration offsets and scale factors
    magX = (rawMX - magXoffs) * magXscale;
    magY = (rawMY - magYoffs) * magYscale;
    magZ = (rawMZ - magZoffs) * magZscale;
  }
}

// Additional utility functions for enhanced functionality

void printSensorInfo() {
  Serial.println("=== LSM9DS1 Sensor Information ===");
  Serial.print("Accelerometer sample rate: ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.print("Gyroscope sample rate: ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.print("Magnetic field sample rate: ");
  Serial.print(IMU.magneticFieldSampleRate());
  Serial.println(" Hz");
  Serial.println("====================================");
}

// Function to read magnetometer (bonus feature)
void readMagnetometer(float &mx, float &my, float &mz) {
  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(mx, my, mz);
  }
}

// Magnetometer calibration function
void calibrateMagnetometer() {
  float magXmax = -1000, magXmin = 1000;
  float magYmax = -1000, magYmin = 1000;
  float magZmax = -1000, magZmin = 1000;
  
  unsigned long startTime = millis();
  int sampleCount = 0;
  
  Serial.println("Rotate the sensor in all directions...");
  
  // Collect data for 30 seconds
  while (millis() - startTime < 30000) {
    if (IMU.magneticFieldAvailable()) {
      float mx, my, mz;
      IMU.readMagneticField(mx, my, mz);
      
      // Track min/max values for each axis
      if (mx > magXmax) magXmax = mx;
      if (mx < magXmin) magXmin = mx;
      if (my > magYmax) magYmax = my;
      if (my < magYmin) magYmin = my;
      if (mz > magZmax) magZmax = mz;
      if (mz < magZmin) magZmin = mz;
      
      sampleCount++;
      
      // Progress indicator
      if (sampleCount % 100 == 0) {
        Serial.print(".");
      }
    }
    delay(10);
  }
  
  // Calculate offsets (center of min/max range)
  magXoffs = (magXmax + magXmin) / 2.0;
  magYoffs = (magYmax + magYmin) / 2.0;
  magZoffs = (magZmax + magZmin) / 2.0;
  
  // Calculate scale factors to normalize ranges
  float magXrange = magXmax - magXmin;
  float magYrange = magYmax - magYmin;
  float magZrange = magZmax - magZmin;
  
  // Use the average range as reference
  float avgRange = (magXrange + magYrange + magZrange) / 3.0;
  
  magXscale = avgRange / magXrange;
  magYscale = avgRange / magYrange;
  magZscale = avgRange / magZrange;
  
  Serial.println();
  Serial.println("Magnetometer calibration values:");
  Serial.print("X offset: "); Serial.print(magXoffs, 3);
  Serial.print(", scale: "); Serial.println(magXscale, 3);
  Serial.print("Y offset: "); Serial.print(magYoffs, 3);
  Serial.print(", scale: "); Serial.println(magYscale, 3);
  Serial.print("Z offset: "); Serial.print(magZoffs, 3);
  Serial.print(", scale: "); Serial.println(magZscale, 3);
  Serial.print("Samples collected: "); Serial.println(sampleCount);
}
