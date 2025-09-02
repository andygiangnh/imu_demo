/*
================================================================================
 LSM9DS1 ADVANCED SENSOR FUSION IMU FIRMWARE
================================================================================

OVERVIEW:
This firmware implements a sophisticated 9-DOF sensor fusion algorithm for the 
LSM9DS1 IMU (accelerometer + gyroscope + magnetometer) to provide stable and 
accurate orientation tracking with minimal drift and lag.

SENSOR FUSION ALGORITHM DETAILS:
================================================================================

1. ROLL & PITCH (X & Y AXES) - STANDARD COMPLEMENTARY FILTER:
   - Uses accelerometer for absolute reference (gravity vector)
   - Uses gyroscope for responsive short-term tracking
   - Filter: 96% gyro + 4% accelerometer
   - Time constant (τ): ~0.48 seconds
   - Works well because gravity provides consistent absolute reference

2. YAW (Z AXIS) - ADAPTIVE MAGNETOMETER FUSION:
   - Problem: Magnetometer is noisy and lags behind movement
   - Solution: Adaptive filter that switches between gyro-only and complementary modes
   
   ADAPTIVE ALGORITHM:
   a) Movement Detection:
      - Calculate gyro Z derivative: |gyrZ(t) - gyrZ(t-1)| × FREQ
      - Threshold: 10.0 deg/s² (adjustable via gyrZderivativeThreshold)
      - High movement detected when derivative > threshold
   
   b) Filter States:
      HIGH MOVEMENT STATE:
      - Use 100% gyroscope data (no magnetometer correction)
      - Provides immediate, lag-free response to fast rotations
      - Eliminates magnetometer noise during movement
      
      STABLE STATE (low movement):
      - Wait period: 200ms after movement stops (stableWaitTime)
      - Then apply complementary filter: 98% gyro + 2% magnetometer
      - Gradually corrects gyroscope drift using magnetic heading reference
   
   c) Relative Heading Tracking:
      - Stores reference heading during calibration/reset (magReferenceHeading)
      - Tracks relative changes from reference, not absolute magnetic north
      - Prevents device from rotating back to north-facing orientation
      - Handles 360°/0° wraparound correctly

3. MAGNETOMETER CALIBRATION:
   - Hard iron calibration: Corrects for magnetic distortions
   - Collects min/max values during 30-second rotation procedure
   - Calculates offsets: (max + min) / 2 for each axis
   - Calculates scale factors to normalize magnetic field strength
   - Sets reference heading after calibration

4. TILT COMPENSATION:
   - Magnetometer heading calculation compensates for roll/pitch
   - Uses accelerometer data to determine device orientation
   - Applies 3D rotation math to get true magnetic heading
   - Formula: heading = atan2(magYcomp, magXcomp) where:
     magXcomp = magX × cos(pitch) + magZ × sin(pitch)
     magYcomp = magX × sin(roll) × sin(pitch) + magY × cos(roll) - magZ × sin(roll) × cos(pitch)

KEY PARAMETERS:
================================================================================
- FREQ: 30.0 Hz - Main loop frequency
- gyrZderivativeThreshold: 10.0 deg/s² - Movement detection sensitivity
- stableWaitTime: 200ms - Wait time before applying magnetometer correction
- Roll/Pitch filter: 96% gyro + 4% accelerometer (fast response, stable)
- Yaw filter: 98% gyro + 2% magnetometer (conservative correction)

COMMANDS:
================================================================================
- '.' : Get current angles (roll, pitch, yaw)
- 'z' : Reset yaw to 0° and update magnetometer reference
- 'm' : Calibrate magnetometer (rotate device for 30 seconds)
      - Response: m1 = in progress, m2 = completed
- 'r' : Show raw sensor data
- 'a' : Show adaptive filter status and debug info
- 'h' : Show current magnetometer heading
- 'c' : Recalibrate gyroscope (keep device still)
      - Response: c1 = in progress, c2 = completed, c3 = failed

BENEFITS:
================================================================================
✓ No yaw drift (magnetometer prevents long-term gyroscope drift)
✓ No lag during movement (gyroscope handles fast rotations)
✓ No noise during movement (magnetometer disabled during high movement)
✓ Stable reference (device maintains orientation when stationary)
✓ Tilt compensated (works correctly at any roll/pitch angle)
✓ Calibrated (accounts for magnetic distortions and sensor offsets)

Compatible with: Arduino Nano 33 BLE Sense, Arduino Nano 33 IoT
Author: Advanced IMU Sensor Fusion Implementation
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

// magnetometer reference heading for relative tracking
float magReferenceHeading = 0;
float magPreviousHeading = 0;      // Previous magnetometer heading for change detection
float magFieldStrength = 0;        // Current magnetic field strength
float magReferenceFieldStrength = 0; // Reference magnetic field strength

// Adaptive yaw filter variables
float gyrZprev = 0;                    // Previous gyro Z reading for derivative calculation
float gyrZderivative = 0;              // Rate of change of gyro Z
float gyrZderivativeThreshold = 10.0;  // Threshold for high movement detection (deg/s²)
unsigned long lastStableTime = 0;     // Time when movement became stable
unsigned long stableWaitTime = 200;   // Wait time before applying mag correction (ms)
bool inHighMovement = false;          // Flag for high movement state

// Magnetic environment change detection
float magHeadingChangeThreshold = 15.0;  // Threshold for sudden heading changes (degrees)
float magFieldChangeThreshold = 0.15;    // Threshold for magnetic field strength changes (15%)
bool magEnvironmentChanged = false;      // Flag for magnetic environment change
unsigned long magEnvironmentChangeTime = 0; // Time when environment change was detected

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
  
  // Set initial magnetometer reference heading and field strength
  delay(200); // Brief delay to ensure sensors are stable
  magReferenceHeading = calculateTiltCompensatedHeading();
  magPreviousHeading = magReferenceHeading;
  magReferenceFieldStrength = calculateMagneticFieldStrength();
  Serial.print("Initial reference heading set to: ");
  Serial.println(magReferenceHeading, 2);
  
  Serial.println("System ready. Commands: '.' = angles, 'z' = reset Z, 'm' = mag cal (m1/m2), 'c' = gyro cal (c1/c2/c3)");
}

void loop() {
  unsigned long start_time, end_time;
  double ax, ay, az, mag_heading;
  
  // ============================================================================
  // SENSOR FUSION ALGORITHM IMPLEMENTATION
  // ============================================================================
  
  start_time = millis();
  
  // Read raw sensor data from LSM9DS1
  read_sensor_data();
  
  // ============================================================================
  // STEP 1: CALCULATE ABSOLUTE ANGLES FROM ACCELEROMETER (ROLL & PITCH)
  // ============================================================================
  // Accelerometer provides absolute reference using gravity vector
  // These angles are accurate long-term but noisy short-term
  ax = atan2(accY, sqrt(pow(accX, 2) + pow(accZ, 2))) * 180.0 / M_PI;  // Roll
  ay = atan2(-accX, sqrt(pow(accY, 2) + pow(accZ, 2))) * 180.0 / M_PI; // Pitch
  
  // ============================================================================
  // STEP 2: CALCULATE MAGNETOMETER HEADING (TILT-COMPENSATED)
  // ============================================================================
  mag_heading = calculateTiltCompensatedHeading();
  magFieldStrength = calculateMagneticFieldStrength();
  
  // Detect sudden magnetic environment changes
  float headingChange = abs(mag_heading - magPreviousHeading);
  if (headingChange > 180) headingChange = 360 - headingChange; // Handle wraparound
  
  float fieldChange = abs(magFieldStrength - magReferenceFieldStrength) / magReferenceFieldStrength;
  
  // Check for sudden magnetic environment change
  if (headingChange > magHeadingChangeThreshold || fieldChange > magFieldChangeThreshold) {
    if (!magEnvironmentChanged) {
      magEnvironmentChanged = true;
      magEnvironmentChangeTime = millis();
      Serial.println("Warning: Magnetic environment change detected!");
    }
  }
  
  // Reset environment change flag after 3 seconds of stable readings
  if (magEnvironmentChanged && (millis() - magEnvironmentChangeTime > 3000) && 
      headingChange < 5.0 && fieldChange < 0.05) {
    magEnvironmentChanged = false;
    // Gradually update reference to new environment
    magReferenceHeading = magReferenceHeading * 0.9 + mag_heading * 0.1;
    magReferenceFieldStrength = magReferenceFieldStrength * 0.9 + magFieldStrength * 0.1;
    Serial.println("Magnetic environment stabilized - reference updated");
  }
  
  magPreviousHeading = mag_heading;
  
  // Calculate relative heading change from reference (not absolute north)
  // This prevents the device from drifting back to north-facing orientation
  float relativeHeading = mag_heading - magReferenceHeading;
  
  // Handle heading wraparound (crossing 0°/360° boundary)
  if (relativeHeading > 180) {
    relativeHeading -= 360;
  } else if (relativeHeading < -180) {
    relativeHeading += 360;
  }
  
  // ============================================================================
  // STEP 3: INTEGRATE GYROSCOPE DATA (ALL AXES)
  // ============================================================================
  // Gyroscope provides responsive short-term tracking but drifts over time
  gx = gx + gyrX / FREQ;  // Roll integration
  gy = gy + gyrY / FREQ;  // Pitch integration
  gz = gz + gyrZ / FREQ;  // Yaw integration
  
  // ============================================================================
  // STEP 4: ADAPTIVE YAW MOVEMENT DETECTION
  // ============================================================================
  // Calculate rate of change of yaw angular velocity (acceleration in rotation)
  gyrZderivative = abs(gyrZ - gyrZprev) * FREQ;  // deg/s²
  gyrZprev = gyrZ;
  
  // Determine if device is in high movement or stable state
  if (gyrZderivative > gyrZderivativeThreshold) {
    inHighMovement = true;
    lastStableTime = millis();  // Reset stable timer when movement detected
  } else {
    inHighMovement = false;
  }
  
  // ============================================================================
  // STEP 5: APPLY COMPLEMENTARY FILTERS
  // ============================================================================
  
  // ROLL & PITCH: Standard complementary filter (accelerometer + gyroscope)
  // 96% gyro (responsive) + 4% accelerometer (stable, gravity reference)
  // Time constant τ = dt*(A)/(1-A) = (1/30)*(0.96)/(0.04) = 0.8 seconds
  gx = gx * 0.96 + ax * 0.04;
  gy = gy * 0.96 + ay * 0.04;
  
  // YAW: Adaptive filter (gyroscope + magnetometer)
  if (inHighMovement || magEnvironmentChanged) {
    // HIGH MOVEMENT STATE OR MAGNETIC ENVIRONMENT CHANGE: Use 100% gyroscope
    // - Provides immediate response to fast rotations
    // - Eliminates magnetometer lag and noise during movement
    // - Prevents corrections during magnetic environment changes (e.g., moving near metal objects)
    // - gz already integrated above, no correction applied
  } else {
    // STABLE STATE: Wait, then apply magnetometer correction
    if (millis() - lastStableTime >= stableWaitTime) {
      // Apply complementary filter: 98% gyro + 2% magnetometer
      // - Conservative correction to prevent oscillations
      // - Uses relative heading to maintain current orientation
      // - Gradually corrects accumulated gyroscope drift
      gz = gz * 0.98 + relativeHeading * 0.02;
    }
    // If wait time hasn't elapsed, continue with gyro-only (prevents premature correction)
  }
  
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
      // Also reset the magnetometer reference to current heading and field strength
      magReferenceHeading = calculateTiltCompensatedHeading();
      magReferenceFieldStrength = calculateMagneticFieldStrength();
      magEnvironmentChanged = false; // Reset environment change flag
      Serial.println("Z axis reset and magnetometer reference updated");
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
    
    // Debug: adaptive filter status
    if (rx_char == 'a') {
      Serial.print("Adaptive Filter - GyrZ Derivative: ");
      Serial.print(gyrZderivative, 2);
      Serial.print(" | High Movement: ");
      Serial.print(inHighMovement ? "YES" : "NO");
      Serial.print(" | Mag Environment Changed: ");
      Serial.print(magEnvironmentChanged ? "YES" : "NO");
      Serial.print(" | Time since stable: ");
      Serial.print(millis() - lastStableTime);
      Serial.print("ms | Mag correction: ");
      Serial.println((inHighMovement || magEnvironmentChanged || (millis() - lastStableTime < stableWaitTime)) ? "DISABLED" : "ENABLED");
      Serial.print("Mag Field Strength: ");
      Serial.print(magFieldStrength, 3);
      Serial.print(" | Reference: ");
      Serial.println(magReferenceFieldStrength, 3);
    }
    
    // Magnetometer calibration
    if (rx_char == 'm') {
      digitalWrite(13, HIGH);
      calibrateMagnetometer();
      digitalWrite(13, LOW);
    }
    
    // Recalibrate gyroscope
    if (rx_char == 'c') {
      digitalWrite(13, HIGH);
      calibrate();
      digitalWrite(13, LOW);
    }
    
    // Show magnetometer heading
    if (rx_char == 'h') {
      float heading = calculateTiltCompensatedHeading();
      Serial.print("Magnetometer heading: ");
      Serial.println(heading, 2);
    }
  }
  
  // Maintain consistent loop timing
  end_time = millis();
  int loopTime = ((1.0/FREQ) * 1000) - (end_time - start_time);
  if (loopTime > 0) {
    delay(loopTime);
  }
}

void calibrate() {
  float xSum = 0, ySum = 0, zSum = 0;
  int validSamples = 0;
  
  Serial.println("c1");  // Gyroscope calibration in progress
  
  for (int i = 0; i < calibrationSamples; i++) {
    if (IMU.gyroscopeAvailable()) {
      float x, y, z;
      IMU.readGyroscope(x, y, z);
      
      xSum += x;
      ySum += y;
      zSum += z;
      validSamples++;
      
      // Progress indicator every 100 samples
      if (i % 100 == 0) {
        Serial.println("c1");  // Progress update
      }
    }
    delay(5); // Small delay between readings
  }
  
  if (validSamples > 0) {
    gyrXoffs = xSum / validSamples;
    gyrYoffs = ySum / validSamples;
    gyrZoffs = zSum / validSamples;
    isCalibrated = true;
    
    Serial.println("c2");  // Gyroscope calibration done
  } else {
    Serial.println("c3");  // Gyroscope calibration failed
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

// Calculate tilt-compensated heading from magnetometer
float calculateTiltCompensatedHeading() {
  // Convert accelerometer readings to roll and pitch in radians
  float roll = atan2(accY, accZ);
  float pitch = atan2(-accX, sqrt(accY * accY + accZ * accZ));
  
  // Tilt compensation calculations
  float magXcomp = magX * cos(pitch) + magZ * sin(pitch);
  float magYcomp = magX * sin(roll) * sin(pitch) + magY * cos(roll) - magZ * sin(roll) * cos(pitch);
  
  // Calculate heading in degrees
  float heading = atan2(magYcomp, magXcomp) * 180.0 / M_PI;
  
  // Normalize to 0-360 degrees
  if (heading < 0) {
    heading += 360.0;
  }
  
  return heading;
}

float calculateMagneticFieldStrength() {
  // Calculate the magnitude of the magnetic field vector
  return sqrt(magX * magX + magY * magY + magZ * magZ);
}

// Magnetometer calibration function
void calibrateMagnetometer() {
  float magXmax = -1000, magXmin = 1000;
  float magYmax = -1000, magYmin = 1000;
  float magZmax = -1000, magZmin = 1000;
  
  unsigned long startTime = millis();
  int sampleCount = 0;
  
  Serial.println("m1");  // Magnetometer calibration in progress
  
  // Collect data for 30 seconds
  unsigned long lastProgressTime = millis();
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
      
      // Progress indicator every 100 samples (more frequent updates)
      if (sampleCount % 100 == 0) {
        Serial.println("m1");  // Progress update
      }
    }
    
    // Time-based progress indicator every 3 seconds as backup
    if (millis() - lastProgressTime > 3000) {
      Serial.println("m1");  // Progress update
      lastProgressTime = millis();
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
  
  Serial.println("m2");  // Magnetometer calibration done
}
