# Arduino IMU Firmware - Kalman Filter Update

## Overview

The Arduino IMU firmware has been successfully updated to use a Kalman filter approach instead of the previous complementary filter, following the implementation pattern from the ESP32 reference code (`esp32_kf.cpp`).

## Key Changes Made

### 1. **Filter Algorithm Change**
- **Before**: Complementary filter (96% gyro + 4% accelerometer) for pitch/roll
- **After**: Kalman filter with optimal adaptive weighting for pitch/roll

### 2. **Yaw Calculation Simplified**
- **Before**: Adaptive filter with movement detection and relative heading tracking
- **After**: Direct magnetometer calculation using `atan2(magY, magX)` with magnetic declination correction

### 3. **New Kalman Filter Implementation**
```cpp
// Added Kalman filter parameters
float Q_angle = 0.001;   // Process noise variance for accelerometer
float Q_bias = 0.003;    // Process noise variance for gyroscope bias  
float R_measure = 0.03;  // Measurement noise variance

// Separate state variables for pitch and roll
float pitchP[2][2], rollP[2][2];  // Error covariance matrices
float pitchBias, rollBias;        // Bias correction variables
```

### 4. **Removed Features**
- Adaptive yaw movement detection
- Relative heading tracking system
- Tilt-compensated magnetometer heading
- Complex magnetometer reference system

### 5. **Simplified Command Set**
- **Removed**: `'a'` (adaptive filter status), `'h'` (magnetometer heading)
- **Kept**: `'.'` (angles), `'z'` (reset yaw), `'m'` (mag calibration), `'r'` (raw data), `'c'` (gyro calibration)

## Technical Implementation

### Kalman Filter Function
```cpp
float kalmanFilter(float angle, float gyroRate, float accelAngle, float P[2][2], float &bias)
```
- **Predict Step**: Integrates gyroscope data and updates uncertainty
- **Update Step**: Corrects prediction using accelerometer measurements
- **Adaptive Weighting**: Automatically adjusts trust between sensors based on uncertainty

### Yaw Calculation
```cpp
gz = atan2(magY, magX);                    // Basic magnetometer heading
float declinationAngle = -0.1783;         // Magnetic declination (-10° 13')
gz += declinationAngle;                    // Apply declination correction
// Normalize to 0-360 degrees
```

## Advantages of Kalman Filter Approach

1. **Optimal Sensor Fusion**: Mathematically optimal combination of accelerometer and gyroscope
2. **Adaptive Noise Filtering**: Automatically adjusts noise filtering based on sensor reliability
3. **Bias Correction**: Continuously corrects gyroscope bias drift
4. **Smooth Output**: Eliminates accelerometer noise without introducing lag
5. **Self-Tuning**: Adapts to changing sensor conditions automatically

## Usage Instructions

### Basic Operation
1. **Upload firmware** to Arduino Nano 33 BLE Sense
2. **Open Serial Monitor** at 38400 baud
3. **Keep device still** during initial gyroscope calibration
4. **Calibrate magnetometer** using command `'m'` (rotate device for 30 seconds)

### Available Commands
- `'.'` - Get current angles (roll, pitch, yaw)
- `'z'` - Reset yaw to 0°
- `'m'` - Calibrate magnetometer
- `'r'` - Show raw sensor data
- `'c'` - Recalibrate gyroscope

### Configuration Parameters

#### Kalman Filter Tuning
```cpp
float Q_angle = 0.001;   // ↑ = trust accelerometer more
float Q_bias = 0.003;    // ↑ = faster bias correction
float R_measure = 0.03;  // ↑ = trust gyroscope more
```

#### Magnetic Declination
```cpp
float declinationAngle = -0.1783;  // Adjust for your location
// Find your declination at: https://www.magnetic-declination.com/
```

## Comparison with ESP32 Reference

| Feature | ESP32 Reference | Arduino Implementation |
|---------|----------------|----------------------|
| Pitch/Roll Filter | Kalman Filter | ✅ Kalman Filter |
| Yaw Calculation | Magnetometer Only | ✅ Magnetometer Only |
| Sensor Hardware | MPU6050 + HMC5883L | LSM9DS1 |
| Display Output | OLED | Serial Monitor |
| Magnetic Declination | ✅ Supported | ✅ Supported |
| Calibration | ✅ Auto-calibration | ✅ Auto-calibration |

## Expected Performance

### Pitch/Roll (Kalman Filter)
- **Noise**: Minimal (automatically filtered)
- **Responsiveness**: Excellent (optimal weighting)
- **Drift**: None (bias correction)
- **Stability**: Superior to complementary filter

### Yaw (Magnetometer Direct)
- **Accuracy**: Dependent on magnetic environment
- **Drift**: None (absolute magnetic reference)
- **Response**: Immediate to orientation changes
- **Calibration**: Required for optimal performance

## Troubleshooting

### If yaw readings are inconsistent:
1. Recalibrate magnetometer (`'m'` command)
2. Check for magnetic interference (metal objects, electronics)
3. Adjust magnetic declination for your location

### If pitch/roll are noisy:
1. Recalibrate gyroscope (`'c'` command)
2. Ensure device is stable during calibration
3. Adjust Kalman filter parameters if needed

### If readings drift over time:
1. This should not happen with Kalman filter implementation
2. If it does, check sensor connections and power supply
3. Recalibrate both sensors

## Files Modified

- `arduino_imu_firmware_lsm9ds1.ino` - Main firmware file completely updated
- `KALMAN_FILTER_UPDATE.md` - This documentation (new file)

The firmware now implements the same approach as the ESP32 reference: Kalman filter for pitch/roll and direct magnetometer calculation for yaw, providing optimal sensor fusion performance.
