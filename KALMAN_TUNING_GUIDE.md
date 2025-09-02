# Kalman Filter Tuning Guide

## 🎛️ **Real-Time Parameter Tuning**

Your Arduino firmware now includes real-time Kalman filter tuning capabilities! This allows you to experiment and find the perfect balance for smooth, responsive movement tracking.

## **New Tuning Commands**

### **Display Current Parameters**
- **`'q'`** - Show current Kalman filter parameters and tuning commands

### **Real-Time Adjustment**
- **`'1'`** - Increase Q_angle (trust accelerometer more)
- **`'2'`** - Decrease Q_angle (trust accelerometer less)
- **`'3'`** - Increase Q_bias (faster bias correction)
- **`'4'`** - Decrease Q_bias (slower bias correction)
- **`'5'`** - Increase R_measure (trust gyroscope more)
- **`'6'`** - Decrease R_measure (trust gyroscope less)

## **Updated Starting Parameters**

The firmware now starts with more responsive settings:

```cpp
float Q_angle = 0.01;   // Was 0.001 - 10x more responsive
float Q_bias = 0.005;   // Was 0.003 - faster bias correction
float R_measure = 0.5;  // Was 0.03 - much higher gyro trust
```

## **Parameter Effects Explained**

### **Q_angle (Process Noise - Accelerometer)**
- **Higher values (0.01-0.1)** = More responsive to orientation changes, trusts accelerometer more
- **Lower values (0.001-0.01)** = Smoother but less responsive, filters out more accelerometer noise
- **🎯 For smooth rotation**: Try values between 0.005-0.02

### **Q_bias (Gyroscope Bias Correction)**
- **Higher values (0.005-0.05)** = Faster correction of gyroscope drift
- **Lower values (0.001-0.005)** = Slower, more stable bias correction
- **🎯 For stability**: Keep between 0.003-0.01

### **R_measure (Measurement Noise - Trust in Gyroscope)**
- **Higher values (0.1-5.0)** = Trust gyroscope more during movement (smoother rotation)
- **Lower values (0.01-0.1)** = Trust accelerometer more (can be jerky during movement)
- **🎯 For smooth rotation**: Try values between 0.3-1.0

## **Tuning Process for Smooth Rotation**

### **Step 1: Test Current Settings**
1. Upload the updated firmware
2. Send `'.'` command to monitor angles
3. Rotate the device slowly and observe smoothness

### **Step 2: If Rotation is Still Jerky**
Try this sequence:
```
'q'           // Check current values
'5' '5' '5'   // Increase R_measure (trust gyro more)
'1' '1'       // Slightly increase Q_angle
```

### **Step 3: If Too Sluggish**
```
'6' '6'       // Decrease R_measure
'1' '1' '1'   // Increase Q_angle (more responsive)
```

### **Step 4: Fine-tune for Your Application**
- **For gaming/VR**: Higher Q_angle (0.02-0.05), moderate R_measure (0.3-0.8)
- **For measurement**: Lower Q_angle (0.005-0.015), higher R_measure (0.5-2.0)
- **For stabilization**: Balanced settings (Q_angle: 0.01, R_measure: 0.5)

## **Recommended Starting Points for Smooth Rotation**

### **Gaming/Fast Response**
```
Q_angle = 0.02
Q_bias = 0.005
R_measure = 0.8
```

### **Smooth Measurement**
```
Q_angle = 0.015
Q_bias = 0.004
R_measure = 1.0
```

### **Ultra-Smooth (Current Default)**
```
Q_angle = 0.01
Q_bias = 0.005
R_measure = 0.5
```

## **Testing Protocol**

1. **Slow Rotation Test**: Rotate slowly - should be smooth without jitter
2. **Fast Rotation Test**: Quick movements - should follow without lag
3. **Stability Test**: Hold still - should not drift or oscillate
4. **Noise Test**: Gentle tapping - should filter out vibrations

## **Troubleshooting**

| Problem | Solution |
|---------|----------|
| Jerky during slow rotation | Increase R_measure (`'5'`) |
| Lags behind fast movement | Increase Q_angle (`'1'`) |
| Oscillates when still | Decrease Q_angle (`'2'`) |
| Drifts over time | Increase Q_bias (`'3'`) |
| Too sensitive to vibration | Decrease Q_angle (`'2'`) |

## **Save Your Settings**

Once you find the perfect parameters, note them down and update the firmware defaults:

```cpp
float Q_angle = YOUR_VALUE;   // Replace with your optimal value
float Q_bias = YOUR_VALUE;    // Replace with your optimal value  
float R_measure = YOUR_VALUE; // Replace with your optimal value
```

**Happy tuning!** 🎯 The real-time adjustment feature makes it easy to find the perfect balance for your specific application.
