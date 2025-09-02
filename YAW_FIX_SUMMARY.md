## Yaw Calculation Fix: Magnetic Environment Change Detection

### Problem Identified:
When moving the IMU from air to a flat surface, the magnetic environment changes, causing sudden yaw jumps in the simulation even though the physical orientation hasn't changed.

### Root Causes:
1. **Magnetic interference** from surfaces (metal, electronics, magnetic objects)
2. **Immediate magnetometer correction** applied when magnetic field changes
3. **No environment change detection** - system couldn't distinguish between intentional rotation and environmental magnetic changes

### Solutions Implemented:

#### 1. **Magnetic Environment Change Detection**
- **Heading Change Monitoring:** Detects sudden magnetometer heading changes >15°
- **Field Strength Monitoring:** Detects magnetic field strength changes >15%
- **Automatic Response:** Temporarily disables magnetometer corrections when environment changes

#### 2. **Adaptive Reference Update**
- **Gradual Adaptation:** After 3 seconds of stable readings, gradually updates magnetic reference
- **Smart Filtering:** Updates reference: 90% old + 10% new (gradual adaptation)
- **Prevents Jumps:** Avoids sudden orientation corrections

#### 3. **Enhanced Movement Detection**
- **Multiple Conditions:** Now considers both physical movement AND magnetic environment changes
- **Extended Protection:** Magnetometer corrections disabled during:
  - High angular movement (existing)
  - Magnetic environment changes (NEW)
  - Stabilization wait period (existing)

#### 4. **Improved Debug Information**
- **Environment Status:** Shows if magnetic environment has changed
- **Field Strength:** Displays current vs reference magnetic field strength
- **Correction Status:** Shows when magnetometer corrections are active/disabled

### Key Variables Added:
```cpp
float magPreviousHeading = 0;         // Previous magnetometer heading
float magFieldStrength = 0;           // Current magnetic field strength
float magReferenceFieldStrength = 0;  // Reference magnetic field strength
float magHeadingChangeThreshold = 15.0;  // Sudden heading change threshold (degrees)
float magFieldChangeThreshold = 0.15;    // Field strength change threshold (15%)
bool magEnvironmentChanged = false;      // Environment change flag
unsigned long magEnvironmentChangeTime = 0; // Time when change detected
```

### Benefits:
✅ **No sudden yaw jumps** when moving from air to surface  
✅ **Stable simulation** during environmental transitions  
✅ **Automatic adaptation** to new magnetic environments  
✅ **Maintains accuracy** for intentional rotations  
✅ **Debug visibility** for troubleshooting  

### Testing Scenarios:
1. **Air to Surface:** Should remain stable when placing IMU on desk
2. **Near Metal Objects:** Should temporarily ignore magnetometer when near interference
3. **Intentional Rotation:** Should still track actual yaw movements correctly
4. **Long-term Stability:** Should gradually adapt to new environments

### Commands for Testing:
- `'a'` - Show adaptive filter status including magnetic environment info
- `'z'` - Reset yaw and update magnetic reference to current environment
- `'.'` - Get current angles to verify stability
