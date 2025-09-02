## Reset Robot Pose Feature

### Overview
The "Reset Robot Pose" button allows you to set the current IMU orientation as the reference pose for "robot facing right on a flat surface". This enables relative orientation tracking regardless of the actual physical IMU mounting orientation.

### How It Works

#### 1. **Setting Robot Reference**
- Click "🤖 Reset Robot Pose" button
- The GUI captures the current IMU angles (roll, pitch, yaw)
- These angles become the reference for "robot facing right on flat surface"
- All subsequent orientation display becomes relative to this reference

#### 2. **Relative Angle Calculation**
- **Relative Roll** = Current Roll - Reference Roll
- **Relative Pitch** = Current Pitch - Reference Pitch  
- **Relative Yaw** = Current Yaw - Reference Yaw (with wraparound handling)

#### 3. **Visual Feedback**
- **Simulation Display**: Shows relative orientation from reference pose
- **OSD Text**: Indicates "(Robot Relative)" vs "(Absolute)" mode
- **Console Output**: Shows reference angles when set

### Use Cases

#### **Scenario 1: Desktop Development**
- Place IMU on desk in any orientation
- Click "Reset Robot Pose" 
- Now tilt/rotate IMU → simulation shows robot movement relative to "flat surface"

#### **Scenario 2: Robot Integration** 
- Mount IMU on robot at any angle
- Position robot "facing right on flat surface"
- Click "Reset Robot Pose"
- Robot movements now display correctly in simulation

#### **Scenario 3: Calibration Verification**
- Set reference pose
- Manually position IMU to known orientations
- Verify simulation matches expected robot poses

### GUI Changes

#### **New Button**
- **Location**: Quick Commands panel
- **Text**: "🤖 Reset Robot Pose"
- **Action**: Captures current IMU orientation as reference

#### **Enhanced Display**
- **OSD Shows**: Relative angles when reference is set
- **Status Indicator**: "(Robot Relative)" or "(Absolute)"
- **Console Feedback**: Reference angles and status messages

### Technical Implementation

#### **Arduino Variables Added**
```cpp
float robotReferenceRoll = 0.0;   // Reference roll angle for robot pose
float robotReferencePitch = 0.0;  // Reference pitch angle for robot pose  
float robotReferenceYaw = 0.0;    // Reference yaw angle for robot pose
bool robotPoseSet = false;        // Flag indicating if robot reference is set
```

#### **Arduino Commands Added**
- **'p' command**: Sets current IMU angles as robot pose reference
- **'.' command enhanced**: Returns relative angles when robot pose is set

#### **GUI Variables Added**
```python
self.robot_reference_roll = 0.0    # Reference roll angle
self.robot_reference_pitch = 0.0   # Reference pitch angle  
self.robot_reference_yaw = 0.0     # Reference yaw angle
self.robot_pose_set = False        # Flag indicating if reference is set
```

#### **Methods Added**
- `reset_robot_pose()`: Captures current angles as reference
- `set_robot_reference()`: Updates simulation with reference pose
- Enhanced rendering: Applies relative transformations

#### **Communication Flow**
1. Button click → `send_quick_command("reset_pose")`
2. GUI sends `'.'` to Arduino → gets current angles
3. Parses response → sets reference variables
4. GUI sends `'p'` to Arduino → sets reference on Arduino side
5. Updates simulation → displays relative orientation

### Benefits
✅ **Flexible Mounting**: Works with any IMU orientation  
✅ **Intuitive Control**: "Robot facing right" makes sense visually  
✅ **Easy Recalibration**: One-click reference reset  
✅ **Visual Feedback**: Clear indication of relative vs absolute mode  
✅ **Synchronized**: Both Arduino and GUI use same reference  
✅ **Robust**: Reference persists even if GUI reconnects  

### Example Workflow
1. **Connect** to Arduino
2. **Start Simulation** 
3. **Position IMU** as desired "robot facing right on flat surface"
4. **Click "Reset Robot Pose"**
5. **Move IMU** → see robot movement relative to reference pose
6. **Re-click** anytime to update reference

### Arduino Integration
- Arduino automatically returns relative angles when robot pose is set
- Use `'p'` command directly from serial monitor to set robot reference
- Arduino handles yaw wraparound (±180°) correctly
- Reference persists until Arduino reset or new `'p'` command
