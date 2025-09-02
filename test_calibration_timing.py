import serial
import time

def test_calibration_timing():
    """Test script to verify actual Arduino calibration timing"""
    
    # Try to find Arduino on common ports
    possible_ports = ['COM3', 'COM4', 'COM5', 'COM6', 'COM7', 'COM8']
    
    ser = None
    for port in possible_ports:
        try:
            ser = serial.Serial(port, 38400, timeout=1)
            time.sleep(2)  # Wait for Arduino to initialize
            print(f"Connected to Arduino on {port}")
            break
        except:
            continue
    
    if not ser:
        print("Could not connect to Arduino. Make sure it's connected and uploading the firmware.")
        return
    
    try:
        print("Testing magnetometer calibration timing...")
        print("Sending 'm' command...")
        
        # Send calibration command
        ser.write(b'm\n')
        
        start_time = time.time()
        calibration_started = False
        calibration_ended = False
        
        # Monitor for 35 seconds to capture full calibration
        while time.time() - start_time < 35:
            if ser.in_waiting > 0:
                try:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    current_time = time.time() - start_time
                    
                    if line:
                        if "CALIBRATING_MAG_START" in line:
                            calibration_started = True
                            print(f"[{current_time:.1f}s] CALIBRATION STARTED: {line}")
                        elif "CALIBRATING_MAG_DONE" in line:
                            calibration_ended = True
                            print(f"[{current_time:.1f}s] CALIBRATION ENDED: {line}")
                        elif line == ".":
                            print(".", end="", flush=True)
                        else:
                            print(f"[{current_time:.1f}s] {line}")
                    
                    if calibration_ended:
                        print(f"\nTotal calibration time: {current_time:.1f} seconds")
                        break
                        
                except UnicodeDecodeError:
                    continue
            
            time.sleep(0.01)
        
        if not calibration_started:
            print("ERROR: Calibration never started!")
        elif not calibration_ended:
            print("ERROR: Calibration started but never completed!")
            
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"Error during test: {e}")
    finally:
        if ser:
            ser.close()
            print("Serial connection closed")

if __name__ == "__main__":
    test_calibration_timing()
