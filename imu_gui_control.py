#!/usr/bin/env python

"""
IMU Control GUI with embedded Pygame simulation
Combines the 3D IMU visualization with a control panel for sending commands to Arduino
"""

import tkinter as tk
from tkinter import ttk, scrolledtext
import threading
import time
import os
import sys

# OpenGL and Pygame imports
from OpenGL.GL import *
from OpenGL.GLU import *
import pygame
from pygame.locals import *
import serial

class IMUSimulation:
    """Class to handle the pygame 3D simulation"""
    
    def __init__(self, serial_connection):
        self.ser = serial_connection
        self.ax = self.ay = self.az = 0.0
        self.yaw_mode = True
        self.calibration_status = "waiting"  # "waiting", "calibrating", "complete"
        self.calibration_message = ""
        self.show_simulation = False
        self.running = False
        self.screen = None
        
        # Robot pose reference variables
        self.robot_reference_roll = 0.0
        self.robot_reference_pitch = 0.0
        self.robot_reference_yaw = 0.0
        self.robot_pose_set = False
        
    def init_pygame(self):
        """Initialize pygame in separate window"""
        pygame.init()
        video_flags = OPENGL | DOUBLEBUF
        self.screen = pygame.display.set_mode((640, 480), video_flags)
        pygame.display.set_caption("IMU 3D Visualization - Press ESC to close")
        self.resize(640, 480)
        self.init_opengl()
        return True
    
    def resize(self, width, height):
        if height == 0:
            height = 1
        glViewport(0, 0, width, height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, 1.0*width/height, 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

    def init_opengl(self):
        glShadeModel(GL_SMOOTH)
        glClearColor(0.0, 0.0, 0.0, 0.0)
        glClearDepth(1.0)
        glEnable(GL_DEPTH_TEST)
        glDepthFunc(GL_LEQUAL)
        glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)

    def draw_text(self, position, textString):     
        font = pygame.font.SysFont("Courier", 18, True)
        textSurface = font.render(textString, True, (255,255,255,255), (0,0,0,255))     
        textData = pygame.image.tostring(textSurface, "RGBA", True)     
        glRasterPos3d(*position)     
        glDrawPixels(textSurface.get_width(), textSurface.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, textData)

    def draw(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        
        glLoadIdentity()
        glTranslatef(0, 0.0, -7.0)

        # Display calibration status or normal operation
        if self.calibration_status == "calibrating":
            # Show calibration message in center of screen
            self.draw_text((-1.5, 0, 2), "CALIBRATING...")
            if self.calibration_message:
                self.draw_text((-2, -0.5, 2), self.calibration_message)
            self.draw_text((-2, -1, 2), "Please wait...")
            return  # Don't draw the cube during calibration
        
        elif self.calibration_status == "complete" and not self.show_simulation:
            # Show calibration complete message
            self.draw_text((-1.5, 0, 2), "CALIBRATION COMPLETE!")
            self.draw_text((-2, -0.5, 2), "Starting simulation...")
            # Set flag to start showing simulation after a brief delay
            time.sleep(0.1)  # Brief delay
            self.show_simulation = True
            return
        
        # Normal simulation display
        if self.show_simulation or self.calibration_status == "waiting":
            osd_text = f"pitch: {self.ay:.2f}, roll: {self.ax:.2f}"

            if self.yaw_mode:
                osd_line = osd_text + f", yaw: {self.az:.2f}"
            else:
                osd_line = osd_text
            
            # Add robot pose mode indicator
            if self.robot_pose_set:
                osd_line += " (Robot Relative)"
            else:
                osd_line += " (Absolute)"

            self.draw_text((-2, -2, 2), osd_line)

            # Draw the 3D cube
            # the way I'm holding the IMU board, X and Y axis are switched 
            # with respect to the OpenGL coordinate system
            if self.yaw_mode:
                glRotatef(self.az, 0.0, 1.0, 0.0)  # Yaw, rotate around y-axis
            else:
                glRotatef(0.0, 0.0, 1.0, 0.0)
            glRotatef(self.ay, 1.0, 0.0, 0.0)        # Pitch, rotate around x-axis
            glRotatef(-1*self.ax, 0.0, 0.0, 1.0)     # Roll, rotate around z-axis

            self.draw_cube()

    def draw_cube(self):
        glBegin(GL_QUADS)
        # Top face (green)
        glColor3f(0.0, 1.0, 0.0)
        glVertex3f( 1.0, 0.2, -1.0)
        glVertex3f(-1.0, 0.2, -1.0)		
        glVertex3f(-1.0, 0.2,  1.0)		
        glVertex3f( 1.0, 0.2,  1.0)		

        # Bottom face (orange)
        glColor3f(1.0, 0.5, 0.0)	
        glVertex3f( 1.0, -0.2,  1.0)
        glVertex3f(-1.0, -0.2,  1.0)		
        glVertex3f(-1.0, -0.2, -1.0)		
        glVertex3f( 1.0, -0.2, -1.0)		

        # Front face (red)
        glColor3f(1.0, 0.0, 0.0)		
        glVertex3f( 1.0,  0.2,  1.0)
        glVertex3f(-1.0,  0.2,  1.0)		
        glVertex3f(-1.0, -0.2,  1.0)		
        glVertex3f( 1.0, -0.2,  1.0)		

        # Back face (yellow)
        glColor3f(1.0, 1.0, 0.0)	
        glVertex3f( 1.0, -0.2, -1.0)
        glVertex3f(-1.0, -0.2, -1.0)
        glVertex3f(-1.0,  0.2, -1.0)		
        glVertex3f( 1.0,  0.2, -1.0)		

        # Left face (blue)
        glColor3f(0.0, 0.0, 1.0)	
        glVertex3f(-1.0,  0.2,  1.0)
        glVertex3f(-1.0,  0.2, -1.0)		
        glVertex3f(-1.0, -0.2, -1.0)		
        glVertex3f(-1.0, -0.2,  1.0)		

        # Right face (magenta)
        glColor3f(1.0, 0.0, 1.0)	
        glVertex3f( 1.0,  0.2, -1.0)
        glVertex3f( 1.0,  0.2,  1.0)
        glVertex3f( 1.0, -0.2,  1.0)		
        glVertex3f( 1.0, -0.2, -1.0)		
        glEnd()

    def read_data(self):
        """Read data from serial port and update angles"""
        try:
            # Request data by sending a dot
            self.ser.write(b".")
            line = self.ser.readline()
            
            # Skip empty lines
            if not line or line.strip() == b'':
                return
                
            line_str = line.decode().strip()
            
            # Check for calibration-related messages and update status
            if "Calibrating" in line_str or "Keep sensor still" in line_str:
                self.calibration_status = "calibrating"
                self.calibration_message = "Gyroscope calibration..."
                return
            elif "Starting magnetometer calibration" in line_str or "Rotate the sensor" in line_str:
                self.calibration_status = "calibrating"
                self.calibration_message = "Magnetometer calibration..."
                return
            elif "Calibration complete" in line_str or "System ready" in line_str:
                self.calibration_status = "complete"
                self.calibration_message = ""
                return
            elif "Magnetometer calibration complete" in line_str:
                self.calibration_status = "complete"
                self.calibration_message = ""
                return
            
            # List of message patterns to skip
            skip_patterns = [
                'Calibration', 'offsets', 'Hello', 'IMU initialized',
                'sample rate', 'Hz', 'complete', 'Starting', 'Rotate', 'values',
                'Samples collected', 'Magnetometer', 'heading', 'axis reset'
            ]
            
            # Check if line contains any skip patterns
            if any(pattern in line_str for pattern in skip_patterns):
                return
            
            # Try to parse as angle data (should be three comma-separated numbers)
            angles = line.split(b", ")
            if len(angles) == 3:
                # Additional check: ensure all parts look like numbers
                for angle in angles:
                    angle_str = angle.decode().strip()
                    if not all(c.isdigit() or c in '.-+ \r\n\t' for c in angle_str):
                        return
                
                self.ax = float(angles[0])
                self.ay = float(angles[1])
                self.az = float(angles[2])
                
                # If we successfully parsed angle data and haven't shown simulation yet,
                # it means calibration is complete and we're getting real data
                if not self.show_simulation and self.calibration_status != "calibrating":
                    self.calibration_status = "complete"
                    self.show_simulation = True
                    
        except Exception as e:
            print(f"Error reading data: {e}")

    def toggle_yaw_mode(self):
        """Toggle yaw mode and send command to Arduino"""
        self.yaw_mode = not self.yaw_mode
        try:
            if self.ser and self.ser.is_open:
                self.ser.write(b"z")
        except Exception as e:
            print(f"Error sending yaw toggle command: {e}")

    def set_robot_reference(self, roll_ref, pitch_ref, yaw_ref):
        """Set robot pose reference for relative angle calculations"""
        self.robot_reference_roll = roll_ref
        self.robot_reference_pitch = pitch_ref
        self.robot_reference_yaw = yaw_ref
        self.robot_pose_set = True

    def stop(self):
        """Stop the simulation"""
        self.running = False

    def run_simulation_loop(self):
        """Main simulation loop - runs in separate thread"""
        try:
            if not self.init_pygame():
                return
            
            clock = pygame.time.Clock()
            
            while self.running:
                # Handle pygame events
                for event in pygame.event.get():
                    if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
                        self.running = False
                        break
                
                # Read data and update simulation
                if self.ser and self.ser.is_open:
                    self.read_data()
                
                # Draw the simulation
                self.draw()
                pygame.display.flip()
                
                # Control frame rate
                clock.tick(30)  # 30 FPS
                
        except Exception as e:
            print(f"Simulation error: {e}")
        finally:
            pygame.quit()
            self.running = False


class IMUControlGUI:
    """Main GUI application class"""
    
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("IMU Control Panel - 3D Visualization")
        self.root.geometry("900x700")
        self.root.minsize(800, 600)  # Ensure minimum size to show all elements
        
        # Serial connection
        self.serial_port = 'COM4'  # Change as needed
        self.baud_rate = 38400
        self.ser = None
        self.simulation = None
        
        # GUI state
        self.simulation_running = False
        self.pygame_thread = None
        self.calibration_status = "idle"  # "idle", "gyro", "mag"
        self.calibration_thread = None
        
        # Robot pose reference variables
        self.robot_reference_roll = 0.0
        self.robot_reference_pitch = 0.0
        self.robot_reference_yaw = 0.0
        self.robot_pose_set = False
        
        self.setup_gui()
        # Show initial instructions
        self.log_response("=== IMU Control Panel ===")
        self.log_response("STEP 1: Click 'Reconnect' to connect to Arduino")
        self.log_response("STEP 2: Once connected, 'Start Simulation' button will be enabled")
        self.log_response("STEP 3: Click 'Start Simulation' to open PyGame window")
        self.log_response("STEP 4: Use control panel to send commands")
        self.log_response("")
        self.log_response("Note: Buttons are visible but disabled until connected")
        
    def setup_gui(self):
        """Create the GUI layout"""
        # Main container
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Left side - Status and info
        info_frame = ttk.LabelFrame(main_frame, text="Simulation Status", padding=10)
        info_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        # Status display
        self.status_label = ttk.Label(info_frame, text="Status: Ready\n1. Connect to Arduino\n2. Start Simulation\n3. PyGame window will open", 
                                     justify=tk.CENTER, font=('Arial', 11))
        self.status_label.pack(pady=20)
        
        # Current angles display
        self.angles_frame = ttk.LabelFrame(info_frame, text="Current Angles", padding=10)
        self.angles_frame.pack(fill=tk.X, pady=10)
        
        self.roll_label = ttk.Label(self.angles_frame, text="Roll: 0.00°", font=('Courier', 11))
        self.roll_label.pack()
        self.pitch_label = ttk.Label(self.angles_frame, text="Pitch: 0.00°", font=('Courier', 11))
        self.pitch_label.pack()
        self.yaw_label = ttk.Label(self.angles_frame, text="Yaw: 0.00°", font=('Courier', 11))
        self.yaw_label.pack()
        
        # Right side - Control panel
        control_frame = ttk.LabelFrame(main_frame, text="Control Panel", padding=10)
        control_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=(10, 0))
        control_frame.configure(width=350)
        control_frame.pack_propagate(False)
        
        # Serial connection info
        conn_frame = ttk.LabelFrame(control_frame, text="Connection", padding=5)
        conn_frame.pack(fill=tk.X, pady=(0, 10))
        
        self.connection_status = ttk.Label(conn_frame, text="Disconnected", foreground="red")
        self.connection_status.pack()
        
        ttk.Button(conn_frame, text="Reconnect", command=self.connect_serial).pack(pady=5)
        
        # Command input
        cmd_frame = ttk.LabelFrame(control_frame, text="Send Command", padding=5)
        cmd_frame.pack(fill=tk.X, pady=(0, 10))
        
        self.command_var = tk.StringVar()
        self.command_entry = ttk.Entry(cmd_frame, textvariable=self.command_var, width=30)
        self.command_entry.pack(fill=tk.X, pady=(0, 5))
        self.command_entry.bind('<Return>', self.send_command)
        
        ttk.Button(cmd_frame, text="Send", command=self.send_command).pack()
        
        # Quick commands
        quick_frame = ttk.LabelFrame(control_frame, text="Quick Commands", padding=5)
        quick_frame.pack(fill=tk.X, pady=(0, 10))
        
        # Store button references for state management
        self.command_buttons = {}
        
        commands = [
            ("Get Angles", "."),
            ("Reset Yaw", "z"),
            ("Raw Data", "r"),
            ("Heading", "h"),
            ("🧭 Cal Mag", "m"),
            ("🔄 Cal Gyro", "c"),
            ("🤖 Reset Robot Pose", "reset_pose")
        ]
        
        for i, (text, cmd) in enumerate(commands):
            row = i // 2
            col = i % 2
            # Fix lambda closure issue by using default parameter
            btn = ttk.Button(quick_frame, text=text, 
                           command=lambda c=cmd: self.send_quick_command(c))
            btn.grid(row=row, column=col, padx=2, pady=2, sticky="ew")
            self.command_buttons[cmd] = btn  # Store reference for state management
        
        # Configure grid weights for even spacing
        for col in range(2):
            quick_frame.columnconfigure(col, weight=1)
        
        # Control buttons - Place BEFORE response display to ensure visibility
        button_frame = ttk.LabelFrame(control_frame, text="Simulation Control", padding=10)
        button_frame.pack(fill=tk.X, pady=(0, 10))
        
        # Create button sub-frame for better layout
        btn_container = ttk.Frame(button_frame)
        btn_container.pack(fill=tk.X)
        
        self.start_btn = ttk.Button(btn_container, text="▶ Start Simulation", command=self.start_simulation, width=15)
        self.start_btn.pack(side=tk.LEFT, padx=(0, 5))
        self.start_btn.configure(state='disabled')  # Disabled until connected
        
        self.stop_btn = ttk.Button(btn_container, text="⏹ Stop Simulation", command=self.stop_simulation, width=15)
        self.stop_btn.pack(side=tk.LEFT, padx=(0, 5))
        self.stop_btn.configure(state='disabled')
        
        self.yaw_btn = ttk.Button(btn_container, text="🔄 Toggle Yaw", command=self.toggle_yaw, width=12)
        self.yaw_btn.pack(side=tk.RIGHT)
        self.yaw_btn.configure(state='disabled')  # Disabled until simulation starts
        
        # Emergency controls
        emergency_frame = ttk.Frame(button_frame)
        emergency_frame.pack(fill=tk.X, pady=(5, 0))
        
        ttk.Button(emergency_frame, text="🔓 Enable All Buttons", 
                  command=self.enable_command_buttons).pack(side=tk.RIGHT)
        
        # Response display - Place AFTER buttons so it doesn't hide them
        response_frame = ttk.LabelFrame(control_frame, text="Arduino Response", padding=5)
        response_frame.pack(fill=tk.BOTH, expand=True)
        
        self.response_text = scrolledtext.ScrolledText(response_frame, height=12, width=40)
        self.response_text.pack(fill=tk.BOTH, expand=True)
        
        # Start angle update timer
        self.update_angles()
        
    def connect_serial(self):
        """Connect to the Arduino via serial port"""
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
            
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            
            # Clear any buffered data and wait for Arduino to be ready
            time.sleep(2)  # Wait for Arduino to initialize
            self.ser.flushInput()  # Clear input buffer
            self.ser.flushOutput()  # Clear output buffer
            
            self.connection_status.configure(text=f"Connected to {self.serial_port}", foreground="green")
            self.log_response(f"Connected to {self.serial_port} at {self.baud_rate} baud")
            
            # Enable start button when connected
            self.start_btn.configure(state='normal')
            self.status_label.configure(text="Status: Connected\nReady to start simulation\nClick 'Start Simulation' below")
            
            # Log button status for debugging
            self.log_response(f"✓ Connected! Start Simulation button is now ENABLED")
            self.log_response(f"Button state: {self.start_btn['state']}")
            
        except Exception as e:
            self.connection_status.configure(text="Connection Failed", foreground="red")
            self.log_response(f"Failed to connect: {e}")
            self.start_btn.configure(state='disabled')
            self.status_label.configure(text="Status: Connection Failed\nCheck COM port and try again\nArduino must be connected")
            
            # Log button status for debugging
            self.log_response(f"✗ Connection failed! Start Simulation button is DISABLED")
            self.log_response(f"Button state: {self.start_btn['state']}")
    
    def send_command(self, event=None):
        """Send command from the entry field"""
        command = self.command_var.get().strip()
        if command and self.ser:
            self.send_quick_command(command)
            self.command_var.set("")  # Clear the entry field
    
    def send_quick_command(self, command):
        """Send a quick command to Arduino"""
        if not self.ser:
            self.log_response("Error: Not connected to Arduino")
            return
        
        # Handle special commands
        if command == "reset_pose":
            self.reset_robot_pose()
            return
            
        try:
            self.ser.write(command.encode())
            self.log_response(f"Sent: '{command}'")  # Added quotes for clarity
            
            # Handle calibration commands specially
            if command == 'c':
                self.log_response("→ Starting gyroscope calibration monitoring...")
                self.start_calibration_monitoring("gyro")
            elif command == 'm':
                self.log_response("→ Starting magnetometer calibration monitoring...")
                self.start_calibration_monitoring("mag")
            # For commands that expect a response, read it
            elif command in ['.', 'r', 'h', 'a']:
                threading.Thread(target=self.read_response, daemon=True).start()
                
        except Exception as e:
            self.log_response(f"Error sending command: {e}")
    
    def disable_command_buttons(self):
        """Disable all command buttons during calibration"""
        for btn in self.command_buttons.values():
            btn.configure(state='disabled')
            
    def enable_command_buttons(self):
        """Enable all command buttons after calibration"""
        for btn in self.command_buttons.values():
            btn.configure(state='normal')
    
    def reset_robot_pose(self):
        """Reset robot pose reference to current IMU orientation"""
        if not self.ser:
            self.log_response("Error: Not connected to Arduino")
            return
        
        try:
            # Request current angles from Arduino
            self.ser.write(b'.')
            time.sleep(0.1)  # Wait for response
            
            # Read the response
            if self.ser.in_waiting > 0:
                response = self.ser.readline().decode().strip()
                if response:
                    try:
                        # Parse the angles (format: "roll, pitch, yaw")
                        angles = [float(x.strip()) for x in response.split(',')]
                        if len(angles) == 3:
                            self.robot_reference_roll = angles[0]
                            self.robot_reference_pitch = angles[1] 
                            self.robot_reference_yaw = angles[2]
                            self.robot_pose_set = True
                            
                            # Also send 'p' command to Arduino to set reference there
                            time.sleep(0.1)  # Small delay
                            self.ser.write(b'p')
                            time.sleep(0.1)  # Wait for Arduino response
                            
                            # Read Arduino confirmation
                            if self.ser.in_waiting > 0:
                                arduino_response = self.ser.readline().decode().strip()
                                if arduino_response:
                                    self.log_response(f"Arduino: {arduino_response}")
                            
                            self.log_response("🤖 Robot pose reference set!")
                            self.log_response(f"   Reference Roll:  {self.robot_reference_roll:.2f}°")
                            self.log_response(f"   Reference Pitch: {self.robot_reference_pitch:.2f}°")
                            self.log_response(f"   Reference Yaw:   {self.robot_reference_yaw:.2f}°")
                            self.log_response("Current IMU orientation is now 'robot facing right on flat surface'")
                            
                            # Update simulation if running
                            if hasattr(self, 'simulation') and self.simulation:
                                self.simulation.set_robot_reference(
                                    self.robot_reference_roll,
                                    self.robot_reference_pitch, 
                                    self.robot_reference_yaw
                                )
                        else:
                            self.log_response("Error: Invalid angle data received")
                    except ValueError:
                        self.log_response("Error: Could not parse angle data")
                else:
                    self.log_response("Error: No response from Arduino")
            else:
                self.log_response("Error: No data received from Arduino")
                
        except Exception as e:
            self.log_response(f"Error setting robot pose: {e}")
    
    def start_calibration_monitoring(self, cal_type):
        """Start monitoring calibration progress"""
        self.calibration_status = cal_type
        self.disable_command_buttons()  # Disable buttons during calibration
        if cal_type == "gyro":
            self.log_response("🔄 Starting gyroscope calibration - keep sensor still!")
        else:
            self.log_response("🧭 Starting magnetometer calibration - rotate sensor in all directions!")
        
        # Start a thread to monitor calibration
        self.calibration_thread = threading.Thread(target=self.monitor_calibration, daemon=True)
        self.calibration_thread.start()
    
    def monitor_calibration(self):
        """Monitor calibration progress and display status"""
        start_time = time.time()
        timeout = 60  # 60 second timeout for calibration
        
        try:
            while self.calibration_status != "idle":
                # Check for timeout
                if time.time() - start_time > timeout:
                    self.log_response("\n⚠️ Calibration timeout - re-enabling buttons")
                    break
                    
                if self.ser and self.ser.is_open:
                    if self.ser.in_waiting > 0:
                        try:
                            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                            if line:
                                self.process_calibration_message(line)
                        except UnicodeDecodeError:
                            continue
                
                # Small delay to prevent excessive CPU usage
                threading.Event().wait(0.1)
        except Exception as e:
            self.log_response(f"Error monitoring calibration: {e}")
        finally:
            self.calibration_status = "idle"
            self.enable_command_buttons()  # Ensure buttons are re-enabled
    
    def process_calibration_message(self, message):
        """Process calibration status messages using new concise protocol only"""
        # New concise protocol
        if message == "c1":
            # Gyroscope calibration in progress
            self.log_response_append(".")  # Progress indicator
        elif message == "c2":
            # Gyroscope calibration completed
            self.log_response("\n✅ Gyroscope calibration completed!")
            self.calibration_status = "idle"
            self.enable_command_buttons()  # Re-enable buttons
        elif message == "c3":
            # Gyroscope calibration failed
            self.log_response("\n❌ Gyroscope calibration failed!")
            self.calibration_status = "idle"
            self.enable_command_buttons()  # Re-enable buttons
        elif message == "m1":
            # Magnetometer calibration in progress
            self.log_response_append(".")  # Progress indicator
        elif message == "m2":
            # Magnetometer calibration completed
            self.log_response("\n✅ Magnetometer calibration completed!")
            self.calibration_status = "idle"
            self.enable_command_buttons()  # Re-enable buttons
        elif message and len(message) > 2:
            # Regular Arduino message (longer than status codes)
            self.log_response(f"Arduino: {message}")
    
    def log_response_append(self, text):
        """Append text to the last line in response area (for progress dots)"""
        self.response_text.insert(tk.END, text)
        self.response_text.see(tk.END)
        self.response_text.update_idletasks()
    
    def read_response(self):
        """Read response from Arduino (runs in separate thread)"""
        try:
            # Clear any buffered data first
            time.sleep(0.1)  # Give Arduino time to respond
            
            # Read the actual response
            response = self.ser.readline().decode().strip()
            if response:
                self.log_response(f"Arduino: {response}")
            else:
                # If no response, try once more
                time.sleep(0.1)
                response = self.ser.readline().decode().strip()
                if response:
                    self.log_response(f"Arduino: {response}")
        except Exception as e:
            self.log_response(f"Error reading response: {e}")
    
    def log_response(self, message):
        """Add message to response text area"""
        self.response_text.insert(tk.END, f"{message}\n")
        self.response_text.see(tk.END)
    
    def start_simulation(self):
        """Start the pygame simulation"""
        if self.simulation_running:
            return
            
        if not self.ser or not self.ser.is_open:
            self.log_response("Error: Connect to Arduino first")
            return
        
        self.simulation_running = True
        
        # Create simulation instance
        self.simulation = IMUSimulation(self.ser)
        self.simulation.running = True
        
        # Start pygame in separate thread
        self.pygame_thread = threading.Thread(target=self.simulation.run_simulation_loop, daemon=True)
        self.pygame_thread.start()
        
        # Update GUI
        self.status_label.configure(text="Status: Simulation Running\nPyGame window is open\nPress ESC in PyGame to close")
        self.start_btn.configure(state='disabled')
        self.stop_btn.configure(state='normal')
        self.yaw_btn.configure(state='normal')  # Enable yaw toggle when simulation runs
        
        self.log_response("Simulation started - PyGame window opened")
    
    def stop_simulation(self):
        """Stop the pygame simulation"""
        if not self.simulation_running:
            return
            
        self.simulation_running = False
        
        if self.simulation:
            self.simulation.stop()
        
        if self.pygame_thread:
            self.pygame_thread.join(timeout=2)
        
        # Update GUI
        self.status_label.configure(text="Status: Simulation Stopped\nReady to start again\nClick 'Start Simulation' below")
        self.start_btn.configure(state='normal')
        self.stop_btn.configure(state='disabled')
        self.yaw_btn.configure(state='disabled')  # Disable yaw toggle when simulation stops
        
        self.log_response("Simulation stopped")
    
    def update_angles(self):
        """Update angle display in GUI"""
        if self.simulation and self.simulation_running:
            self.roll_label.configure(text=f"Roll: {self.simulation.ax:.2f}°")
            self.pitch_label.configure(text=f"Pitch: {self.simulation.ay:.2f}°")
            self.yaw_label.configure(text=f"Yaw: {self.simulation.az:.2f}°")
        
        # Schedule next update
        self.root.after(100, self.update_angles)  # Update every 100ms
    
    def toggle_yaw(self):
        """Toggle yaw mode"""
        if self.simulation:
            self.simulation.toggle_yaw_mode()
            mode = "ON" if self.simulation.yaw_mode else "OFF"
            self.log_response(f"Yaw mode: {mode}")
        else:
            self.log_response("Error: Simulation not running")
    
    def on_closing(self):
        """Handle window closing"""
        self.stop_simulation()
        self.calibration_status = "idle"  # Stop calibration monitoring
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.root.destroy()
    
    def run(self):
        """Start the GUI application"""
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.root.mainloop()


if __name__ == '__main__':
    app = IMUControlGUI()
    app.run()
