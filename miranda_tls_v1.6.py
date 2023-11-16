## Created by Donny Mott
# 3dmapmaker@gmail.com
# v1.6
# https://github.com/Rotoslider/Miranda-TLS/tree/main

import sys
from smbus2 import SMBus
import time
from datetime import datetime
from more_itertools import time_limited
import os
import signal
import threading
import subprocess
from ouster import client, pcap
from ouster.client.core import ClientTimeout
import Jetson.GPIO as GPIO
import tkinter as tk
import tkinter.font as tkFont
from tkinter import ttk
from tkinter import Label, Button, Entry, StringVar, Frame, OptionMenu, Checkbutton, IntVar, messagebox
from contextlib import closing
import traceback

class MotorControl:
    def __init__(self, speed_value, angle_value, time_multiplier, direction, lidar, root, GLabel_564):
        self.root = root
        self.speed_value = speed_value
        self.angle_value = angle_value
        self.time_multiplier = time_multiplier
        self.direction = direction
        self.lidar = lidar
        self.stop_countdown = False
        self.operation_completed = False
        self.motor_stopped_manually = False
        self.conversion_factor = 91.01944444444445 # 1 degree/second ~ 91 motor counts/second
        self.dev_addr = 0x29 # adress if switch set to on. 0x28 if set to 1
        self.bus = SMBus(0) # I2C SDA(27), SCL(28) AGX bus1, NANO bus0
        self.pcap_initialized = False
        self.pcap_status = False  # Initialize to False
        self.pcap_thread = None
        self.config = client.SensorConfig()  # create empty config
        self.BUFFER_EXTENSION=2  # adds time to the record time to make up for start lag?
        self.RECORD_EXTENSION=5  # pcap record extension
        self.ouster_rotation_time = 0
        self.hostname = '169.254.201.29'  # Ouster IP address
        self.motor_running = False
        self.last_time = 0
        self.led_green = 35 # green led
        self.led_blue = 37 # blue led
        self.led_red = 38 # red led
        self.button_pin = 40 # start motor button
        self.turbo_mode = False  # Default Turbo Mode state, Doubles speed to 120rpm
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.led_green, GPIO.OUT)
        GPIO.setup(self.led_blue, GPIO.OUT)
        GPIO.setup(self.led_red, GPIO.OUT)
        GPIO.setup(self.button_pin, GPIO.IN)
        GPIO.output(self.led_green, GPIO.LOW)
        GPIO.output(self.led_blue, GPIO.LOW)
        GPIO.output(self.led_red, GPIO.HIGH)
        GPIO.add_event_detect(self.button_pin, GPIO.FALLING, callback=self.button_event, bouncetime=500)
        self.GLabel_564 = GLabel_564 # Status Display line
        self.ouster_is_ready = False
    
    # Method for Velodyne LIDAR recording
    def velodyne_pcap(self, rotation_time):
        self.pcap_initialized = False  # Reset flag at start
        #print(f"velodyne_pcap called with rotation_time: {rotation_time}")
        pcap_filename = f"/home/lidar/pointclouds/VTLS_{time.strftime('%Y_%m_%d_%H_%M_%S')}.pcap"
        self.p = subprocess.Popen(['tcpdump', '-i', 'eth0', '-w', pcap_filename, f'-G{int(rotation_time)}'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        time.sleep(2)  # Wait for a moment to give tcpdump time to start

        if os.path.exists(pcap_filename):
            print(f"PCAP file {pcap_filename} has been created.")
            self.pcap_status = True
            self.pcap_initialized = True  # Set flag to True when successful
            os.system(f"chown lidar:lidar {pcap_filename}")  # Change the file owner to 'lidar'
        else:
            print(f"Failed to create PCAP file {pcap_filename}.")
            self.pcap_status = False
               
    # Method for Ouster LIDAR recording
    def ouster_record(self, rotation_time):
        try:
            self.ouster_is_ready = False
            self.pcap_initialized = False  # Reset flag at start
       
            # Step 1: Configuring the Ouster sensor
            self.ouster_sensor_init()
    
            # Step 2: Handle standby mode if necessary
            self.set_operating_mode('OPERATING_NORMAL')
            print("Waiting 24 seconds for Ouster to switch to operating mode...")
            time.sleep(24)

            # Step 3: Calculating time for Ouster recording, similar to Velodyne
            self.ouster_rotation_time = int(rotation_time) + self.BUFFER_EXTENSION
            print(f"Ouster rotation time: {self.ouster_rotation_time} seconds")

            # Calculate buffer size
            # buffer_size = 640 * self.ouster_rotation_time + self.RECORD_EXTENSION
            buffer_size = 6400
            print(f"buffer size: {buffer_size}")
            self.pcap_status = True

            # Signal that ouster is ready
            self.ouster_is_ready = True
            print("Ouster ready for motor to start")
            self.pcap_initialized = True

            # Wait for motor to actually start
            while not self.motor_running:
                time.sleep(2.1)
    
            # Step 4: Start recording using Ouster's API
            with closing(client.Sensor(self.hostname, self.config.udp_port_lidar, self.config.udp_port_imu, buf_size=buffer_size)) as source:
                try:
                    # Generate filenames for metadata and pcap files
                    time_part = datetime.now().strftime("%Y%m%d_%H%M%S")
                    meta = source.metadata
                    print("Obtained sensor metadata.")
                    fname_base = f"/home/lidar/pointclouds/OS32_{meta.prod_line}_{meta.sn}_{meta.mode}_{time_part}"

                    print(f"Saving sensor metadata to:{fname_base}.json")
                    source.write_metadata(f"{fname_base}.json")

                    print(f"Writing to: {fname_base}.pcap")
                    source_it = time_limited(self.ouster_rotation_time + self.RECORD_EXTENSION, source)
                    try:
                        n_packets = pcap.record(source_it, f"{fname_base}.pcap")
                    except ClientTimeout:
                        print("Caught ClientTimeout exception. Continuing...")
                        n_packets = 0  # Assign a default value here if the try block fails

                    print(f"Captured {n_packets} packets")
                except Exception as e:
                    print(f"Error during Step 4: {e}")
                    #traceback.print_exc()
    
            
            # Step 5: Check for successful initialization and file creation
            if self.ouster_pcap_file_created(pcap_filename=f"{fname_base}.pcap", json_filename=f"{fname_base}.json"):
                self.pcap_status = True
                self.pcap_initialized = True  # Set flag to True when successful
            else:
                print(f"Failed to create Ouster PCAP and JSON files.")
        except Exception as e:
            print(f"An exception occurred: {e}")

            self.pcap_status = False

    # Ouster file save check
    def ouster_pcap_file_created(self, pcap_filename, json_filename):
        if os.path.exists(pcap_filename) and os.path.exists(json_filename):
            print(f"Ouster PCAP file {pcap_filename} and JSON file {json_filename} have been created.")
            os.system(f"chown lidar:lidar {json_filename}")  # Change the file owner to 'lidar'
            os.system(f"chown lidar:lidar {pcap_filename}")  # Change the file owner to 'lidar'
            return True
        else:
            print(f"Failed to create Ouster PCAP file {pcap_filename} and/or JSON file {json_filename}.")
            return False

    # Ouster initialization
    def ouster_sensor_init(self):
        self.config.operating_mode = client.OperatingMode.OPERATING_STANDBY
        self.config.lidar_mode = client.LidarMode.MODE_1024x10
        self.config.udp_port_lidar = 7502
        self.config.udp_port_imu = 7503
        client.set_config(hostname=self.hostname, config=self.config, persist=True, udp_dest_auto=True)

    def set_operating_mode(self, mode):
        if mode == 'OPERATING_STANDBY':
            self.config.operating_mode = client.OperatingMode.OPERATING_STANDBY
        else:
            self.config.operating_mode = client.OperatingMode.OPERATING_NORMAL
            
        client.set_config(hostname=self.hostname, config=self.config, persist=True, udp_dest_auto=True)

    def miranda_write(self, address, command, tx_data):
        try:
            self.bus.write_i2c_block_data(address, command, tx_data)
        except Exception as e:
            print(f"Failed to write to Miranda: {e}")

    def miranda_read(self, address, command, num_bytes):
        try:
            return self.bus.read_i2c_block_data(address, command, num_bytes)
        except Exception as e:
            print(f"Failed to read from Miranda: {e}")
            return None


    def read_motor_status(self):
        try:
            status_byte = self.bus.read_byte_data(self.dev_addr, 0x03)
            if status_byte == 0xff:
                return "Moving anticlockwise"
            elif status_byte == 0x00:
                return "Stopped"
            elif status_byte == 0x01:
                return "Moving clockwise"
            else:
                return f"Error: Unknown status {status_byte}"
        except Exception as e:
            return f"Error: {e}"

    def calibration_status(self):
        try:
            status_byte = self.bus.read_byte_data(self.dev_addr, 0x02)
            if status_byte == 0x00:
                return "calibration in progress"
            elif status_byte == 0x01:
                return "calibration completed successfully"
            elif status_byte == 0x02:
                return "calibration failed" 
        except Exception as e:
            print(f'Failed due to: {e}')
            return f"Error: {e}"              


    def set_turbo_mode(self, enable):
        self.turbo_mode = enable
        data = [0x01] if enable else [0x00]
        self.miranda_write(self.dev_addr, 0x53, data)
        time.sleep(0.2)  # Adding a delay to ensure Turbo Mode takes effect
        read_data = self.miranda_read(self.dev_addr, 0x54, 1)  # Verifying if Turbo Mode was set correctly
        
        if read_data:
            print(f"Turbo Mode read status: {'enabled' if read_data[0] == 0x01 else 'disabled'}")

    def stop_motor(self):
        self.stop_countdown = True  # Stop the countdown        
        if not self.operation_completed:
            self.GLabel_564.config(text="Motor Stopped")
        
        if self.motor_running:
            self.motor_running = False
            tx_data = [0x00, 0x00]
            self.miranda_write(self.dev_addr, 0x07, tx_data)
            GPIO.output(self.led_red, GPIO.HIGH)
            GPIO.output(self.led_green, GPIO.LOW)
            print("Motor stopped")
            self.set_operating_mode('OPERATING_STANDBY')
            
            if hasattr(self, 'p'):
                try:
                   # Check if the process is running
                   self.p.poll()
                   if self.p.returncode is None:
                       # Process is still running. Send SIGINT
                       self.p.send_signal(signal.SIGINT)
                       self.p.wait()  # Wait for the process to terminate
                except Exception as e:
                    print(f"An error occurred while stopping tcpdump: {e}")
            if self.pcap_thread is not None:
                # You can implement logic to safely terminate the pcap_thread if needed.
                # For now, we'll just set it to None
                self.pcap_thread = None
        
        
    def start_motor(self, speed_deg_sec, angle_deg, time_multiplier):
        if not self.motor_running:
            self.motor_running = True
            self.stop_countdown = False
            self.operation_completed = False
            self.motor_stopped_manually = False
            self.continuous_pcap_record = True

            # Calculations
            angle_deg = float(self.angle_value.get())
            initial_rotation_time = angle_deg / speed_deg_sec
            rotation_time = initial_rotation_time
            buffer_time = 0.1 * initial_rotation_time

            time_multiplier *= 60
            if time_multiplier > 0:
                angle_deg *= time_multiplier
                rotation_time = time_multiplier

            rotation_time += buffer_time
            rotation_time = round(rotation_time, 1)

            # LIDAR Thread
            if self.pcap_thread is None:  
                #print("About to start a new pcap_thread.")

                if self.lidar.get() == "Ouster":
                    self.pcap_thread = threading.Thread(target=self.ouster_record, args=(rotation_time + 15,))
                else:
                     self.pcap_thread = threading.Thread(target=self.velodyne_pcap, args=(rotation_time + 15,))
            
                self.pcap_thread.start()
                print("Debug: Started the LIDAR recording thread.")

            # Wait for pcap to initialize
            start_time = time.time()
            timeout_limit = 12 if self.lidar.get() != "Ouster" else (self.ouster_rotation_time + 120)  # Extend the timeout based on rotation time

            while not self.pcap_initialized:
                if time.time() - start_time > timeout_limit:
                    print("Timed out waiting for pcap to initialize.")
                    break
                time.sleep(0.2)

            if not self.pcap_status:
                user_response = messagebox.askyesno("Warning", "PCAP file not created. Would you like to proceed anyway?")
                if not user_response:
                    print("Motor will not start.")
                    return
         
            # Motor setup
            time.sleep(5)
            if speed_deg_sec > 720:
                speed_deg_sec = 720
                messagebox.showwarning("Warning", "Input RPM should not exceed 120. Defaulting to 120 RPM.")

            turbo_enabled = speed_deg_sec > 360
            self.set_turbo_mode(turbo_enabled)

            motor_counts = int((speed_deg_sec / (2 if turbo_enabled else 1)) * self.conversion_factor)
            print(f"Motor counts for {speed_deg_sec} deg/sec: {motor_counts}")

            current_direction = self.direction.get()
            if current_direction == "CounterClockwise":
                motor_counts = 0xFFFF - motor_counts + 1

            MSB = (motor_counts >> 8) & 0xFF
            LSB = motor_counts & 0xFF
            tx_data = [MSB, LSB]

            print("Debug: About to start the motor.") 
            self.miranda_write(self.dev_addr, 0x07, tx_data)
            GPIO.output(self.led_red, GPIO.LOW)
            GPIO.output(self.led_green, GPIO.HIGH)

            print(f"Motor will stop in {rotation_time} seconds.")
            self.root.after(int(rotation_time * 1000), self.stop_motor)
            self.countdown(rotation_time)

            print("PCAP file created successfully. Starting the motor.")
            
    def countdown(self, seconds):
        self._countdown_seconds = int(seconds)
        self._countdown()
    
    def _countdown(self):
        if self.stop_countdown:
            if self.motor_stopped_manually:  # Check if the motor was manually stopped
                self.GLabel_564.config(text="Motor Stopped")
            else:
                self.GLabel_564.config(text="Scan Completed")
            return
    
        if self._countdown_seconds > 0:
            mins, secs = divmod(self._countdown_seconds, 60)
            if mins > 0:
               time_str = f"{mins}min {secs}sec remaining"
            else:
               time_str = f"{secs}sec remaining"
            self.GLabel_564.config(text=time_str)
            self._countdown_seconds -= 1
            self.root.after(1000, self._countdown)  # Call this method again after 1 second
        else:
            self.operation_completed = True  # Set this flag when operation is complete
            self.GLabel_564.config(text="Scan Completed")
        

    def reset_motor(self):
        try:
        # Reset the motor
            self.miranda_write(self.dev_addr, 0x01, [])
            time.sleep(1.1)  # Wait 100ms before issuing new I2C commands, as per the manual
            GPIO.output(self.led_red, GPIO.LOW)
            GPIO.output(self.led_green, GPIO.LOW)
            GPIO.output(self.led_blue, GPIO.HIGH) 
            self.GLabel_564.config(text="Motor reset.")
            self.root.update_idletasks()  # Force GUI update
            print("Motor reset successfully.")
            time.sleep(1.5)
            
        # Initialize a flag for the message
            message_printed = False
  
        # Wait for calibration to complete
            while True:
                status = self.calibration_status()

                # Print the message only once
                if not message_printed:
                    print("Waiting for the motor to finish calibrating...")
                    self.GLabel_564.config(text="Waiting for calibrate...")
                    self.root.update_idletasks()  # Force GUI update
                    message_printed = True  # Set the flag to True so the message won't be printed again

                if status == "calibration completed successfully":
                    print("Calibration completed successfully.")
                    self.GLabel_564.config(text="Calibration completed")
                    self.set_default_kp()
                    self.set_default_ki()
                    self.set_default_kd()
                    GPIO.output(self.led_blue, GPIO.LOW)
                    GPIO.output(self.led_red, GPIO.HIGH)
                    break
                elif status == "calibration failed":
                    print("Calibration failed. Exiting.")
                    self.GLabel_564.config(text="Calibration failed")
                    self.root.update_idletasks()  # Force GUI update
                    exit(1)

                time.sleep(1)  # Check every second

        except Exception as e:
            print(f"Failed to reset the motor: {e}")
            self.GLabel_564.config(text=f"Failed to reset the motor: {e}")
            self.root.update_idletasks()  # Force GUI update
            
    def goto_relative_angle(self, angle, clockwise=True):
        degree_conversion_factor = 182  # 1 degree = 182 motor counts
        
        # Convert the desired angle to motor counts
        position = int(angle * degree_conversion_factor)
        
        # If the direction is counterclockwise, flip the sign of the position
        if not clockwise:
            position = -position
        
        # Extract MSB and LSB from the position
        msb = (position >> 8) & 0xFF
        lsb = position & 0xFF
        
        
        command = 0x06  # Command to move to a relative position
        
        try:
            # Use self.dev_addr to access the device address stored in the class instance
            self.miranda_write(self.dev_addr, command, [msb, lsb])
        except Exception as e:
            print(f"Failed to move to relative angle: {e}")            
               
 
    def button_event(self, channel):
        curr_time = time.time()
        if curr_time - self.last_time < 1.0:
            return
        self.last_time = curr_time
        if not self.motor_running:
            speed_deg_sec = float(self.speed_value.get())  
            angle_deg = float(self.angle_value.get())  
            self.start_motor(speed_deg_sec, angle_deg)  
        else:
            self.stop_motor()


    def on_close(self):
        GPIO.cleanup()
        self.root.destroy()
        self.set_operating_mode('OPERATING_STANDBY')
  

class App:
    def __init__(self, root):
        self.speed_value = tk.StringVar()
        self.angle_value = tk.StringVar()
        self.time_multiplier = tk.StringVar()
        self.direction = tk.StringVar(value="CounterClockwise")
        self.lidar = tk.StringVar(value="Velodyne")
        self.motor_control = MotorControl(self.speed_value, self.angle_value, self.time_multiplier, self.direction, self.lidar, root, None)
        self.original_font_sizes = {}  # Initialize as an empty dictionary
        
        self.root = root  # Store root as an attribute
        root.title("Miranda TLS")
        root.geometry("1024x1024")  # Set initial window size
        root.resizable(width=True, height=True)
        # Set a minimum window size
        root.minsize(500,500)
        
        for i in range(10):
      
            root.grid_rowconfigure(i, weight=1)
            root.grid_columnconfigure(i, weight=1)
        
        # Row 0
        GLabel_293 = tk.Label(root, text="Terrestrial Lidar Scanner", font=tkFont.Font(family='Times', size=42), fg="#333333", justify="center")
        GLabel_293.grid(row=0, column=1, columnspan=5, sticky="ew")
        GLabel_293.bind('<Configure>', self.resize_font)

        # Row 1
        GLabel_129 = tk.Label(root, text="Speed (deg/S)", font=tkFont.Font(family='Times', size=24), fg="#333333", justify="right", width=12)
        GLabel_129.grid(row=1, column=0, sticky="w")
        GLabel_129.bind('<Configure>', self.resize_font)
        
        GLineEdit_364 = tk.Entry(root, textvariable=self.speed_value, borderwidth="1", font=tkFont.Font(family='Times', size=30), fg="#333333", justify="center", relief="sunken", width=6)
        GLineEdit_364.grid(row=1, column=1, sticky="ew")
        GLineEdit_364.bind('<Configure>', self.resize_font)

        self.GButton_toggle = tk.Button(root, text="CounterClockwise", command=self.toggle_direction, font=tkFont.Font(family='Times', size=30), justify="center", relief='raised', borderwidth=2, width=20, anchor="center")
        self.GButton_toggle.grid(row=1, column=4, columnspan=2, sticky="ew")
        self.GButton_toggle.bind('<Configure>', self.resize_font)

        # Row 2
        GLabel_796 = tk.Label(root, text="Angle (°)", font=tkFont.Font(family='Times', size=24), fg="#333333", justify="right", width=12)
        GLabel_796.grid(row=2, column=0, sticky="w")
        GLabel_796.bind('<Configure>', self.resize_font)
       
        glabel_805 = tk.Label(root, text="Cont. (minutes)", font=tkFont.Font(family='Times', size=20), fg="#333333", justify="center", width=30)
        glabel_805.grid(row=2, column=3, columnspan=2, sticky="ew")
        glabel_805.bind('<Configure>', self.resize_font)
       
        glineedit_806 = tk.Entry(root, textvariable=self.time_multiplier, borderwidth="1", font=tkFont.Font(family='Times', size=30), fg="#333333", justify="center", relief="sunken", width=6)
        glineedit_806.grid(row=2, column=5, sticky="ew") 
        glineedit_806.bind('<Configure>', self.resize_font)      
                
        GLineEdit_248 = tk.Entry(root, textvariable=self.angle_value, borderwidth="1", font=tkFont.Font(family='Times', size=30), fg="#333333", justify="center", relief="sunken", width=6)
        GLineEdit_248.grid(row=2, column=1, sticky="ew")
        GLineEdit_248.bind('<Configure>', self.resize_font)

        # Row 3
        GLabel_lidar = tk.Label(root, text="Lidar", font=tkFont.Font(family='Times', size=34), fg="#333333", justify="right", width=6)
        GLabel_lidar.grid(row=3, column=1, sticky="w")
        GLabel_lidar.bind('<Configure>', self.resize_font)
        
        self.GButton_lidar = tk.Button(root, text="Velodyne", command=self.toggle_lidar, font=tkFont.Font(family='Times', size=30), justify="center", relief='raised', borderwidth=2, width=20, anchor="center")
        self.GButton_lidar.grid(row=3, column=4, columnspan=2, sticky="ew")
        self.GButton_lidar.bind('<Configure>', self.resize_font)
        
        # Row 4
        GLabel_546 = tk.Label(root, text="Manual Control", font=tkFont.Font(family='Times', size=38), fg="#333333", justify="center")
        GLabel_546.grid(row=4, column=1, columnspan=5, sticky="ew", pady=(60, 0))
        GLabel_546.bind('<Configure>', self.resize_font)

        # Row 5
        GButton_151 = tk.Button(root, text="⬅️", command=self.GButton_151_command, bg="#e9e9ed", font=tkFont.Font(family='Times', size=40), fg="#000000", justify="center", relief="raised", width=6)
        GButton_151.grid(row=5, column=1, sticky="ew", pady=(0,10))
        
        GLabel_732 = tk.Label(root, text="30°", font=tkFont.Font(family='Times', size=30), fg="#333333", justify="center")
        GLabel_732.grid(row=5, column=3, columnspan=2, sticky="ew")
        GLabel_732.bind('<Configure>', self.resize_font)

        GButton_939 = tk.Button(root, text="➡️", command=self.GButton_939_command, bg="#e9e9ed", font=tkFont.Font(family='Times', size=40), fg="#000000", justify="center", relief="raised", width=6)
        GButton_939.grid(row=5, column=5, sticky="ew", pady=(0,10))

        # Row 6
        GButton_938 = tk.Button(root, text="Start", command=self.GButton_938_command, bg="#e9e9ed", font=tkFont.Font(family='Times', size=40), fg="#000000", justify="center", relief="raised", width=6)
        GButton_938.grid(row=6, column=1, sticky="ew", pady=10)
        
        GLabel_216 = tk.Label(root, text="Scan", font=tkFont.Font(family='Times', size=30), fg="#333333", justify="center")
        GLabel_216.grid(row=6, column=3, columnspan=2, sticky="ew")
        GLabel_216.bind('<Configure>', self.resize_font)

        GButton_341 = tk.Button(root, text="Stop", command=self.GButton_341_command, bg="#e9e9ed", font=tkFont.Font(family='Times', size=40), fg="#000000", justify="center", relief="raised", width=6)
        GButton_341.grid(row=6, column=5, sticky="ew", pady=10)

        # Row 7
        GLabel_709 = tk.Label(root, text="Status", font=tkFont.Font(family='Times', size=34), fg="#333333", justify="center")
        GLabel_709.grid(row=7, column=1, columnspan=5, sticky="ew")
        GLabel_709.bind('<Configure>', self.resize_font)
        
        # Row 8
        self.GLabel_564 = tk.Label(root, text="", font=tkFont.Font(family='Times', size=34), fg="#333333", justify="center", relief="sunken")
        self.GLabel_564.grid(row=8, column=1, columnspan=5, sticky="ew")
        self.GLabel_564.bind('<Configure>', self.resize_font)
        self.motor_control.GLabel_564 = self.GLabel_564
        
        # Row 9
        GButton_884 = tk.Button(root, text="Reset Motor", command=self.motor_control.reset_motor, font=tkFont.Font(family='Times', size=28), fg="#000000", justify="center", relief="raised", width=14)
        GButton_884.grid(row=9, column=0, sticky="ew")
        GButton_884.bind('<Configure>', self.resize_font)

        GButton_565 = tk.Button(root, text="EXIT", command=self.safe_exit, bg="#e9e9ed", font=tkFont.Font(family='Times', size=34), fg="#000000", justify="center", relief="raised", width=8)
        GButton_565.grid(row=9, column=6, sticky="ew")
        GButton_565.bind('<Configure>', self.resize_font)
        
    def clear_selection(self, event):
        event.widget.select_clear()
        
    def toggle_direction(self):
        if self.direction.get() == "Clockwise":  # Currently Clockwise
            self.direction.set("CounterClockwise")
            self.GButton_toggle["text"] = "Counter Clockwise"
        else:  # Currently Counter Clockwise
            self.direction.set("Clockwise")
            self.GButton_toggle["text"] = "Clockwise"
            #print(f"Direction set to: {self.direction.get()}")
        
    def toggle_lidar(self):
        if self.lidar.get() == "Ouster":  # Currently Ouster
            self.lidar.set("Velodyne")
            self.GButton_lidar["text"] = "Velodyne"
        else:  # Currently Velodyne
            self.lidar.set("Ouster")
            self.GButton_lidar["text"] = "Ouster"
            #print(f"lidar set to: {self.lidar.get()}")
            
        
    def safe_exit(self):
        answer = messagebox.askyesno("Exit", "Are you sure you want to exit?")
        if answer:
            self.motor_control.stop_motor()  # Stop the motor using your existing logic
            self.motor_control.on_close()    # Clean up GPIO using your existing logic
            try:
                self.root.destroy()  # Close the GUI
            except:
                pass  # Ignore if the window is already closed            

    def GButton_151_command(self): #jog 30deg left
       self.motor_control.goto_relative_angle(30, clockwise=True)  # Move is backwards of what you would think
       print("Jog Counter Clockwise")


    def GButton_939_command(self): #jog 30deg right
        self.motor_control.goto_relative_angle(30, clockwise=False)  # Move is backwards of what you would think
        print("Jog Clockwise")


    def GButton_938_command(self):  # Start button
        try:
            speed_deg_sec = float(self.speed_value.get())
            angle_deg = float(self.angle_value.get())
            time_multiplier = float(self.time_multiplier.get())  # Read the time multiplier
            print(f"Speed: {speed_deg_sec} deg/sec, Angle: {angle_deg}°") 
        except ValueError:
            messagebox.showerror("Error", "Invalid or empty input for speed or angle.")
            return
        self.motor_control.start_motor(speed_deg_sec, angle_deg, time_multiplier)

    def GButton_341_command(self):
        # Stop the motor
        self.motor_control.stop_motor()
        
    def resize_font(self, event):
        widget = event.widget
        widget_id = id(widget)  # Get a unique identifier for the widget

        # Get the original font size of the widget only if it hasn't been stored yet
        if widget_id not in self.original_font_sizes:
            original_font = tkFont.Font(font=widget.cget("font"))
            self.original_font_sizes[widget_id] = original_font.actual()["size"]

        original_font_size = self.original_font_sizes[widget_id]

        # Get main window dimensions
        width = self.root.winfo_width()
        height = self.root.winfo_height()

        #print(f"Main window dimensions: Width={width}, Height={height}")  # Debugging line

        # Set minimum and maximum dimensions
        min_dim = 500
        max_dim = 1024

        # Calculate a new font size based on main window dimensions and constraints
        scale_factor = min(width, height) / max_dim
        new_size = int(original_font_size * scale_factor)  # Use the original font size for scaling
        new_size = max(8, new_size)  # Set a minimum font size

        # Configure the new font size directly
        widget.config(font=tkFont.Font(family='Times', size=new_size))


        #print(f"New font size: {new_size}")  # Debugging line
   
        
   
if __name__ == "__main__":
    root = tk.Tk()
    #motor_control = MotorControl(root)  # Pass root to MotorControl
    app = App(root)
    root.protocol("WM_DELETE_WINDOW", app.safe_exit)  # Handle window close button
    root.mainloop()
