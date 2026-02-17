
# Objective: Control Delorian with GUI control panel
# Provides sliders for steering and throttle, plus state display

import mujoco
import mujoco.viewer
import numpy as np
import time
import threading
import tkinter as tk
from tkinter import ttk


class CarControllerGUI:
    def __init__(self, model_path):
        # Load model and create data structure
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        
        # Find actuator indices
        self.steer_actuator_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "ghost-steer")
        self.drive_actuator_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "drive")
        
        # Control variables
        self.steering_angle = 0.0
        self.throttle = 0.0
        
        # Control limits
        self.max_steer = 0.52
        self.max_throttle = 1.0
        
        # Simulation control
        self.running = True
        self.paused = False
        self.reset = False
        
        # Create GUI
        self.create_gui()
    
    def create_gui(self):

        self.root = tk.Tk()
        self.root.title("MuJoCo Car Controller")
        self.root.geometry("400x500")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # Main frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Title
        title = ttk.Label(main_frame, text="Car Control Panel", font=('Arial', 16, 'bold'))
        title.grid(row=0, column=0, columnspan=2, pady=10)
        
        # === STEERING CONTROL ===
        ttk.Label(main_frame, text="Steering Angle", font=('Arial', 12)).grid(row=1, column=0, sticky=tk.W, pady=5)
        
        self.steering_var = tk.DoubleVar(value=0.0)
        steering_slider = ttk.Scale(
            main_frame, 
            from_=-self.max_steer, 
            to=self.max_steer,
            orient=tk.HORIZONTAL,
            variable=self.steering_var,
            length=300,
            command=self.on_steering_change
        )
        steering_slider.grid(row=2, column=0, columnspan=2, pady=5)
        
        self.steering_label = ttk.Label(main_frame, text="0.00° (0.000 rad)", font=('Arial', 10))
        self.steering_label.grid(row=3, column=0, columnspan=2)
        
        # === THROTTLE CONTROL ===
        ttk.Label(main_frame, text="Throttle", font=('Arial', 12)).grid(row=4, column=0, sticky=tk.W, pady=5)
        
        self.throttle_var = tk.DoubleVar(value=0.0)
        throttle_slider = ttk.Scale(
            main_frame,
            from_=-self.max_throttle,
            to=self.max_throttle,
            orient=tk.HORIZONTAL,
            variable=self.throttle_var,
            length=300,
            command=self.on_throttle_change
        )
        throttle_slider.grid(row=5, column=0, columnspan=2, pady=5)
        
        self.throttle_label = ttk.Label(main_frame, text="0.00", font=('Arial', 10))
        self.throttle_label.grid(row=6, column=0, columnspan=2)
        
        # === CONTROL BUTTONS ===
        button_frame = ttk.Frame(main_frame)
        button_frame.grid(row=7, column=0, columnspan=2, pady=20)
        
        self.center_button = ttk.Button(button_frame, text="Center Steering", command=self.center_steering)
        self.center_button.grid(row=0, column=0, padx=5)
        
        self.stop_button = ttk.Button(button_frame, text="Stop", command=self.stop_car)
        self.stop_button.grid(row=0, column=1, padx=5)
        
        self.pause_button = ttk.Button(button_frame, text="Pause", command=self.toggle_pause)
        self.pause_button.grid(row=0, column=2, padx=5)
        
        # === STATE DISPLAY ===
        ttk.Separator(main_frame, orient=tk.HORIZONTAL).grid(row=8, column=0, columnspan=2, sticky='ew', pady=10)
        
        ttk.Label(main_frame, text="Vehicle State", font=('Arial', 12, 'bold')).grid(row=9, column=0, columnspan=2, pady=5)
        
        state_frame = ttk.Frame(main_frame)
        state_frame.grid(row=10, column=0, columnspan=2, pady=5)
        
        self.position_label = ttk.Label(state_frame, text="Position: (0.00, 0.00, 0.00)", font=('Arial', 9))
        self.position_label.grid(row=0, column=0, sticky=tk.W, pady=2)
        
        self.velocity_label = ttk.Label(state_frame, text="Velocity: 0.00 m/s", font=('Arial', 9))
        self.velocity_label.grid(row=1, column=0, sticky=tk.W, pady=2)
        
        self.heading_label = ttk.Label(state_frame, text="Heading: 0.0°", font=('Arial', 9))
        self.heading_label.grid(row=2, column=0, sticky=tk.W, pady=2)
        
        # Start GUI update loop
        self.update_gui()
    
    def on_steering_change(self, value):
        """Called when steering slider changes"""
        self.steering_angle = float(value)
        deg = np.rad2deg(self.steering_angle)
        self.steering_label.config(text=f"{deg:.2f}° ({self.steering_angle:.3f} rad)")
    
    def on_throttle_change(self, value):
        """Called when throttle slider changes"""
        self.throttle = float(value)
        self.throttle_label.config(text=f"{self.throttle:.2f}")
    
    def center_steering(self):
        """Center the steering wheel"""
        self.steering_var.set(0.0)
        self.steering_angle = 0.0
    
    def stop_car(self):
        """Stop the car (zero throttle and center steering)"""
        self.steering_var.set(0.0)
        self.throttle_var.set(0.0)
        self.steering_angle = 0.0
        self.throttle = 0.0
    
    def toggle_pause(self):
        """Pause/unpause the simulation"""
        self.paused = not self.paused
        self.pause_button.config(text="Resume" if self.paused else "Pause")
    
    def update_gui(self):
        """Update GUI state display"""
        if self.running:
            # Get car state
            state = self.get_car_state()
            
            # Update labels
            self.position_label.config(
                text=f"Position: ({state['position'][0]:.2f}, {state['position'][1]:.2f}, {state['position'][2]:.2f})"
            )
            self.velocity_label.config(text=f"Velocity: {state['speed']:.2f} m/s")
            self.heading_label.config(text=f"Heading: {state['heading']:.1f}°")
            
            # Schedule next update
            self.root.after(50, self.update_gui)  # Update at ~20 Hz
    
    def get_car_state(self):
        """Get current state of the car"""
        car_position = self.data.qpos[:3].copy()
        car_orientation = self.data.qpos[3:7].copy()  # Quaternion [w, x, y, z]
        car_velocity = self.data.qvel[:3].copy()
        
        # Calculate heading from quaternion (rotation around Z-axis)
        qw, qx, qy, qz = car_orientation
        heading = np.arctan2(2*(qw*qz + qx*qy), 1 - 2*(qy**2 + qz**2))
        heading_deg = np.rad2deg(heading)
        
        return {
            'position': car_position,
            'orientation': car_orientation,
            'velocity': car_velocity,
            'speed': np.linalg.norm(car_velocity),
            'heading': heading_deg
        }
    
    def apply_controls(self):
        """Apply control values to actuators"""
        self.data.ctrl[self.steer_actuator_id] = self.steering_angle
        self.data.ctrl[self.drive_actuator_id] = self.throttle
    
    def on_closing(self):
        """Handle window closing"""
        self.running = False
        self.root.destroy()
    
    def simulation_thread(self):
        """Run simulation in separate thread"""
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            viewer.cam.lookat[:] = [0, 0, 0.5]
            viewer.cam.distance = 2.0
            viewer.cam.elevation = -20
            
            while viewer.is_running() and self.running:
                step_start = time.time()
                
                if not self.paused:
                    # Apply controls
                    self.apply_controls()
                    
                    # Step simulation
                    mujoco.mj_step(self.model, self.data)
                    
                    # Sync viewer
                    viewer.sync()
                
                # Maintain real-time
                time_until_next_step = self.model.opt.timestep - (time.time() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)
            
            self.running = False
    
    def run(self):
        # Start simulation in separate thread
        sim_thread = threading.Thread(target=self.simulation_thread, daemon=True)
        sim_thread.start()
        
        # Run GUI (blocking)
        self.root.mainloop()


def main():
    
    model_path = "../xml/mcqueen_floored.xml"
    
    if not os.path.exists(model_path):
        print(f"Error: Model file '{model_path}' not found!")
        print(f"Current directory: {os.getcwd()}")
        return
    
    print("Starting MuJoCo Car Controller with GUI...")
    controller = CarControllerGUI(model_path)
    controller.run()


if __name__ == "__main__":
    main()