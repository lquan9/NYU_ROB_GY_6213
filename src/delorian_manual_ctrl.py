# Objective: Control the MuJoCo car manually using keyboard input
# This script uses the modern MuJoCo API with keyboard controls
# Todo: Get Pygame onto the main thread so that this works on MacOS. 
# Contributed by: sous.chef
 
import mujoco
import mujoco.viewer
import numpy as np
import time
import os
import threading
try:
    import pygame
    HAS_PYGAME = True
except ImportError:
    print("Warning: PyGame not found. Keyboard controls will not work.")
    HAS_PYGAME = False


class CarController: 
    def __init__(self, model_path):
        # Load model and create data structure
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        
        # Find actuator indices by name
        self.steer_actuator_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "ghost-steer")
        self.drive_actuator_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "drive")
        
        # Control variables
        self.steering_angle = 0.0
        self.throttle = 0.0
        
        # Control parameters
        self.max_steer = 0.52      # (radians, ~30 degrees)
        self.steer_rate = 0.05     
        self.max_throttle = 1.0    # Maximum throttle
        self.throttle_rate = 0.1   
        
        # Decay rates
        self.steer_decay = 0.95    # Steering returns to center
        self.throttle_decay = 0.9  # Throttle decays
        
        if HAS_PYGAME:
            pygame.init()
            # Create a small hidden window just for keyboard input
            self.pygame_screen = pygame.display.set_mode((1, 1))
            pygame.display.set_caption("Car Controls (Keep this window focused)")

        print(f"Steer actuator ID: {self.steer_actuator_id}")
        print(f"Drive actuator ID: {self.drive_actuator_id}")
        print("\n=== CONTROLS ===")
        print("Arrow Keys:")
        print("  UP    : Accelerate forward")
        print("  DOWN  : Brake/Reverse")
        print("  LEFT  : Steer left")
        print("  RIGHT : Steer right")
        print("  SPACE : Stop (release all controls)")
        print("  ESC   : Exit")
        print("================\n")
    
    def keyboard_callback(self):
        if not HAS_PYGAME:
            print("Keyboard Input is not available.")
            return True

        # Handle events in PyGame
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    return False  # Exit signal
                elif event.key == pygame.K_SPACE:
                    self.steering_angle = 0.0
                    self.throttle = 0.0
                    print("Controls Released.")
                elif event.key == pygame.K_r:
                    mujoco.mj_resetData(self.model, self.data)
                    self.data.qpos[2] = 0.05  # Reset the car to a height of 5cm to avoid ground penetration
                    self.steering_angle = 0.0
                    self.throttle = 0.0
                    print("Simulation Reset.")

        # Check for keys pressed (continuous)
        keys = pygame.key.get_pressed()
        if keys[pygame.K_UP]: 
            self.throttle = min(self.throttle + self.throttle_rate, self.max_throttle)
        
        elif keys[pygame.K_DOWN]:
            # Brake/reverse
            self.throttle = max(self.throttle - self.throttle_rate, -self.max_throttle)
        
        elif keys[pygame.K_LEFT]:
            # Steer left
            self.steering_angle = min(self.steering_angle + self.steer_rate, self.max_steer)
        
        elif keys[pygame.K_RIGHT]:
            # Steer right 
            self.steering_angle = max(self.steering_angle - self.steer_rate, -self.max_steer)

        
        return True

    def update_controls(self):

        # Apply decay to steering (return to center)
        if abs(self.steering_angle) > 0.001:
            self.steering_angle *= self.steer_decay
        else:
            self.steering_angle = 0.0
        
        # Apply decay to throttle
        if abs(self.throttle) > 0.001:
            self.throttle *= self.throttle_decay
        else:
            self.throttle = 0.0
        
        # Set actuator controls
        self.data.ctrl[self.steer_actuator_id] = self.steering_angle
        self.data.ctrl[self.drive_actuator_id] = self.throttle
    
    def get_car_state(self):
        
        # The free joint "centroid" has 7 position DoFs and 6 velocity DoFs
        # qpos: [x, y, z, qw, qx, qy, qz, ...other joints...]
        # qvel: [vx, vy, vz, ωx, ωy, ωz, ...other joints...]
        
        car_position = self.data.qpos[:3].copy()
        car_orientation = self.data.qpos[3:7].copy()  # Quaternion [w, x, y, z]
        car_velocity = self.data.qvel[:3].copy()
        car_angular_velocity = self.data.qvel[3:6].copy()
        
        return {
            'position': car_position,
            'orientation': car_orientation,
            'velocity': car_velocity,
            'angular_velocity': car_angular_velocity,
            'speed': np.linalg.norm(car_velocity),
            'steering': self.steering_angle,
            'throttle': self.throttle
        }
    
    def print_state(self):
        state = self.get_car_state()
        print(f"\rPos: ({state['position'][0]:6.2f}, {state['position'][1]:6.2f}, {state['position'][2]:6.2f}) | "
              f"Speed: {state['speed']:5.2f} m/s | "
              f"Steer: {np.rad2deg(state['steering']):5.1f}° | "
              f"Throttle: {state['throttle']:5.2f}", end='')
    
    def run(self):
        # Create passive viewer
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            
            # Set camera to follow the car
            viewer.cam.lookat[:] = [0, 0, 0.5]
            viewer.cam.distance = 2.0
            viewer.cam.elevation = -20
            
            # Simulation loop
            frame_count = 0
            start_time = time.time()
            
            while viewer.is_running():
                step_start = time.time()
                # Handle keyboard input using PyGame    
                if not self.process_keyboard():
                    break

                self.update_controls()
                mujoco.mj_step(self.model, self.data)
                
                viewer.sync()
                
                # Print state every 30 frames
                frame_count += 1
                if frame_count % 30 == 0:
                    self.print_state()
                
                # Maintain real-time execution
                time_until_next_step = self.model.opt.timestep - (time.time() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)

        if HAS_PYGAME:
            pygame.quit()
        print("\nSimulation ended. PyGame quit successfully.")


def main():
    model_path = "../xml/delorian_floored.xml"
    if not os.path.exists(model_path):
        print(f"Error: Model file '{model_path}' not found!")
        print(f"Current directory: {os.getcwd()}")
    else:
        print(f"Loading model from: {model_path}")

    controller = CarController(model_path)
    controller.run()

if __name__ == "__main__":
    main()