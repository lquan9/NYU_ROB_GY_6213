"""GUI application"""
from importlib.resources import files
from pathlib import Path

# External libraries
import time
import asyncio
import math
import matplotlib
import cv2
import numpy as np

from nicegui import ui, app, run
from fastapi import Response

# Local libraries
from . import parameters, robot_python_code

matplotlib.use('Agg') # Force non-interactive backend

# Global variables
LOGGING = False
STREAM_VIDEO = False
ASSETS_DIR = Path(__file__).parent / 'assets'

if not ASSETS_DIR.exists():
    ASSETS_DIR = Path(__file__).resolve().parent.parent.parent / 'docs' / 'assets'

if ASSETS_DIR.exists():
    app.add_static_files('/assets', str(ASSETS_DIR))

# Frame converter for the video stream, from OpenCV to a JPEG image
def convert(frame: np.ndarray) -> bytes:
    """Converts a frame from OpenCV to a JPEG image.
    This is a free function (not in a class or inner-function),
    to allow run.cpu_bound to pickle it and send it to a separate process.
    """
    _, imencode_image = cv2.imencode('.jpg', frame)
    return imencode_image.tobytes()

def connect_with_camera():
    """Connect with the camera and return the video capture object."""
    video_capture = cv2.VideoCapture(1)
    return video_capture
    
def update_video(video_image):
    """Update video stream"""
    if STREAM_VIDEO:
        video_image.force_reload()

@ui.page('/')
def main_page():
    """ Main page of the GIU """
    # Move robot logic here if it needs to be accessible to the UI

    # Robot variables
    robot = robot_python_code.Robot()
    print("Creating robot instance")

    # Set dark mode for gui
    dark = ui.dark_mode()
    dark.value = True

    # Determine what speed and steering commands to send
    def update_commands():
        if speed_switch.value:
            cmd_speed = slider_speed.value
        else:
            cmd_speed = 0
        if steering_switch.value:
            cmd_steering_angle = slider_steering.value
        else:
            cmd_steering_angle = 0
        return cmd_speed, cmd_steering_angle

    # Update
    def update_connection_to_robot():
        if udp_switch.value:
            if not robot.connected_to_hardware:
                udp, udp_success = robot_python_code.create_udp_communication(parameters.arduinoIP, 
                                                                              parameters.localIP, 
                                                                              parameters.arduinoPort, 
                                                                              parameters.localPort, 
                                                                              parameters.bufferSize)
                if udp_success:
                    robot.setup_udp_connection(udp)
                    robot.connected_to_hardware = True
                    print("Should be set for UDP!")
                else:
                    udp_switch.value = False
                    robot.connected_to_hardware = False
        else:
            if robot.connected_to_hardware:
                robot.eliminate_udp_connection()
                robot.connected_to_hardware = False

    # Update the speed slider if steering is not enabled
    def enable_speed():
        if not speed_switch.value:
            slider_speed.value = 0

    # Update the steering slider if steering is not enabled
    def enable_steering():
        if not steering_switch.value:
            slider_steering.value = 0

    # Create the gui title bar
    with ui.card().classes('w-full  items-center'):
        ui.label('ROB-GY - 6213: Robot Navigation & Localization').style('font-size: 24px;')

    # Create the encoder sensor visualizations. These may be dummys for lab 01.
    with ui.card().classes('w-full'):
        with ui.grid(columns=3).classes('w-full items-center'):
            with ui.card().classes('w-full items-center h-60'):
                if STREAM_VIDEO:
                    video_image = ui.interactive_image('/video/frame').classes('w-full h-full')
                else:
                    ui.image('assets/a_robot_image.jpg').props('height=2')
                    video_image = None
            # with ui.card().classes('w-full items-center h-60'):
            #     main_plot = ui.pyplot(figsize=(3, 3))
            with ui.card().classes('items-center h-60'):
                ui.label('Encoder:').style('text-align: center;')
                encoder_count_label = ui.label('0')
                logging_switch = ui.switch('Data Logging ')
                udp_switch = ui.switch('Robot Connect')

    # Create the robot manual control slider and switch for speed
    with ui.card().classes('w-full'):
        with ui.grid(columns=4).classes('w-full'):
            with ui.card().classes('w-full items-center'):
                ui.label('SPEED:').style('text-align: center;')
            with ui.card().classes('w-full items-center'):
                slider_speed = ui.slider(min=0, max=100, value=0)
            with ui.card().classes('w-full items-center'):
                ui.label().bind_text_from(slider_speed, 'value').style('text-align: center;')
            with ui.card().classes('w-full items-center'):
                speed_switch = ui.switch('Enable', on_change=lambda: enable_speed())

    # Create the robot manual control slider and switch for steering
    with ui.card().classes('w-full'):
        with ui.grid(columns=4).classes('w-full'):
            with ui.card().classes('w-full items-center'):
                ui.label('STEER:').style('text-align: center;')
            with ui.card().classes('w-full items-center'):
                slider_steering = ui.slider(min=-20, max=20, value=0)
            with ui.card().classes('w-full items-center'):
                ui.label().bind_text_from(slider_steering, 'value').style('text-align: center;')
            with ui.card().classes('w-full items-center'):
                steering_switch = ui.switch('Enable', on_change=lambda: enable_steering())

    # Update slider values, plots, etc. and run robot control loop
    async def control_loop():
        update_connection_to_robot()
        cmd_speed, cmd_steering_angle = update_commands()
        robot.control_loop(cmd_speed, cmd_steering_angle, logging_switch.value)
        encoder_count_label.set_text(robot.robot_sensor_signal.encoder_counts)
        # update_lidar_data()
        # show_lidar_plot()
        update_video(video_image)

    ui.timer(0.1, control_loop)

    pass

def main():
    """ Entry point for GUI """
    ui.run(reload=False,
           host='0.0.0.0',
           port=8080,
           show=False,
           title="ROB-GY 6213 Robot Navigation & Localization",
           )

# Run the gui
# ui.run(native=True)
if __name__ in {"__main__", "__mp_main__"}:
    ui.run(reload=False, port=8080, show=True)

    main()
