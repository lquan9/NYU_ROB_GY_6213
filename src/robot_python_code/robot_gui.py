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
from robot_python_code import robot, parameters, data_handling

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

def get_time_in_ms():
    """Get the current time in milliseconds."""
    return int(time()*1000)

# Create the gui page
@ui.page('/')
def main_page():
    """Main page of the GUI."""

    # Robot variables
    robot_instance = robot.Robot()

    # Lidar data
    max_lidar_range = 12
    lidar_angle_res = 2
    num_angles = int(360 / lidar_angle_res)
    lidar_distance_list = []
    lidar_cos_angle_list = []
    lidar_sin_angle_list = []
    for i in range(num_angles):
        lidar_distance_list.append(max_lidar_range)
        lidar_cos_angle_list.append(math.cos(i*lidar_angle_res/180*math.pi))
        lidar_sin_angle_list.append(math.sin(i*lidar_angle_res/180*math.pi))

    # Set dark mode for gui
    dark = ui.dark_mode()
    dark.value = True

    # Set up the video stream, not needed for lab 1
    if STREAM_VIDEO:
        video_capture = cv2.VideoCapture(1)

    # Enable frame grabs from the video stream.
    @app.get('/video/frame')
    async def grab_video_frame() -> Response:
        if not video_capture.isOpened():
            return placeholder
        # The `video_capture.read` call is a blocking function.
        # So we run it in a separate thread (default executor) to avoid blocking the event loop.
        _, frame = await run.io_bound(video_capture.read)
        if frame is None:
            return placeholder
        # `convert` is a CPU-intensive function, so we run it in a separate process to avoid blocking the event loop and GIL.
        jpeg = await run.cpu_bound(convert, frame)
        return Response(content=jpeg, media_type='image/jpeg')

    # Convert lidar data to something visible in correct units. This is dummy data for lab 1.
    def update_lidar_data():
        for i in range(robot_instance.robot_sensor_signal.num_lidar_rays):
            distance_in_mm = robot_instance.robot_sensor_signal.distances[i]
            angle = 360-robot_instance.robot_sensor_signal.angles[i]
            if distance_in_mm > 20 and abs(angle) < 360:
                index = max(0,min(int(360/lidar_angle_res-1),int((angle-(lidar_angle_res/2))/lidar_angle_res)))
                lidar_distance_list[index] = distance_in_mm/1000
 
    # Determine what speed and steering commands to send
    def update_commands():

        # Experiment trial controls
        if robot_instance.running_trial:
            delta_time = get_time_in_ms() - robot_instance.trial_start_time
            if delta_time > parameters.trial_time:
                robot_instance.running_trial = False
                speed_switch.value = False
                steering_switch.value = False
                robot_instance.extra_logging = True
                print("End Trial :", delta_time)

        if robot_instance.extra_logging:
            delta_time = get_time_in_ms() - robot_instance.trial_start_time
            if delta_time > parameters.trial_time + parameters.extra_trial_log_time:
                logging_switch.value = False
                robot_instance.extra_logging = False

        # Regular slider controls
        if speed_switch.value:
            cmd_speed = slider_speed.value
        else:
            cmd_speed = 0
        if steering_switch.value:
            cmd_steering_angle = slider_steering.value
        else:
            cmd_steering_angle = 0
        return cmd_speed, cmd_steering_angle

    def update_connection_to_robot():
        """Update"""
        if udp_switch.value:
            if not robot_instance.connected_to_hardware:
                udp, udp_success = robot_instance.create_udp_communication(parameters.arduinoIP, parameters.localIP, parameters.arduinoPort, parameters.localPort, parameters.bufferSize)
                if udp_success:
                    robot_instance.setup_udp_connection(udp)
                    robot_instance.connected_to_hardware = True
                    print("Should be set for UDP!")
                else:
                    udp_switch.value = False
                    robot_instance.connected_to_hardware = False
        else:
            if robot_instance.connected_to_hardware:
                robot_instance.eliminate_udp_connection()
                robot_instance.connected_to_hardware = False

    def enable_speed():
        """Update the speed slider if steering is not enabled."""
        #if not speed_switch.value:
        #    slider_speed.value = 0
        d = 0

    def enable_steering():
        """Update the steering slider if steering is not enabled."""
        #if not steering_switch.value:
        #    slider_steering.value = 0
        d = 0

    def show_lidar_plot():
        """ Visualize the lidar scans"""
        with main_plot:
            fig = main_plot.fig
            fig.patch.set_facecolor('black')
            plt.clf()
            plt.style.use('dark_background')
            plt.tick_params(axis='x', colors='lightgray')
            plt.tick_params(axis='y', colors='lightgray')

            for i in range(num_angles):
                distance = lidar_distance_list[i]
                cos_ang = lidar_cos_angle_list[i]
                sin_ang = lidar_sin_angle_list[i]
                x = [distance * cos_ang, max_lidar_range * cos_ang]
                y = [distance * sin_ang, max_lidar_range * sin_ang]
                plt.plot(x, y, 'r')
            plt.grid(True)
            #plt.axis('equal')
            plt.xlim(-2,2)
            plt.ylim(-2,2)

    def run_trial():
        robot_instance.trial_start_time = get_time_in_ms()
        robot_instance.running_trial = True
        steering_switch.value = True
        speed_switch.value = True
        logging_switch.value = True
        if parameters.trial_type == "steering":
            slider_speed.value = parameters.trial_max_speed
            slider_steering.value = parameters.trial_input
        elif parameters.trial_type == "distance":
            slider_steering.value = 0
            if math.abs(parameters.trial_input) < parameters.trial_max_speed:
                slider_speed.value = parameters.trial_input
            else:
                slider_speed.value = parameters.trial_max_speed
                print("Trial input speed exceeds max speed, setting to max speed.")

        print("Start time:", robot.trial_start_time)

    def build_trial_plots(selected_file):
        """ Build the trial plot for the selected file"""
        if not selected_file:
            ui.dialog('No trial file selected.').open()
            return

        # overall data metrics 
        trial_metrics = data_handling.get_trial_metrics(trial_files)

        with selected_trial_plot:
            data_handling.plot_trial_basics(selected_trial_plot.fig, selected_file)

        with aggregate_plot:
            data_handling.plot_trial_aggregates(aggregate_plot.fig, trial_metrics)

    def build_sim_plots(selected_file):
        """ Build sim plots"""
        if not selected_file:
            ui.dialog('No trial file selected.').open()
            return

        with model_plot:
            data_handling.run_my_model_on_trial(model_plot.fig, selected_file)


    def stop_trial():
        robot_instance.running_trial = False
        robot_instance.extra_logging = False
        speed_switch.value = False
        steering_switch.value = False
        logging_switch.value = False
        slider_speed.value = 0
        slider_steering.value = 0
        print("Trial stopped")

    def create_trial_selector(trial_files, callback, button_label="Generate", title="Select Trial"):
        """Trial selector"""
        if trial_files:
            selector = ui.select(
                options={file: Path(file).name for file in trial_files},
                value=trial_files[0],
                label='Select a trial file',
            ).classes('w-full')
            ui.button(button_label, on_click=lambda: callback(selector.value))
            return selector
        else:
            ui.label('No trial files found.').style('color: #ff7f7f')
            return None

    trial_data_dir = parameters.datapath
    print(f"Looking for trial data in: {trial_data_dir}")
    if not trial_data_dir.exists():
        print("Trial data directory does not exist.")
        trial_data_dir = Path(__file__).resolve().parents[2] / 'data'
    trial_files = data_handling.get_trial_files(trial_data_dir)

    # Create the gui title bar
    with ui.card().classes('w-full items-center'):
        ui.label('ROB-GY - 6213: Robot Navigation & Localization').style('font-size: 24px;')

    with ui.tabs().classes('w-full') as tabs:
        control_tab = ui.tab('Robot Control')
        plot_tab = ui.tab('Data Plots')
        sim_tab = ui.tab('Simulation')

    with ui.tab_panels(tabs, value=control_tab).classes('w-full'):
        with ui.tab_panel(control_tab):
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
                        with ui.row():
                            run_trial_button = ui.button('Run Trial', on_click=lambda:run_trial())
                            stop_trial_button = ui.button('Stop Trial', on_click=lambda:stop_trial()).props('color=negative')

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

        with ui.tab_panel(plot_tab):
            with ui.card().classes('w-full'):
                ui.label('Logged Trials').style('font-size: 20px;')
                trial_selector = create_trial_selector(trial_files, build_trial_plots, "Generate")

            selected_trial_plot = ui.pyplot(figsize=(8, 5)).classes('w-full')
            aggregate_plot = ui.pyplot(figsize=(8, 5)).classes('w-full')

            if trial_files and trial_selector:
                trial_selector.on_value_change(lambda event: build_trial_plots(event.value))
                build_trial_plots(trial_files[0])

        with ui.tab_panel(sim_tab):
            with ui.card().classes('w-full'):
                ui.label('Run Model Against Trials').style('font-size: 20px;')
                ui.label(f'Data directory: {trial_data_dir}')

            with ui.card().classes('w-full'):
                sim_selector = create_trial_selector(trial_files,
                                                     build_sim_plots,
                                                     "Run Model")

            model_plot = ui.pyplot(figsize=(8, 5)).classes('w-full')

            with model_plot:
                model_plot.fig.patch.set_facecolor('black')
                ax = model_plot.fig.add_subplot(1, 1, 1)
                ax.set_facecolor('black')
                ax.set_xlim(0, 1)
                ax.set_ylim(0, 1)
                ax.axis('off')

            with ui.card().classes('w-full'):
                ui.label('Sample Model').style('font-size: 20px;')

    # Update slider values, plots, etc. and run robot control loop
    async def control_loop():
        update_connection_to_robot()
        cmd_speed, cmd_steering_angle = update_commands()
        robot_instance.control_loop(cmd_speed, cmd_steering_angle, logging_switch.value)
        encoder_count_label.set_text(robot_instance.robot_sensor_signal.encoder_counts)
        # update_lidar_data()
        # show_lidar_plot()
        update_video(video_image)

    ui.timer(0.1, control_loop)

    pass

def main():
    """ Entry point for GUI """
    favicon_path = ASSETS_DIR / 'favicon.png' if ASSETS_DIR.exists() else None
    ui.run(reload=False,
           host='0.0.0.0',
           port=8080,
           show=False,
           title="ROB-GY 6213 Robot Navigation & Localization",
           favicon=str(favicon_path) if favicon_path else None,
           )

# Run the gui
# ui.run(native=True)
if __name__ in {"__main__", "__mp_main__"}:
    favicon_path = ASSETS_DIR / 'favicon.png' if ASSETS_DIR.exists() else None
    ui.run(reload=False,
           port=8080,
           show=True,
           title="ROB6213",
           favicon=str(favicon_path) if favicon_path else None)

    main()
