"""GUI application"""
from importlib.resources import files
from pathlib import Path

# External libraries
import time
import asyncio
import math
import matplotlib
import matplotlib.pyplot as plt
import cv2
import numpy as np
import tempfile

from nicegui import ui, app, run
from fastapi import Response

# Local libraries
from robot_python_code import robot, parameters, data_handling, calibrate

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
    return int(time.time()*1000)

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

    def update_commands():
        """Determin what speed and steering commands to send"""

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
                udp, udp_success = robot_instance.create_udp_communication(parameters.arduinoIP,
                                                                           parameters.localIP,
                                                                           parameters.arduinoPort,
                                                                           parameters.localPort,
                                                                           parameters.bufferSize)
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
        if not speed_switch.value:
            slider_speed.value = 0

    def enable_steering():
        """Update the steering slider if steering is not enabled."""
        if not steering_switch.value:
            slider_steering.value = 0

    # def show_lidar_plot():
    #     """ Visualize the lidar scans"""
    #     with main_plot:
    #         fig = main_plot.fig
    #         fig.patch.set_facecolor('black')
    #         plt.clf()
    #         plt.style.use('dark_background')
    #         plt.tick_params(axis='x', colors='lightgray')
    #         plt.tick_params(axis='y', colors='lightgray')

    #         for i in range(num_angles):
    #             distance = lidar_distance_list[i]
    #             cos_ang = lidar_cos_angle_list[i]
    #             sin_ang = lidar_sin_angle_list[i]
    #             x = [distance * cos_ang, max_lidar_range * cos_ang]
    #             y = [distance * sin_ang, max_lidar_range * sin_ang]
    #             plt.plot(x, y, 'r')
    #         plt.grid(True)
    #         #plt.axis('equal')
    #         plt.xlim(-2,2)
    #         plt.ylim(-2,2)

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

        # print("Start time:", robot.trial_start_time)
        print("Start time:", robot_instance.trial_start_time)

    def build_trial_plots(selected_file):
        """ Build the trial plot for the selected file"""
        if not selected_file:
            ui.dialog('No trial file selected.').open()
            return

        # overall data metrics 
        trial_metrics = data_handling.get_trial_metrics(trial_files)

        with selected_trial_plot:
            data_handling.plot_trial_basics(selected_trial_plot.fig, selected_file)

        # with aggregate_plot:
        #     data_handling.plot_trial_aggregates(aggregate_plot.fig, trial_metrics)

    def build_sim_plots(selected_file):
        """ Build sim plots"""
        if not selected_file:
            ui.dialog('No trial file selected.').open()
            return

        d_pred = data_handling.run_my_model_to_predict_distance(selected_file)
        predicted_distance_label.text = f'{d_pred:.3f} m'

        with model_plot:
            data_handling.run_my_model_on_trial(model_plot.fig, selected_file)

    def run_multi_predict():
        """Run multi-predict plotting"""
        with model_plot:
            data_handling.plot_many_trial_predictions(model_plot.fig, trial_data_dir)

    def run_sample_model():
        """Run sample model plotting"""
        with model_plot:
            data_handling.sample_model(model_plot.fig)

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
        calibration_tab = ui.tab('Calibration')
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
                        with ui.row():
                            logging_switch = ui.switch('Data Logging ')
                            debug_switch = ui.switch('Debug Prints', value=parameters.DEBUG_PRINTS,
                                                    on_change=lambda e: setattr(parameters, 'DEBUG_PRINTS', e.value))
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

        # calibration
        with ui.tab_panel(calibration_tab):
            with ui.card().classes('w-full'):
                ui.label('Run Calibration').style('font-size: 16px; font-weight: bold;')

                # store custom config path
                custom_config_path = None
                config_label = ui.label('Using default config: calibrate_base.json').style('font-size: 12px; color: gray;')

                # Calibration results display
                results_container = ui.card().classes('w-full').style('display: none;')
                with results_container:
                    ui.label('Calibration Results').style('font-size: 14px; font-weight: bold; color: lightgreen;')
                    results_grid = ui.grid(columns=2).classes('w-full gap-2')
                    with results_grid:
                        ui.label('counts_to_m:').style('color: lightgray;')
                        counts_result_label = ui.label('--').style('color: white; font-family: monospace;')
                        ui.label('std deviation:').style('color: lightgray;')
                        counts_std_label = ui.label('--').style('color: white; font-family: monospace;')
                        ui.label('distance_variance_a:').style('color: lightgray;')
                        var_a_result_label = ui.label('--').style('color: white; font-family: monospace;')
                        ui.label('distance_variance_b:').style('color: lightgray;')
                        var_b_result_label = ui.label('--').style('color: white; font-family: monospace;')
                        ui.label('trials:').style('color: lightgray;')
                        trials_result_label = ui.label('--').style('color: white; font-family: monospace;')
                        ui.label('plot:').style('color: lightgray;')
                        plot_result_label = ui.label('--').style('color: white; font-family: monospace;')

                    calib_plot = ui.pyplot(figsize=(10, 10)).classes('w-full')

                    with calib_plot:
                        calib_plot.fig.patch.set_facecolor('black')
                        ax = calib_plot.fig.add_subplot(1, 1, 1)
                        ax.set_facecolor('black')
                        ax.set_xlim(0, 1)
                        ax.set_ylim(0, 1)
                        ax.axis('off')

                # def handle_config_upload(e):
                #     """Handle custom config file upload"""
                #     nonlocal custom_config_path
                #     if e.content:
                #         try:
                #             # Save uploaded file temporarily
                #             with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
                #                 f.write(e.content.read().decode('utf-8'))
                #                 custom_config_path = f.name
                #             config_label.text = f'Using custom config: {e.name}'
                #             config_label.style('font-size: 12px; color: lightgreen;')
                #             ui.notify(f'Config loaded: {e.name}', type='positive')
                #         except Exception as ex:
                #             ui.notify(f'Error loading config: {str(ex)}', type='negative')
                #             custom_config_path = None

                # def reset_to_default_config():
                #     """Reset to default config"""
                #     nonlocal custom_config_path
                #     custom_config_path = None
                #     config_label.text = 'Using default config: calibrate_base.json'
                #     config_label.style('font-size: 12px; color: gray;')
                #     ui.notify('Reset to default config', type='info')

                def run_encoder_calibration():
                    """Run encoder calibration"""
                    try:
                        if custom_config_path:
                            config = calibrate.load_config(custom_config_path)
                        else:
                            config = calibrate.load_config()
                        result = calibrate.calibrate_encoder(config)

                        if result:
                            # update parameters
                            parameters.counts_to_m = result['value']
                            counts_to_m_input.value = result['value']
                            parameters.distance_variance_a = result['distance_variance_a']
                            distance_var_a_input.value = result['distance_variance_a']
                            parameters.distance_variance_b = result['distance_variance_b']
                            distance_var_b_input.value = result['distance_variance_b']

                            results_container.style('display: block;')
                            counts_result_label.text = f"{result['value']:.0f} counts/m"
                            counts_std_label.text = f"{result['std']:.0f}"
                            var_a_result_label.text = f"{result['distance_variance_a']:.6f} m^2"
                            var_b_result_label.text = f"{result['distance_variance_b']:.6f}"
                            trials_result_label.text = f"{len(result['trials'])} trials"
                            plot_result_label.text = "encoder_calibration.png"

                            message = f"Encoder calibration complete!\n"
                            message += f"counts_to_m = {result['value']:.0f}, std: {result['std']:.0f} counts/m\n"
                            message += f"distance_variance_a = {result['distance_variance_a']:.6f}\n"
                            message += f"distance_variance_b = {result['distance_variance_b']:.6f}\n"
                            message += f"Based on {len(result['trials'])} trials\n"
                            message += f"Plot saved to assets/calibration_plots/encoder_calibration.png"
                            ui.notify(message, type='positive', multi_line=True, timeout=8000)
                            print(f"Calibration complete: counts_to_m = {result['value']:.6f}")

                            # load and display the calibration plot
                            plot_path = Path('docs/assets/calibration_plots/encoder_calibration.png')
                            if plot_path.exists():
                                with calib_plot:
                                    calib_plot.fig.clear()
                                    img = plt.imread(str(plot_path))
                                    ax = calib_plot.fig.add_subplot(1, 1, 1)
                                    ax.imshow(img, interpolation='nearest')
                                    ax.axis('off')
                                    calib_plot.fig.tight_layout(pad=0)
                                    calib_plot.update()
                        else:
                            ui.notify('No valid calibration trials found', type='warning')
                    except Exception as e:
                        ui.notify(f'Calibration error: {str(e)}', type='negative')
                        print(f"Error during calibration: {e}")

                def run_steering_calibration():
                    """Run steering calibration"""
                    try:
                        if custom_config_path:
                            config = calibrate.load_config(custom_config_path)
                        else:
                            config = calibrate.load_config()
                        result = calibrate.calibrate_steering(config)

                        if result:
                            parameters.steering_to_w = result['value']
                            steering_to_w_input.value = result['value']
                            parameters.steering_variance_a = result['steering_variance_a']
                            steering_var_a_input.value = result['steering_variance_a']
                            parameters.steering_variance_b = result['steering_variance_b']
                            steering_var_b_input.value = result['steering_variance_b']

                            message = f"Steering calibration complete!\n"
                            message += f"steering_to_w = {result['value']:.4f} rad/s per unit\n"
                            message += f"steering_variance_a = {result['steering_variance_a']:.6f} (rad/s)^2\n"
                            message += f"steering_variance_b = {result['steering_variance_b']:.6f}\n"
                            message += f"Based on {len(result['trials'])} trials"
                            ui.notify(message, type='positive', multi_line=True, timeout=8000)
                            print(f"Steering calibration complete: steering_to_w = {result['value']:.6f}")
                        else:
                            ui.notify('No valid calibration trials found', type='warning')
                    except Exception as e:
                        ui.notify(f'Steering calibration error: {str(e)}', type='negative')
                        print(f"Error during steering calibration: {e}")

                with ui.column().classes('gap-2'):
                    with ui.row().classes('items-center gap-2'):
                        ui.button('Calibrate Encoders', on_click=run_encoder_calibration, icon='straighten').props('color=primary')
                        ui.button('Calibrate Steering', on_click=run_steering_calibration, icon='pivot_table_chart').props('color=primary')
                    # with ui.row().classes('items-center gap-2'):
                        # ui.upload(on_upload=handle_config_upload,
                        #           auto_upload=True).props('accept=.json').classes('max-w-xs').tooltip('Upload custom config')
                        # ui.button('Reset Config', on_click=reset_to_default_config, icon='refresh').props('flat color=grey')

            # encoder and distance
            with ui.card().classes('w-full'):
                ui.label('Encoder & Distance Parameters').style('font-size: 16px; font-weight: bold;')
                with ui.grid(columns=2).classes('w-full'):
                    with ui.column():
                        ui.label('counts_to_m (counts/m)')
                        counts_to_m_input = ui.number(value=parameters.counts_to_m, format='%.1f',
                                                      on_change=lambda e: setattr(parameters, 'counts_to_m', e.value))
                        ui.label('Formula: encoder_count_change / measured_distance').style('font-size: 12px; color: gray;')

                    with ui.column():
                        ui.label('distance_variance_a (m^2)')
                        distance_var_a_input = ui.number(value=parameters.distance_variance_a, format='%.6f',
                                                        on_change=lambda e: setattr(parameters, 'distance_variance_a', e.value))
                        ui.label('Base variance in distance measurement').style('font-size: 12px; color: gray;')

                    with ui.column():
                        ui.label('distance_variance_b')
                        distance_var_b_input = ui.number(value=parameters.distance_variance_b, format='%.4f',
                                                        on_change=lambda e: setattr(parameters, 'distance_variance_b', e.value))
                        ui.label('Variance scaling factor').style('font-size: 12px; color: gray;')

            # steering
            with ui.card().classes('w-full'):
                ui.label('Steering Parameters').style('font-size: 16px; font-weight: bold;')
                with ui.grid(columns=2).classes('w-full'):
                    with ui.column():
                        ui.label('steering_to_w')
                        steering_to_w_input = ui.number(value=parameters.steering_to_w, format='%.4f',
                                                       on_change=lambda e: setattr(parameters, 'steering_to_w', e.value))
                        ui.label('Converts steering command to angular velocity').style('font-size: 12px; color: gray;')

                    with ui.column():
                        ui.label('max_steer_deg (degrees)')
                        max_steer_input = ui.number(value=parameters.max_steer_deg, format='%.1f',
                                                   on_change=lambda e: setattr(parameters, 'max_steer_deg', e.value))
                        ui.label('Maximum steering angle').style('font-size: 12px; color: gray;')

                    with ui.column():
                        ui.label('steering_variance_a (rad^2)')
                        steering_var_a_input = ui.number(value=parameters.steering_variance_a, format='%.6f',
                                                        on_change=lambda e: setattr(parameters, 'steering_variance_a', e.value))
                        ui.label('Base variance in steering').style('font-size: 12px; color: gray;')

                    with ui.column():
                        ui.label('steering_variance_b')
                        steering_var_b_input = ui.number(value=parameters.steering_variance_b, format='%.4f',
                                                        on_change=lambda e: setattr(parameters, 'steering_variance_b', e.value))
                        ui.label('Variance scaling factor').style('font-size: 12px; color: gray;')

            # chasis
            with ui.card().classes('w-full'):
                ui.label('Base Parameters').style('font-size: 16px; font-weight: bold;')
                with ui.grid(columns=3).classes('w-full'):
                    with ui.column():
                        ui.label('wheelbase (m)')
                        wheelbase_input = ui.number(value=parameters.wheelbase, format='%.4f',
                                                   on_change=lambda e: setattr(parameters, 'wheelbase', e.value))
                        ui.label('Distance between front and rear axles').style('font-size: 12px; color: gray;')

                    with ui.column():
                        ui.label('track_width (m)')
                        track_width_input = ui.number(value=parameters.track_width, format='%.4f',
                                                     on_change=lambda e: setattr(parameters, 'track_width', e.value))
                        ui.label('Distance between left and right wheels').style('font-size: 12px; color: gray;')

                    with ui.column():
                        ui.label('wheel_radius (m)')
                        wheel_radius_input = ui.number(value=parameters.wheel_radius, format='%.4f',
                                                      on_change=lambda e: setattr(parameters, 'wheel_radius', e.value))
                        ui.label('Radius of the drive wheels').style('font-size: 12px; color: gray;')

        with ui.tab_panel(plot_tab):
            with ui.card().classes('w-full'):
                ui.label('Logged Trials').style('font-size: 20px;')
                ui.label(f'Data directory: {trial_data_dir}')
                trial_selector = create_trial_selector(trial_files,
                                                       build_trial_plots,
                                                       "Generate")

            selected_trial_plot = ui.pyplot(figsize=(8, 5)).classes('w-full')
            # aggregate_plot = ui.pyplot(figsize=(4, 4)).classes('w-full')

            def save_trial_plot():
                """Save trial plot"""
                save_path = Path('docs/latex/plots/trial_plot_export.png')
                save_path.parent.mkdir(parents=True, exist_ok=True)
                selected_trial_plot.fig.savefig(str(save_path), dpi=150, bbox_inches='tight', facecolor='black')
                ui.notify(f'Plot saved to {save_path}', type='positive')

            ui.button('Save Plot', on_click=save_trial_plot, icon='save').props('color=secondary')

            if trial_files and trial_selector:
                trial_selector.on_value_change(lambda event: build_trial_plots(event.value))
                build_trial_plots(trial_files[0])

        with ui.tab_panel(sim_tab):
            with ui.card().classes('w-full'):
                ui.label('Run Model Using Data').style('font-size: 20px;')
                ui.label(f'Data directory: {trial_data_dir}')
                with ui.row():
                    create_trial_selector(trial_files,
                                          build_sim_plots,
                                          "Single Run")
                    ui.button("Multi-Predict", on_click=lambda: run_multi_predict())
                    ui.button("Sample Model", on_click=lambda: run_sample_model())
                ui.label('Predicted Distance').style('font-size: 16px; font-weight: bold;')
                predicted_distance_label = ui.label('--').style('font-size: 24px; color: cyan;')

            model_plot = ui.pyplot(figsize=(5, 5)).classes('w-full')

            with model_plot:
                model_plot.fig.patch.set_facecolor('black')
                ax = model_plot.fig.add_subplot(1, 1, 1)
                ax.set_facecolor('black')
                ax.set_xlim(0, 1)
                ax.set_ylim(0, 1)
                ax.axis('off')

            def save_model_plot():
                """Save model plot"""
                save_path = Path('docs/latex/plots/model_plot_export.png')
                save_path.parent.mkdir(parents=True, exist_ok=True)
                model_plot.fig.savefig(str(save_path), dpi=150, bbox_inches='tight', facecolor='black')
                ui.notify(f'Plot saved to {save_path}', type='positive')

            ui.button('Save Plot', on_click=save_model_plot, icon='save').props('color=secondary')

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
