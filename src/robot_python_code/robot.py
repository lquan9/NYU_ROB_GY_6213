"""Python code for the robot"""

# External libraries
from time import strftime
import socket
import pickle
import time
import math
import cv2
import cv2.aruco as aruco
import numpy as np
import matplotlib.pyplot as plt
from time import strftime

# Local libraries
from . import parameters, extended_kalman_filter, particle_filter

def create_udp_communication(arduinoIP, localIP, arduinoPort, localPort, bufferSize):
    try:
        udp = UDPCommunication(arduinoIP, localIP, arduinoPort, localPort, bufferSize)
        print("Success in creating udp communication")
        return udp, True
    except Exception as exc:
        print(f"Failed to create udp communication: {exc}")
        return None, False

class UDPCommunication:
    def __init__(self, arduinoIP, localIP, arduinoPort, localPort, bufferSize):
        self.arduinoIP = arduinoIP
        self.arduinoPort = arduinoPort
        self.localIP = localIP
        self.localPort = localPort
        self.bufferSize = bufferSize
        self.UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        self.UDPServerSocket.bind((localIP, localPort))

    def receive_msg(self):
        bytesAddressPair = self.UDPServerSocket.recvfrom(self.bufferSize)
        message = bytesAddressPair[0]
        address = bytesAddressPair[1]
        clientMsg = "{}".format(message.decode())
        clientIP = "{}".format(address)
        return clientMsg

    def send_msg(self, msg):
        bytesToSend = str.encode(msg)
        self.UDPServerSocket.sendto(bytesToSend, (self.arduinoIP, self.arduinoPort))


class DataLogger:
    def __init__(self, datapath, data_name_list):
        self.datapath = datapath
        datapath.mkdir(parents=True, exist_ok=True)
        self.filename = datapath / "robot_data"
        self.line_count = 0
        self.dictionary = {}
        self.data_name_list = data_name_list
        for name in data_name_list:
            self.dictionary[name] = []
        self.currently_logging = False

    def reset_logfile(self, control_signal):
        self.filename = self.datapath / ("lab_" + str(control_signal[0]) + "_" + str(control_signal[1]) + strftime("_%d_%m_%y_%H_%M_%S.pkl"))
        self.dictionary = {}
        for name in self.data_name_list:
            self.dictionary[name] = []
        self.dictionary['state_mean'] = []
        self.dictionary['state_covariance'] = []

    #TODO: update to make configurable
    # def log(self, logging_switch_on, time, control_signal, robot_sensor_signal, camera_sensor_signal, state_mean=None, state_covariance=None):
    def log(self, logging_switch_on, time, control_signal, robot_sensor_signal, state_mean, particle_set):
        if not logging_switch_on:
            if self.currently_logging:
                self.currently_logging = False
        else:
            if not self.currently_logging:
                self.currently_logging = True
                self.reset_logfile(control_signal)

        self.dictionary['time'].append(time)
        self.dictionary['control_signal'].append(control_signal)
        self.dictionary['robot_sensor_signal'].append(robot_sensor_signal)
        self.dictionary['state_mean'].append(state_mean)
        # TODO: make configurable
        # self.dictionary['camera_sensor_signal'].append(camera_sensor_signal)
        # self.dictionary['state_covariance'].append(state_covariance)
        self.dictionary['state_covariance'].append(particle_set)

        self.line_count += 1
        if self.line_count > parameters.max_num_lines_before_write:
            self.line_count = 0
            with open(self.filename, 'wb') as file_handle:
                pickle.dump(self.dictionary, file_handle)


class CustomUnpickler(pickle.Unpickler):
    """Custom unpickler to fix module paths"""
    def find_class(self, module, name):
        if module == 'particle_filter':
            module = 'robot_python_code.particle_filter'
        elif module == 'motion_models':
            module = 'robot_python_code.motion_models'
        return super().find_class(module, name)

class DataLoader:
    def __init__(self, filename):
        self.filename = filename

    def load(self):
        with open(self.filename, 'rb') as file_handle:
            loaded_dict = CustomUnpickler(file_handle).load()
        return loaded_dict


class CameraSensor:
    def __init__(self, camera_id):
        self.camera_id = camera_id
        # Resolve camera source: network URL takes priority over local index
        source = parameters.camera_source if parameters.camera_source is not None else camera_id
        self.source = source
        self.cap = cv2.VideoCapture(source)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)    # 1=manual, 3=auto
        self.cap.set(cv2.CAP_PROP_EXPOSURE, 150)        # adjust this value (50-500)
        self.cap.set(cv2.CAP_PROP_AUTO_WB, 0)           # disable auto white balance

        if isinstance(source, str):
            print(f"[CameraSensor] Using network camera: {source}")
        else:
            print(f"[CameraSensor] Using local camera device: {source}")
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.parameters)

    def get_signal(self, last_camera_signal):
        camera_signal = last_camera_signal
        ret, pose_estimate = self.get_pose_estimate()
        if ret:
            camera_signal = pose_estimate
        return camera_signal, ret  # ret = True if marker was detected this frame

    def get_pose_estimate(self):
        ret, frame = self.cap.read()
        if not ret:
            return False, []

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = self.detector.detectMarkers(gray)
        if ids is not None:
            for i in range(len(ids)):
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[i], parameters.marker_length, parameters.camera_matrix, parameters.dist_coeffs)
                pose_estimate = [tvec[0][0][0], tvec[0][0][1], tvec[0][0][2], rvec[0][0][0], rvec[0][0][1], rvec[0][0][2]]
            return True, pose_estimate

        return False, []

    def close(self):
        self.cap.release()
        cv2.destroyAllWindows()


class MsgSender:
    delta_send_time = 0.1

    def __init__(self, last_send_time, msg_size, udp_communication):
        self.last_send_time = last_send_time
        self.msg_size = msg_size
        self.udp_communication = udp_communication

    def send_control_signal(self, control_signal):
        packed_send_msg = self.pack_msg(control_signal)
        self.send(packed_send_msg)

    def send(self, msg):
        new_send_time = time.perf_counter()
        if new_send_time - self.last_send_time > self.delta_send_time:
            message = ""
            for data in msg:
                message = message + str(data)
            self.udp_communication.send_msg(message)
            self.last_send_time = new_send_time

    def pack_msg(self, msg):
        packed_msg = ""
        for data in msg:
            if packed_msg == "":
                packed_msg = packed_msg + str(data)
            else:
                packed_msg = packed_msg + ", " + str(data)
        packed_msg = packed_msg + "\n"
        return packed_msg


class RobotSensorSignal:
    def __init__(self, unpacked_msg):
        self.encoder_counts = int(unpacked_msg[0])
        self.steering = int(unpacked_msg[1])
        self.num_lidar_rays = int(unpacked_msg[2])
        self.angles = []
        self.distances = []
        for i in range(self.num_lidar_rays):
            index = 3 + i * 2
            self.angles.append(unpacked_msg[index])
            self.distances.append(unpacked_msg[index + 1])

    def convert_hardware_angle(self, angle):
        return -angle * math.pi / 180

    def convert_hardware_distance(self, distance):
        return distance / 1000

    def print(self):
        print("Robot Sensor Signal")
        print(" encoder: ", self.encoder_counts)
        print(" steering:", self.steering)
        print(" num_lidar_rays: ", self.num_lidar_rays)
        print(" angles: ", self.angles)
        print(" distances: ", self.distances)

    def to_list(self):
        sensor_data_list = []
        sensor_data_list.append(self.encoder_counts)
        sensor_data_list.append(self.steering)
        sensor_data_list.append(self.num_lidar_rays)
        for i in range(self.num_lidar_rays):
            sensor_data_list.append(self.angles[i])
            sensor_data_list.append(self.distances[i])
        return sensor_data_list

    # def convert_hardware_angle(self, angle):                       # TODO: have to update
    #     return -angle * math.pi / 180

    # def convert_hardware_distance(self, distance):               # TODO: have to updatre
    #     return distance / 1000


class MsgReceiver:
    delta_receive_time = 0.05

    def __init__(self, last_receive_time, msg_size, udp_communication):
        self.last_receive_time = last_receive_time
        self.msg_size = msg_size
        self.udp_communication = udp_communication

    def receive(self):
        new_receive_time = time.perf_counter()
        if new_receive_time - self.last_receive_time > self.delta_receive_time:
            received_msg = self.udp_communication.receive_msg()
            self.last_receive_time = new_receive_time
            return True, received_msg
        return False, ""

    def unpack_msg(self, packed_msg):
        unpacked_msg = []
        msg_list = packed_msg.split(',')
        if len(msg_list) >= self.msg_size:
            for data in msg_list:
                unpacked_msg.append(float(data))
            return True, unpacked_msg
        return False, unpacked_msg

    def receive_robot_sensor_signal(self, last_robot_sensor_signal):
        robot_sensor_signal = last_robot_sensor_signal
        receive_ret, packed_receive_msg = self.receive()
        if receive_ret:
            unpack_ret, unpacked_receive_msg = self.unpack_msg(packed_receive_msg)
            if unpack_ret:
                robot_sensor_signal = RobotSensorSignal(unpacked_receive_msg)
        return robot_sensor_signal


class Robot:
    def __init__(self):
        self.connected_to_hardware = False
        self.running_trial = False
        self.extra_logging = False
        self.trial_start_time = 0
        self.msg_sender = None
        self.msg_receiver = None
        self.camera_sensor = CameraSensor(parameters.camera_id)
        self.data_logger = DataLogger(parameters.datapath, parameters.data_name_list)
        self.robot_sensor_signal = RobotSensorSignal([0, 0, 0])
        self.camera_sensor_signal = [0, 0, 0, 0, 0, 0]  # raw camera frame values
        self.camera_pose = [0, 0, 0]                     # world frame [x, y, theta]
        self.camera_is_fresh = False                      # True when marker detected this frame
        self.last_update_time = time.perf_counter()       # for computing actual delta_t
        self.extended_kalman_filter = extended_kalman_filter.ExtendedKalmanFilter(
            x_0=[0, 0, 0], Sigma_0=parameters.I3 * 10e12, encoder_counts_0=0)
        map = particle_filter.Map(parameters.wall_corner_list)
        # self.particle_filter = particle_filter.ParticleFilter(
        #     parameters.num_particles, map,
        #     particle_filter.State(parameters.pf_start_x, parameters.pf_start_y, parameters.pf_start_theta),
        #     particle_filter.State(parameters.pf_start_stdev, parameters.pf_start_stdev, parameters.pf_start_stdev),
        #     parameters.pf_known_start,0)
        self.particle_filter = particle_filter.ParticleFilter(
            parameters.num_particles, map,
            particle_filter.State(parameters.pf_start_x, parameters.pf_start_y, parameters.pf_start_theta),
            particle_filter.State(parameters.pf_start_stdev, parameters.pf_start_stdev, parameters.pf_start_stdev),
            parameters.pf_known_start,
            self.robot_sensor_signal.encoder_counts)
        self.pf_initialized = False

    def create_udp_communication(self, arduinoIP, localIP, arduinoPort, localPort, bufferSize):
        return create_udp_communication(arduinoIP, localIP, arduinoPort, localPort, bufferSize)

    def setup_udp_connection(self, udp_communication):
        self.msg_sender = MsgSender(time.perf_counter(), parameters.num_robot_control_signals, udp_communication)
        self.msg_receiver = MsgReceiver(time.perf_counter(), parameters.num_robot_sensors, udp_communication)
        print("Reset msg_senders and receivers!")

    def eliminate_udp_connection(self):
        self.msg_sender = None
        self.msg_receiver = None
        print("Eliminate UDP !!!")

    def update_state_estimate(self):
        u_t = np.array([self.robot_sensor_signal.encoder_counts, self.robot_sensor_signal.steering])
        
        # first time we get a real encoder reading, sync the PF baseline so delta starts at 0
        if not self.pf_initialized and self.robot_sensor_signal.encoder_counts != 0:
            self.particle_filter.last_encoder_counts = self.robot_sensor_signal.encoder_counts
            self.pf_initialized = True

        if self.robot_sensor_signal.num_lidar_rays > 0:
            z_t = self.robot_sensor_signal
        else:
            z_t = None
        now = time.perf_counter()
        delta_t = max(0.01, min(0.5, now - self.last_update_time))
        self.last_update_time = now
        self.particle_filter.update(u_t, z_t, delta_t)

    def control_loop(self, cmd_speed=0, cmd_steering_angle=0, logging_switch_on=False):
        # get raw camera signal in camera frame + whether marker was detected
        # TODO: make this configurable
        # self.camera_sensor_signal, self.camera_is_fresh = self.camera_sensor.get_signal(self.camera_sensor_signal)

        # transform to world frame - this is what EKF uses as measurement z_t
        self.camera_pose = parameters.camera_to_world(self.camera_sensor_signal)

        fresh_str = "FRESH" if self.camera_is_fresh else "STALE"
        print(f"Camera [{fresh_str}] raw: ",
              int(100 * self.camera_sensor_signal[0]),
              int(100 * self.camera_sensor_signal[1]),
              int(100 * self.camera_sensor_signal[2]))
        print(f"Camera [{fresh_str}] world:",
              round(self.camera_pose[0], 3),
              round(self.camera_pose[1], 3),
              round(self.camera_pose[2], 3))

        # receive encoder and steering from robot
        if self.msg_sender is not None:
            self.robot_sensor_signal = self.msg_receiver.receive_robot_sensor_signal(self.robot_sensor_signal)

        # run EKF with encoder (prediction) and camera world pose (correction)
        self.update_state_estimate()

        control_signal = [cmd_speed, cmd_steering_angle]

        # send control to robot
        if self.msg_receiver is not None:
            self.msg_sender.send_control_signal(control_signal)

        # log raw camera signal so offline EKF can rerun the transform if needed
        # self.data_logger.log(logging_switch_on, time.perf_counter(), control_signal,
        #                      self.robot_sensor_signal, self.camera_sensor_signal,
        #                      self.extended_kalman_filter.state_mean,
        #                      self.extended_kalman_filter.state_covariance)
        # TODO: make configurable
        self.data_logger.log(logging_switch_on, time.perf_counter(), control_signal, 
                             self.robot_sensor_signal, 
                             self.particle_filter.particle_set.mean_state, self.particle_filter.particle_set)

    # TODO:
    # Put lidar angles in the correct units and correct direction.
    def convert_hardware_angle(self, angle):
        return -angle * math.pi / 180 # degrees to rad
    
    # Put lidar distances in the correct units.
    def convert_hardware_distance(self, distance):
        return distance / 1000 # mm to m