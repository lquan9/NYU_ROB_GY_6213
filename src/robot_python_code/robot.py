"""Python code for the robot"""

# External libraries
from time import strftime
import socket
import pickle
import time
import cv2
import cv2.aruco as aruco
import numpy as np
import matplotlib.pyplot as plt
from time import strftime

# Local libraries
from . import parameters, extended_kalman_filter

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
        self.filename = self.datapath / ("robot_data_" + str(control_signal[0]) + "_" + str(control_signal[1]) + strftime("_%d_%m_%y_%H_%M_%S.pkl"))
        self.dictionary = {}
        for name in self.data_name_list:
            self.dictionary[name] = []
        self.dictionary['state_mean'] = []
        self.dictionary['state_covariance'] = []

    def log(self, logging_switch_on, time, control_signal, robot_sensor_signal, camera_sensor_signal, state_mean=None, state_covariance=None):
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
        self.dictionary['camera_sensor_signal'].append(camera_sensor_signal)
        self.dictionary['state_mean'].append(state_mean)
        self.dictionary['state_covariance'].append(state_covariance)

        self.line_count += 1
        if self.line_count > parameters.max_num_lines_before_write:
            self.line_count = 0
            with open(self.filename, 'wb') as file_handle:
                pickle.dump(self.dictionary, file_handle)


class DataLoader:
    def __init__(self, filename):
        self.filename = filename

    def load(self):
        with open(self.filename, 'rb') as file_handle:
            loaded_dict = pickle.load(file_handle)
        return loaded_dict


class CameraSensor:
    def __init__(self, cam_cfg):
        """Initialize with a camera config dict from parameters (camera_a or camera_b)."""
        self.cam_cfg = cam_cfg
        self.name = cam_cfg.get('name', 'Camera')
        # Resolve camera source: network URL takes priority over local index
        source = cam_cfg['source'] if cam_cfg['source'] is not None else cam_cfg['camera_id']
        self.source = source
        self.cap = cv2.VideoCapture(source)
        if isinstance(source, str):
            print(f"[{self.name}] Using network camera: {source}")
        else:
            print(f"[{self.name}] Using local camera device: {source}")
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.aruco_params = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        # Per-camera intrinsics
        self.camera_matrix = cam_cfg['camera_matrix']
        self.dist_coeffs = cam_cfg['dist_coeffs']

    def get_signal(self, last_camera_signal):
        camera_signal = last_camera_signal
        ret, pose_estimate = self.get_pose_estimate()
        if ret:
            camera_signal = pose_estimate
        return camera_signal, ret  # ret = True if marker was detected this frame

    def get_pose_estimate(self):
        """Detect markers and compute robot pose in world frame.

        If both world-origin tag and robot tag are visible, returns
        the robot's pose relative to the world tag via relative_pose().
        If only the robot tag is visible, returns raw camera-frame values
        (backward compatible, but less useful without world reference).
        """
        ret, frame = self.cap.read()
        if not ret:
            return False, []

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None:
            return False, []

        # Collect per-ID poses
        world_rvec, world_tvec = None, None
        robot_rvec, robot_tvec = None, None

        for i, marker_id in enumerate(ids.flatten()):
            if marker_id == parameters.world_marker_id:
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                    corners[i], parameters.world_marker_length,
                    self.camera_matrix, self.dist_coeffs)
                world_rvec = rvec[0][0]
                world_tvec = tvec[0][0]
            elif marker_id == parameters.robot_marker_id:
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                    corners[i], parameters.robot_marker_length,
                    self.camera_matrix, self.dist_coeffs)
                robot_rvec = rvec[0][0]
                robot_tvec = tvec[0][0]

        if robot_rvec is None:
            return False, []

        # If we have both markers, compute relative pose (robot in world frame)
        if world_rvec is not None:
            world_pose = parameters.relative_pose(
                world_rvec, world_tvec, robot_rvec, robot_tvec)
            return True, world_pose  # [x, y, theta] in world frame

        # Fallback: only robot tag visible, return raw camera-frame values
        pose_raw = [robot_tvec[0], robot_tvec[1], robot_tvec[2],
                    robot_rvec[0], robot_rvec[1], robot_rvec[2]]
        return True, pose_raw


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

        # Dual-camera setup
        self.camera_a = CameraSensor(parameters.camera_a)
        self.camera_b_enabled = (parameters.camera_b['source'] is not None)
        self.camera_b = CameraSensor(parameters.camera_b) if self.camera_b_enabled else None

        self.data_logger = DataLogger(parameters.datapath, parameters.data_name_list)
        self.robot_sensor_signal = RobotSensorSignal([0, 0, 0])

        # Per-camera raw signals and detection flags
        self.cam_signal_a = [0, 0, 0]
        self.cam_signal_b = [0, 0, 0]
        self.cam_fresh_a = False
        self.cam_fresh_b = False

        # Fused result: best available world-frame pose
        self.camera_sensor_signal = [0, 0, 0]  # signal from chosen camera
        self.camera_pose = [0, 0, 0]            # world frame [x, y, theta]
        self.camera_is_fresh = False             # True when any camera detected this frame
        self.camera_source_label = '-'           # which camera provided the detection

        self.extended_kalman_filter = extended_kalman_filter.ExtendedKalmanFilter(
            x_0=[0, 0, 0], Sigma_0=parameters.I3 * 10e12, encoder_counts_0=0)

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
        # Only pass camera measurement when a fresh marker was detected this frame
        if self.camera_is_fresh:
            z_t = np.array([self.camera_pose[0], self.camera_pose[1], self.camera_pose[2]])
        else:
            z_t = None  # EKF will run prediction-only
        delta_t = 0.1
        self.extended_kalman_filter.update(u_t, z_t, delta_t)

    def _fuse_cameras(self):
        """Read both cameras, pick the best fresh detection.

        CameraSensor.get_signal() now returns [x, y, theta] in world frame
        when both world and robot markers are visible, or raw camera-frame
        values as fallback.
        """
        # Camera A
        self.cam_signal_a, self.cam_fresh_a = self.camera_a.get_signal(self.cam_signal_a)

        # Camera B
        if self.camera_b is not None:
            self.cam_signal_b, self.cam_fresh_b = self.camera_b.get_signal(self.cam_signal_b)
        else:
            self.cam_fresh_b = False

        # Priority: Camera A if fresh, else Camera B if fresh, else stale
        if self.cam_fresh_a:
            self.camera_sensor_signal = self.cam_signal_a
            self.camera_pose = self.cam_signal_a[:3]  # [x, y, theta]
            self.camera_is_fresh = True
            self.camera_source_label = 'A'
        elif self.cam_fresh_b:
            self.camera_sensor_signal = self.cam_signal_b
            self.camera_pose = self.cam_signal_b[:3]
            self.camera_is_fresh = True
            self.camera_source_label = 'B'
        else:
            self.camera_is_fresh = False
            self.camera_source_label = '-'

    def control_loop(self, cmd_speed=0, cmd_steering_angle=0, logging_switch_on=False):
        # get camera detections and fuse
        self._fuse_cameras()

        fresh_str = f"FRESH:{self.camera_source_label}" if self.camera_is_fresh else "STALE"
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

        # log data
        self.data_logger.log(logging_switch_on, time.perf_counter(), control_signal,
                             self.robot_sensor_signal, self.camera_sensor_signal,
                             self.extended_kalman_filter.state_mean,
                             self.extended_kalman_filter.state_covariance)