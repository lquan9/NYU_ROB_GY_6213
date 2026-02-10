"""Python code for the robot"""

# External libraries
from time import strftime
import socket
import pickle
import time
import cv2
import cv2.aruco as aruco

# Local libraries
from . import parameters

def create_udp_communication(arduinoIP, localIP, arduinoPort, localPort, bufferSize):
    """Function to try to connect to the robot via udp over wifi"""
    try:
        udp = UDPCommunication(arduinoIP, localIP, arduinoPort, localPort, bufferSize)
        print("Success in creating udp communication")
        return udp, True
    except:
        print("Failed to create udp communication!")
        return _, False

class UDPCommunication:
    """Class to hold the UPD over wifi connection setup"""
    def __init__(self, arduinoIP, localIP, arduinoPort, localPort, bufferSize):
        self.arduinoIP = arduinoIP
        self.arduinoPort = arduinoPort
        self.localIP = localIP
        self.localPort = localPort
        self.bufferSize = bufferSize
        self.UDPServerSocket = socket.socket(family = socket.AF_INET, type = socket.SOCK_DGRAM)
        self.UDPServerSocket.bind((localIP, localPort))

    def receive_msg(self):
        """Receive a message from the robot"""
        bytesAddressPair = self.UDPServerSocket.recvfrom(self.bufferSize)
        message = bytesAddressPair[0]
        address = bytesAddressPair[1]
        clientMsg = "{}".format(message.decode())
        clientIP = "{}".format(address)

        return clientMsg

    def send_msg(self, msg):
        """ Send a message to the robot"""
        bytesToSend = str.encode(msg)
        self.UDPServerSocket.sendto(bytesToSend, (self.arduinoIP, self.arduinoPort))


class DataLogger:
    """Class to hold the data logger that records data when needed"""

    # Constructor
    def __init__(self, filename_start, data_name_list):
        self.filename_start = filename_start
        self.filename = filename_start
        self.line_count = 0
        self.file = open(filename, 'w', encoding='utf-8')
        self.dictionary = {}
        self.data_name_list = data_name_list
        for name in data_name_list:
            self.dictionary[name] = []
        self.currently_logging = False

    # Open the log file
    def reset_logfile(self, control_signal):
        self.filename = self.filename_start + "_"+str(control_signal[0])+"_"+str(control_signal[1]) + strftime("_%d_%m_%y_%H_%M_%S.pkl")
        self.dictionary = {}
        for name in self.data_name_list:
            self.dictionary[name] = []

        
    # Log one time step of data
    def log(self, logging_switch_on, time, control_signal, robot_sensor_signal, camera_sensor_signal):
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

        self.line_count += 1
        if self.line_count > parameters.max_num_lines_before_write:
            self.line_count = 0
            with open(self.filename, 'wb') as file_handle:
                pickle.dump(self.dictionary, file_handle)

class DataLoader:
    """ Utility for loading saved data"""

    # Constructor
    def __init__(self, filename):
        self.filename = filename

    def load(self):
        """ Load a d dcitionary from file"""
        with open(self.filename, 'rb') as file_handle:
            loaded_dict = pickle.load(file_handle)
        return loaded_dict

class CameraSensor:
    """Class to hold a camera sensor data. Not needed for lab 1."""

    # Constructor
    def __init__(self, camera_id):
        self.camera_id = camera_id
        self.cap = cv2.VideoCapture(camera_id)
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.parameters)

    def get_signal(self, last_camera_signal):
        """Get a new pose estimate from a camera image"""
        camera_signal = last_camera_signal
        ret, pose_estimate = self.get_pose_estimate()
        if ret:
            camera_signal = pose_estimate

        return camera_signal

    def get_pose_estimate(self):
        """If there is a new image, calculate a pose estimate from the fiducial tag on the robot."""
        ret, frame = self.cap.read()
        if not ret:
            return False, []

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = self.detector.detectMarkers(gray)
        if ids is not None:
            # Estimate pose for each detected marker
            for i in range(len(ids)):
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[i], parameters.marker_length, camera_matrix, dist_coeffs)
                pose_estimate = [tvec[0][0][0], tvec[0][0][1], tvec[0][0][2], rvec[0][0][0], rvec[0][0][1], rvec[0][0][2]]
            return True, pose_estimate

        return False, []

    def close(self):
        """ Close the camera stream"""
        self.cap.release()
        cv2.destroyAllWindows()


class MsgSender:
    """ Class to hold a message sender"""

    # Time step size between message to robot sends, in seconds
    delta_send_time = 0.1

    # Constructor
    def __init__(self, last_send_time, msg_size, udp_communication):
        self.last_send_time = last_send_time
        self.msg_size = msg_size
        self.udp_communication = udp_communication

    def send_control_signal(self, control_signal):
        """Pack and send a control signal to the robot."""
        packed_send_msg = self.pack_msg(control_signal)
        self.send(packed_send_msg)

    def send(self, msg):
        """If its time, send the control signal to the robot."""
        new_send_time = time.perf_counter()
        if new_send_time - self.last_send_time > self.delta_send_time:
            message = ""
            for data in msg:
                message = message + str(data)
            self.udp_communication.send_msg(message)
            self.last_send_time = new_send_time

    def pack_msg(self, msg):
        """Pack a message so it is in the correct format for the robot to receive it."""
        packed_msg = ""
        for data in msg:
            if packed_msg == "":
                packed_msg = packed_msg + str(data)
            else:
                packed_msg = packed_msg + ", "+ str(data)
        packed_msg = packed_msg + "\n"
        return packed_msg

class RobotSensorSignal:
    """ A storage vessel for an instance of a robot signal"""

    # Constructor
    def __init__(self, unpacked_msg):
        self.encoder_counts = int(unpacked_msg[0])
        self.steering = int(unpacked_msg[1])
        self.num_lidar_rays = int(unpacked_msg[2])
        self.angles = []
        self.distances = []
        for i in range(self.num_lidar_rays):
            index = 3 + i*2
            self.angles.append(unpacked_msg[index])
            self.distances.append(unpacked_msg[index+1])

    def print(self):
        """ Print the robot sensor signal contents."""
        print("Robot Sensor Signal")
        print(" encoder: ", self.encoder_counts)
        print(" steering:" , self.steering)
        print(" num_lidar_rays: ", self.num_lidar_rays)
        print(" angles: ",self.angles)
        print(" distances: ", self.distances)

    def to_list(self):
        """ Convert the sensor signal to a list of ints and floats."""
        sensor_data_list = []
        sensor_data_list.append(self.encoder_counts)
        sensor_data_list.append(self.steering)
        sensor_data_list.append(self.num_lidar_rays)
        for i in range(self.num_lidar_rays):
            sensor_data_list.append(self.angles[i])
            sensor_data_list.append(self.distances[i])

        return sensor_data_list

class MsgReceiver:
    """ The robot's message receiver"""

    # Determines how often to look for incoming data from the robot.
    delta_receive_time = 0.05

    # Constructor
    def __init__(self, last_receive_time, msg_size, udp_communication):
        self.last_receive_time = last_receive_time
        self.msg_size = msg_size
        self.udp_communication = udp_communication

    def receive(self):
        """Check if its time to look for a new message from the robot."""
        new_receive_time = time.perf_counter()
        if new_receive_time - self.last_receive_time > self.delta_receive_time:
            received_msg = self.udp_communication.receive_msg()
            self.last_receive_time = new_receive_time
            return True, received_msg

        return False, ""

    def unpack_msg(self, packed_msg):
        """Given a new message, put it in a digestable format"""
        unpacked_msg = []
        msg_list = packed_msg.split(',')
        if len(msg_list) >= self.msg_size:
            for data in msg_list:
                unpacked_msg.append(float(data))
            return True, unpacked_msg

        return False, unpacked_msg

    def receive_robot_sensor_signal(self, last_robot_sensor_signal):
        """Check for new message and unpack it if there is one"""
        robot_sensor_signal = last_robot_sensor_signal
        receive_ret, packed_receive_msg = self.receive()
        if receive_ret:
            unpack_ret, unpacked_receive_msg = self.unpack_msg(packed_receive_msg)
            if unpack_ret:
                robot_sensor_signal = RobotSensorSignal(unpacked_receive_msg)

        return robot_sensor_signal

class Robot:
    """ The core robot class"""

    def __init__(self):
        self.connected_to_hardware = False
        self.running_trial = False
        self.extra_logging = False
        self.trial_start_time = 0
        self.msg_sender = None
        self.msg_receiver = None
        self.camera_sensor = CameraSensor(parameters.camera_id)
        self.data_logger = DataLogger(parameters.filename_start, parameters.data_name_list)
        self.robot_sensor_signal = RobotSensorSignal([0, 0, 0])
        self.camera_sensor_signal = [0,0,0,0,0,0]
        print("New robot!")

    def setup_udp_connection(self, udp_communication):
        """Create udp senders and receiver instances with the udp communication"""
        self.msg_sender = MsgSender(time.perf_counter(), parameters.num_robot_control_signals, udp_communication)
        self.msg_receiver = MsgReceiver(time.perf_counter(), parameters.num_robot_sensors, udp_communication)
        print("Reset msg_senders and receivers!")

    def eliminate_udp_connection(self):
        """Step udp senders and receiver instances with the udp communication"""
        self.msg_sender = None
        self.msg_receiver = None
        print("Eliminate UDP !!!")

    # One iteration of the control loop to be called repeatedly
    def control_loop(self, cmd_speed = 0, cmd_steering_angle = 0, logging_switch_on = False):
        """ One iteration of the robot control loop to be called repeatedly"""
        # Receive msg
        if self.msg_sender is not None:
            self.robot_sensor_signal = self.msg_receiver.receive_robot_sensor_signal(self.robot_sensor_signal)

        # Update control signals
        control_signal = [cmd_speed, cmd_steering_angle]

        # Send msg
        if self.msg_receiver is not None:
            self.msg_sender.send_control_signal(control_signal)

        # Log the data
        self.data_logger.log(logging_switch_on, time.perf_counter(), control_signal, self.robot_sensor_signal, self.camera_sensor_signal)
