"""
OpenVr based Vive tracker server
"""

import argparse
import json
import logging
import logging.handlers
import socket
from multiprocessing import Queue, Process, Pipe
from pathlib import Path
from typing import List
from typing import Optional
import yaml
import numpy as np
import scipy.spatial.transform as transform
import time

from base_server import Server
from gui import GuiManager
from models import ViveDynamicObjectMessage, ViveStaticObjectMessage, Configuration
from triad_openvr import TriadOpenVR


def construct_socket_msg(data: ViveDynamicObjectMessage) -> str:
    """
    Send vive tracker message to socket

    Args:
        data: ViveTracker Message to send

    Returns:
        message in string to send

    """
    json_data = json.dumps(data.json(), sort_keys=False)
    json_data = "&" + json_data
    json_data = json_data + "\r"  # * (512 - len(json_data))
    return json_data


class ViveTrackerServer(Server):
    """
    Defines a UDP vive tracker server that constantly "shout out" messages at (HOST, PORT)

    Utilizes OpenVR as its interaction with SteamVR. For hardware setup, please see this tutorial:
    http://help.triadsemi.com/en/articles/836917-steamvr-tracking-without-an-hmd

    """

    def __init__(self, port: int, pipe: Pipe, logging_queue: Queue,
                 config_path: Path = Path(f"~/vive_ros2/config.yml").expanduser(),
                 use_gui: bool = False, buffer_length: int = 1024, should_record: bool = False,
                 output_file_path: Path = Path(f"~/vive_ros2/data/RFS_track.txt").expanduser()):
        """
        Initialize socket and OpenVR
        
        Args:
            port: desired port to open
            logging_queue: handler with where to send logs
            buffer_length: maximum buffer (tracker_name) that it can listen to at once
            should_record: should record data or not
            output_file_path: output file's path
        """
        super(ViveTrackerServer, self).__init__(port)
        self.logger = logging.getLogger("ViveTrackerServer")
        self.logger.addHandler(logging.handlers.QueueHandler(logging_queue))
        self.logger.setLevel(logging.INFO)
        self.pipe = pipe
        self.use_gui = use_gui
        self.config_path = config_path
        self.config = Configuration()

        # load the configuration if one exists otherwise create one and set defaults
        if not self.config_path.exists():
            with open(self.config_path, 'w') as f:
                yaml.dump(self.config.dict(), f)
        else:
            with open(self.config_path, 'r') as f:
                data = yaml.load(f, Loader=yaml.FullLoader)
                self.config = self.config.parse_obj(data)

        self.socket = self.initialize_socket()
        self.triad_openvr: Optional[TriadOpenVR] = None
        self.reconnect_triad_vr()

        self.should_record = should_record
        self.output_file_path = output_file_path
        self.output_file = None
        if not self.output_file_path.exists():
            self.output_file_path.parent.mkdir(parents=True, exist_ok=True)
        self.output_file = self.output_file_path.open('w')
        self.buffer_length = buffer_length

    def run(self):
        """
        Initialize a server that runs forever.

        This server can be put into a multi-process module to run concurrently with other servers.

        This server will listen for client's request for a specific tracker's name

        It will compute that tracker's information

        It will then send that information
        Returns:
            None
        """
        self.logger.info(f"Starting server at {self.ip}:{self.port}")
        self.logger.info("Connected VR devices: \n###########\n" + str(self.triad_openvr) + "###########")
        # Main server loop
        while True:
            messages = {"state": {}}
            # Transmit data over the network
            try:
                tracker_name, addr = self.socket.recvfrom(self.buffer_length)
                tracker_name = tracker_name.decode()
                tracker_key = self.resolve_name_to_key(tracker_name)
                if tracker_key in self.get_tracker_keys():
                    message = self.poll_tracker(tracker_key=tracker_key)
                    messages["state"][tracker_key] = message
                    if message is not None:
                        socket_message = construct_socket_msg(data=message)
                        self.socket.sendto(socket_message.encode(), addr)
                        if self.should_record:
                            self.record(data=message)
                else:
                    self.logger.error(f"Tracker {tracker_name} with key {tracker_key} not found")
            except socket.timeout:
                self.logger.info("Did not receive connection from client")
            except Exception as e:
                self.logger.error(e)

            # See if any commands have been sent from the gui
            while self.pipe.poll():
                data = self.pipe.recv()
                if "config" in data:
                    self.config = data["config"]
                    self.logger.info(f"Configuration updated")
                if "save" in data:
                    self.save_config(data["save"])
                if "refresh" in data:
                    self.logger.info("Refreshing system")
                    self.reconnect_triad_vr()
                if "calibrate" in data:
                    self.calibrate_world_frame(*data["calibrate"])

            # Update the GUI
            if self.use_gui:
                # Make sure all trackers are shown in the GUI regardless of if they are being subscribed to
                for tracker_key in self.get_tracker_keys():
                    if tracker_key not in messages["state"]:
                        message = self.poll_tracker(tracker_key=tracker_key)
                        if message is not None:
                            messages["state"][tracker_key] = message
                for reference_key in self.get_tracking_reference_keys():
                    if reference_key not in messages["state"]:
                        message = self.poll_tracking_reference(tracking_reference_key=reference_key)
                        if message is not None:
                            messages["state"][reference_key] = message

                # Always send the current configuration to ensure synchronization with GUI
                messages["config"] = self.config

                self.pipe.send(messages)

    def resolve_name_to_key(self, name):
        """
        Takes in a name that is either assigned to a device serial number
        or a key. Note that the name should not resemble the keys automatically assigned
        to devices.
        """
        keys = list(self.config.name_mappings.keys())
        values = list(self.config.name_mappings.values())
        for i in range(len(values)):
            if values[i] == name:
                serial = keys[i]
                for device_key in self.get_device_keys():
                    if self.get_device(device_key).get_serial() == serial:
                        return device_key
                return keys[i]
        return name

    def clear_calibration(self):
        self.config.Twv_x = float(0)
        self.config.Twv_y = float(0)
        self.config.Twv_z = float(0)
        self.config.Twv_qx = float(0)
        self.config.Twv_qy = float(0)
        self.config.Twv_qz = float(0)
        self.config.Twv_qw = float(1)

    def set_config_calibration_from_matrix(self, T):
        q = transform.Rotation.from_matrix(T[:3, :3]).as_quat()  # x y z w
        t = T[:3, 3]

        self.config.Twv_x = float(t[0])
        self.config.Twv_y = float(t[1])
        self.config.Twv_z = float(t[2])
        self.config.Twv_qx = float(q[0])
        self.config.Twv_qy = float(q[1])
        self.config.Twv_qz = float(q[2])
        self.config.Twv_qw = float(q[3])

    def calibrate_world_frame(self, origin: str, pos_x: str, pos_y: str, duration: float = 2.0):

        self.clear_calibration()

        origin_key = self.resolve_name_to_key(origin)
        pos_x_key = self.resolve_name_to_key(pos_x)
        pos_y_key = self.resolve_name_to_key(pos_y)

        origin_history = []
        pos_x_history = []
        pos_y_history = []
        start = time.time()
        while time.time() - start < duration:
            origin_message = self.poll_tracker(origin_key)
            pos_x_message = self.poll_tracker(pos_x_key)
            pos_y_message = self.poll_tracker(pos_y_key)

            origin_history.append(np.array([origin_message.x, origin_message.y, origin_message.z]))
            pos_x_history.append(np.array([pos_x_message.x, pos_x_message.y, pos_x_message.z]))
            pos_y_history.append(np.array([pos_y_message.x, pos_y_message.y, pos_y_message.z]))

        avg_origin = np.average(np.array(origin_history), axis=0)
        avg_pos_x = np.average(np.array(pos_x_history), axis=0)
        avg_pos_y = np.average(np.array(pos_y_history), axis=0)

        vx = avg_pos_x - avg_origin
        vy = avg_pos_y - avg_origin

        vx /= np.linalg.norm(vx)
        vy /= np.linalg.norm(vy)
        vz = np.cross(vx, vy)

        m_rot = np.array([[*vx, 0],
                          [*vy, 0],
                          [*vz, 0],
                          [0, 0, 0, 1]])

        m_pos = np.array([[1, 0, 0, -avg_origin[0]],
                          [0, 1, 0, -avg_origin[1]],
                          [0, 0, 1, -avg_origin[2]],
                          [0, 0, 0, 1]])

        self.set_config_calibration_from_matrix(m_rot @ m_pos)

    def save_config(self, path: Path = None):
        path = path or self.config_path  # default to self.config_path is path is None
        self.logger.info(f"Saving configuration to {path}")
        with open(path, 'w') as f:
            yaml.dump(self.config.dict(), f)
        self.logger.info(f"Configuration saved successfully!")

    def poll_tracker(self, tracker_key) -> Optional[ViveDynamicObjectMessage]:
        """
        Polls tracker message by name

        Note:
            Server will attempt to reconnect if tracker name is not found.

        Args:
            tracker_key: the vive tracker message intended to poll

        Returns:
            ViveTrackerMessage if tracker is found, None otherwise.
        """
        tracker = self.get_device(key=tracker_key)
        if tracker is not None:
            message: Optional[ViveDynamicObjectMessage] = self.create_dynamic_message(device=tracker,
                                                                                      device_key=tracker_key)
            return message
        else:
            self.reconnect_triad_vr()
        return None

    def poll_controller(self, controller_key) -> Optional[ViveDynamicObjectMessage]:
        """
        Polls controller message by name

        Note:
            Server will attempt to reconnect if tracker name is not found.

        Args:
            controller_key: the vive tracker message intended to poll

        Returns:
            ViveTrackerMessage if tracker is found, None otherwise.
        """
        controller = self.get_device(key=controller_key)
        if controller is not None:
            message: Optional[ViveDynamicObjectMessage] = self.create_dynamic_message(device=controller,
                                                                                      device_key=controller_key)
            return message
        else:
            self.reconnect_triad_vr()
        return None

    def poll_tracking_reference(self, tracking_reference_key) -> Optional[ViveStaticObjectMessage]:
        """
        Polls tracking reference message by name

        Note:
            Server will attempt to reconnect if tracker name is not found.

        Args:
            tracking_reference_key: the vive tracking reference intended to poll

        Returns:
            ViveTrackerMessage if tracker is found, None otherwise.
        """
        tracking_reference = self.get_device(key=tracking_reference_key)
        if tracking_reference is not None:
            message: Optional[ViveStaticObjectMessage] = self.create_static_message(device=tracking_reference,
                                                                                    device_key=tracking_reference_key)
            return message
        else:
            self.reconnect_triad_vr()
        return None

    def get_device(self, key):
        """
        Given tracker name, find the tracker instance

        Args:
            key: desired tracker's name to find

        Returns:
            tracker instance if found, None otherwise
        """
        return self.triad_openvr.devices.get(key, None)

    def get_rot_vw(self) -> transform.Rotation:
        """Get the rotation from the vive frame to the world frame"""
        return transform.Rotation.from_quat([self.config.Twv_qx,
                                             self.config.Twv_qy,
                                             self.config.Twv_qz,
                                             self.config.Twv_qw])

    def get_rot_wv(self) -> transform.Rotation:
        """Get the rotation from the world frame to the vive frame"""
        return transform.Rotation.from_quat([self.config.Twv_qx,
                                             self.config.Twv_qy,
                                             self.config.Twv_qz,
                                             self.config.Twv_qw]).inverse()

    def translate_to_origin(self, x, y, z):
        return x + self.config.Twv_x, y + self.config.Twv_y, z + self.config.Twv_z

    def create_dynamic_message(self, device, device_key) -> Optional[ViveDynamicObjectMessage]:
        """
        Create dynamic object message given device and device name

        Note:
            it will attempt to reconnect to OpenVR if conversion or polling from device went wrong.

        Args:
            device: tracker instance
            device_key: the device's name corresponding to this tracker

        Returns:
            Vive dynamic message if this is a successful conversion, None otherwise

        """
        try:
            _, _, _, r, p, y = device.get_pose_euler()
            x, y, z, qw, qx, qy, qz = device.get_pose_quaternion()

            vel_x, vel_y, vel_z = device.get_velocity()
            p, q, r = device.get_angular_velocity()

            # handle world transform
            rot_vw = self.get_rot_vw()
            x, y, z = rot_vw.apply([x, y, z])
            x, y, z = self.translate_to_origin(x, y, z)

            # bring velocities into the local device frame such that positive x is pointing out the USB port
            rot_lv = transform.Rotation.from_quat([qx, qy, qz, qw]) * transform.Rotation.from_matrix([[0, 1, 0],
                                                                                                      [1, 0, 0],
                                                                                                      [0, 0, -1]])
            vel_x, vel_y, vel_z = rot_lv.apply([vel_x, vel_y, vel_z], inverse=True)
            p, q, r = rot_lv.apply([p, q, r], inverse=True)

            qx, qy, qz, qw = rot_lv.inv().as_quat()

            serial = device.get_serial()
            device_name = device_key if serial not in self.config.name_mappings else self.config.name_mappings[serial]
            message = ViveDynamicObjectMessage(valid=True, x=x, y=y, z=z,
                                               qx=qx, qy=qy, qz=qz, qw=qw,
                                               vel_x=vel_x, vel_y=vel_y, vel_z=vel_z,
                                               p=p, q=q, r=r,
                                               device_name=device_name,
                                               serial_num=serial)
            return message
        except OSError as e:
            self.logger.error(f"OSError: {e}. Need to restart Vive Tracker Server")
            self.reconnect_triad_vr()
        except Exception as e:
            self.logger.error(f"Exception {e} has occurred, this may be because device {device} "
                              f"is either offline or malfunctioned")
            self.reconnect_triad_vr()
            return None

    def create_static_message(self, device, device_key) -> Optional[ViveStaticObjectMessage]:
        """
        Create tracker message given device and device name

        Note:
            it will attempt to reconnect to OpenVR if conversion or polling from tracker went wrong.

        Args:
            device: device instance
            device_key: the device's name corresponding to this tracker

        Returns:
            Vive static message if this is a successful conversion, None otherwise

        """
        try:
            x, y, z, qw, qx, qy, qz = device.get_pose_quaternion()
            x, y, z = self.get_rot_vw().apply([x, y, z])
            x, y, z = self.translate_to_origin(x, y, z)
            serial = device.get_serial()
            device_name = device_key if serial not in self.config.name_mappings else self.config.name_mappings[serial]
            message = ViveStaticObjectMessage(valid=True, x=x, y=y, z=z,
                                              qx=qx, qy=qy, qz=qz, qw=qw,
                                              device_name=device_name,
                                              serial_num=serial)
            return message
        except OSError as e:
            self.logger.error(f"OSError: {e}. Need to restart Vive Tracker Server")
            self.reconnect_triad_vr()
        except Exception as e:
            self.logger.error(f"Exception {e} has occurred, this may be because device {device} "
                              f"is either offline or malfunctioned")
            self.reconnect_triad_vr()
            return None

    def reconnect_triad_vr(self, debug=False):
        """
        Attempt to reconnect to TriadOpenVR

        Notes:
            this method will automatically assign self.triad_openvr

        Args:
            debug: **deprecated flag

        Returns:
            openvr instance
        """
        del self.triad_openvr
        self.triad_openvr = TriadOpenVR()

        if debug:
            self.logger.debug(
                f"Trying to reconnect to OpenVR to refresh devices. "
                f"Devices online:")
            self.logger.info(self.triad_openvr.devices)

    def get_tracker_keys(self) -> List[str]:
        """
        Get a list of trackers

        Returns:
            list of tracker names

        """
        return self.get_device_keys(filters=["tracker"])

    def get_tracking_reference_keys(self) -> List[str]:
        """
        Get a list of tracking references (base stations)

        Returns:
            list of references names

        """
        return self.get_device_keys(filters=["reference"])

    def get_controller_keys(self) -> List[str]:
        """
        Get a list of controllers

        Returns:
            list of controller names

        """
        return self.get_device_keys(filters=["controller"])

    def get_device_keys(self, filters=None) -> List[str]:
        result = []
        for device_name in self.triad_openvr.devices.keys():
            if filters is None:
                result.append(device_name)
            else:
                for s in filters:
                    if s in device_name:
                        result.append(device_name)
        return result

    def record(self, data: ViveDynamicObjectMessage):
        """
        Record the current data

        Args:
            data: current ViveTrackerMessage to record

        Returns:
            None
        """
        x, y, z, qw, qx, qy, qz = data.x, data.y, data.z, data.qw, data.qx, data.qy, data.qz
        recording_data = f"{x}, {y},{z},{qw},{qx},{qy},{qz}"
        m = f"Recording: {recording_data}"
        self.logger.info(m)
        self.output_file.write(recording_data + "\n")


def run_server(port: int, pipe: Pipe, logging_queue: Queue, config: Path, use_gui: bool, should_record: bool = False):
    vive_tracker_server = ViveTrackerServer(port=port, pipe=pipe, logging_queue=logging_queue, use_gui=use_gui,
                                            config_path=config, should_record=should_record)
    vive_tracker_server.run()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Vive tracker server')
    parser.add_argument('--headless', default=False, help='if true will not run the gui')
    parser.add_argument('--port', default=8000, help='port to broadcast tracker data on')
    parser.add_argument('--config', default=f"~/vive_ros2/config.yml",
                        help='tracker configuration file')
    args = parser.parse_args()

    logger_queue = Queue()
    gui_conn, server_conn = Pipe()
    config = Path(args.config).expanduser()
    string_formatter = logging.Formatter(fmt='%(asctime)s|%(name)s|%(levelname)s|%(message)s', datefmt="%H:%M:%S")

    if args.headless:
        p = Process(target=run_server, args=(args.port, server_conn, logger_queue, config, False,))
        p.start()
        try:
            # This should be updated to be a bit cleaner
            while True:
                print(string_formatter.format(logger_queue.get()))
        finally:
            p.kill()
    else:
        p = Process(target=run_server, args=(args.port, server_conn, logger_queue, config, True,))
        p.start()
        try:
            gui = GuiManager(gui_conn, logger_queue)
            gui.start()
        finally:
            p.kill()
