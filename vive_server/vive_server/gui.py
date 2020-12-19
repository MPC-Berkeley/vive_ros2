from abc import ABC, abstractmethod
import queue
from pathlib import Path
import math

from dearpygui.simple import *
from dearpygui.core import *

from models import Configuration

RED = [255, 0, 0, 255]
PURPLE = [128, 0, 128, 255]
GREEN = [0, 255, 0, 255]
BLUE = [0, 0, 255, 255]
GREY = [128, 128, 128, 255]
GRIDLINES = [128, 128, 128, 50]
BLACK = [0, 0, 0, 255]

TRACKER_COLOR = [0, 255, 255, 255]
REFERENCE_COLOR = [255, 0, 255, 255]
CONTROLLER_COLOR = [255, 255, 255, 255]


class Page(ABC):
    def __init__(self, name: str, gui_manager):
        self.name = name
        self.gui_manager = gui_manager

    def show(self) -> bool:
        if not does_item_exist(self.name):
            add_window(self.name, autosize=True, on_close=self.clear)
            return True
        return False

    @abstractmethod
    def update(self, system_state: dict):
        pass

    def clear(self, sender, data):
        delete_item(self.name)


# render 3d scene from the top down (size of dot represent the scale on the z)
# Moving in and out changes the x and y axis by changing the virtual camera configuration
class Scene:
    def __init__(self, width=1000, height=500, name="scene"):
        self.name = name
        self.width = width
        self.height = height
        self.scale_x, self.scale_y = self.width / 10, self.width / 10
        self.z_scale, self.z_offset = 10, 1.0

        self.center = [self.width / 2, self.height / 2]
        self.bottom_left = [self.width, self.height]

    def add(self):
        add_spacing()
        add_drawing(self.name, width=self.width, height=self.height)
        set_mouse_wheel_callback(self.mouse_wheel)

    def real_pose_from_pixels(self, point):
        return [(point[0] - self.center[0]) / self.scale_x, (point[1] - self.center[1]) / self.scale_y]

    def real_pose_to_pixels(self, point):
        return [(point[0] * self.scale_x + self.center[0]), (point[1] * self.scale_y + self.center[1])]

    def mouse_wheel(self, sender, data):
        if get_active_window() == get_item_parent(self.name):
            self.scale_x, self.scale_y = self.scale_x + data[1] * 3, self.scale_y + data[1] * 3
            self.z_scale += data[1] / 2

    def draw_tracker(self, tracker_msg):
        point = self.real_pose_to_pixels([tracker_msg.x, tracker_msg.y])
        diameter = abs(tracker_msg.z) + self.z_offset * self.z_scale
        draw_text(self.name, [point[0], point[1] - diameter - 15], f'{tracker_msg.device_name}', color=TRACKER_COLOR,
                  size=13,
                  tag=f"{tracker_msg.device_name}txt")
        draw_circle(self.name, point, diameter, TRACKER_COLOR, fill=TRACKER_COLOR,
                    tag=f"{tracker_msg.device_name}dot")
        _, _, yaw = tracker_msg.rotation_as_scipy_transform().as_euler("xyz")
        radius = 20 + diameter / 2
        pt2 = [point[0] - radius * math.cos(yaw), point[1] + radius * math.sin(yaw)]
        draw_line(self.name, point, pt2, PURPLE, 3, tag=f"{tracker_msg.device_name}line")

    def draw_reference(self, reference_msg):
        pass

    def draw_scales(self):
        tick_h = 5
        for x in range(0, self.width, 50):
            draw_line(self.name, [x, self.height], [x, 0], GRIDLINES, 1, tag=f"{x}xgridline")
            draw_line(self.name, [x, self.height], [x, self.height - tick_h], GREY, 1, tag=f"{x}xtick")
            x_real = self.real_pose_from_pixels([x, 0])[0]
            draw_text(self.name, [x, self.height - tick_h - 20], f'{round(x_real, 1)}m', color=GREY, size=13,
                      tag=f"{x}xticktext")
        for y in range(0, self.height, 50):
            draw_line(self.name, [0, y], [self.width, y], GRIDLINES, 1, tag=f"{y}ygridline")
            draw_line(self.name, [0, y], [tick_h, y], GREY, 1, tag=f"{y}ytick")
            y_real = self.real_pose_from_pixels([0, y])[1]
            draw_text(self.name, [tick_h + 5, y - 2], f'{round(y_real, 1)}m', color=GREY, size=13,
                      tag=f"{y}yticktext")

    def add_axes(self):
        length = 40
        draw_line("scene", self.center, [self.center[0], self.center[1] + length], GREEN, 3, tag="axis1")
        draw_line("scene", self.center, [self.center[0] + length, self.center[1]], RED, 3, tag="axis2")
        draw_circle("scene", self.center, 4, BLUE, fill=BLUE,
                    tag="axis3")

    def draw(self, device_state):
        clear_drawing("scene")
        draw_rectangle("scene", [0, 0], self.bottom_left, BLACK, fill=BLACK, tag="backround")
        self.draw_scales()
        self.add_axes()
        for device in device_state:
            if 'tracker' in device:
                if device_state[device] is not None:
                    self.draw_tracker(device_state[device])


class DevicesPage(Page):
    def __init__(self, gui_manager, name="devices"):
        super().__init__(name, gui_manager)
        self.devices_shown = []

    def update(self, system_state):
        for device in system_state:
            serial = system_state[device].serial_num
            if device not in self.devices_shown:
                self.devices_shown.append(device)
                add_input_text(f"{device}:{serial}##name", default_value=system_state[device].device_name,
                               on_enter=True, callback=self.update_device_name,
                               callback_data=(device, serial))
                add_text(f"{serial}_txt", color=GREY)
            else:
                set_value(f"{serial}_txt", f"x: {round(system_state[device].x, 2)}, "
                                           f"y: {round(system_state[device].y, 2)}, "
                                           f"z: {round(system_state[device].z, 2)}")

    def update_device_name(self, sender, data):
        device, serial = data
        new_name = get_value(f"{device}:{serial}##name")
        config = self.gui_manager.get_config()
        config.name_mappings[serial] = new_name
        self.gui_manager.update_config(config)

    def clear(self, sender, data):
        super(DevicesPage, self).clear(sender, data)
        self.devices_shown = []


# Calibration page includes scene with special configuration
class CalibrationPage(Page):
    def __init__(self, name: str, gui_manager):
        super(CalibrationPage, self).__init__(name, gui_manager)
        self.trackers = []
        self.origin_tracker = None
        self.pos_x_tracker = None
        self.pos_y_tracker = None

    def show(self):
        if super(CalibrationPage, self).show():
            with window(self.name):
                add_text("instructions##calibration", default_value="Please select a tracker for "
                                                                    "each axis. Available trackers "
                                                                    "are listed below for convenience:")
                add_spacing()
                add_text("trackers##calibration", default_value=str(self.trackers))
                add_input_text(f"origin##calibration", default_value="", callback=self.update_origin)
                add_input_text(f"+x##calibration", default_value="", callback=self.update_pos_x)
                add_input_text(f"+y##calibration", default_value="", callback=self.update_pos_y)
                add_button("Start calibration", callback=self.run_calibration)

    def update_origin(self, sender, data):
        self.origin_tracker = get_value("origin##calibration")

    def update_pos_x(self, sender, data):
        self.pos_x_tracker = get_value("+x##calibration")

    def update_pos_y(self, sender, data):
        self.pos_y_tracker = get_value("+y##calibration")

    def run_calibration(self, sender, data):
        # verify valid input (trackers + unique)
        if self.origin_tracker in self.trackers and \
                self.pos_y_tracker in self.trackers and \
                self.pos_x_tracker in self.trackers and \
                self.origin_tracker != self.pos_x_tracker and \
                self.origin_tracker != self.pos_y_tracker and \
                self.pos_x_tracker != self.pos_y_tracker:
            self.gui_manager.call_calibration(self.origin_tracker, self.pos_x_tracker, self.pos_y_tracker)
        else:
            log_warning("Invalid tracker entered for calibration")

    def update(self, system_state: dict):
        trackers = []
        for key in system_state:
            if "tracker" in key:
                trackers.append(system_state[key].device_name)
        if len(trackers) > len(self.trackers):
            self.trackers = trackers
            set_value("trackers##calibration", str(trackers))

    def clear(self, sender, data):
        super(CalibrationPage, self).clear(sender, data)
        self.trackers = []


class TestCalibrationPage:
    def __init__(self):
        pass


class ConfigurationPage(Page):
    def show(self):
        super(ConfigurationPage, self).show()

    def update(self, system_state):
        config = self.gui_manager.get_config()
        if config is not None:
            config_dict = dict(self.gui_manager.get_config())
            for value in config_dict:
                if not does_item_exist(f"{value}##config"):
                    add_input_text(f"{value}##config", default_value=str(config_dict[value]),
                                   on_enter=True, callback=self.update_config_entry,
                                   callback_data=value)
                else:
                    set_value(f"{value}##config", str(config_dict[value]))

    def update_config_entry(self, sender, data):
        config = self.gui_manager.get_config()


class VisualizationPage:
    def __init__(self, gui_manager):
        self.gui_manager = gui_manager
        self.scene = Scene()
        self.devices_page = DevicesPage(name="Devices List", gui_manager=self.gui_manager)
        self.configuration_page = ConfigurationPage(name="Configuration", gui_manager=self.gui_manager)
        self.calibrattion_page = CalibrationPage(name="Calibration", gui_manager=self.gui_manager)

    def show(self):
        add_button("Save Configuration", callback=self.save_config)
        add_same_line()
        add_button("Refresh", callback=self.refresh)
        add_same_line()
        add_button("Calibrate", callback=self.calibrate)
        add_same_line()
        add_button("Test Calibration", callback=self.test_calibration)
        add_same_line()
        add_button("List Devices", callback=self.list_devices)
        add_same_line()
        add_button("Show Configuration", callback=self.show_configuration)
        add_same_line()
        add_button("Logs", callback=self.logs)
        self.scene.add()

    def save_config(self, sender, data):
        self.gui_manager.save_config()

    def refresh(self, sender, data):
        self.gui_manager.refresh_system()

    def calibrate(self, sender, data):
        self.calibrattion_page.show()

    def test_calibration(self, sender, data):
        pass

    def list_devices(self, sender, data):
        self.devices_page.show()

    def show_configuration(self, sender, data):
        self.configuration_page.show()

    def logs(self, sender, data):
        show_logger()

    def update(self, system_state: dict):
        self.scene.draw(system_state)
        if does_item_exist("Devices List"):
            self.devices_page.update(system_state)
        if does_item_exist("Configuration"):
            self.configuration_page.update(system_state)
        if does_item_exist("Calibration"):
            self.calibrattion_page.update(system_state)

    def clear(self):
        pass


class GuiManager:
    def __init__(self, pipe, logging_queue):
        self._pipe = pipe
        self._logging_queue = logging_queue
        self._server_config: Configuration() = None
        self._page = VisualizationPage(self)

    def on_render(self, sender, data):
        while self._logging_queue.qsize() > 0:
            try:
                record = self._logging_queue.get_nowait()
                message = record.getMessage()
                logging_level = record.levelname
                if logging_level == "DEBUG":
                    log_debug(message)
                elif logging_level == "INFO":
                    log_info(message)
                elif logging_level == "WARNING":
                    log_warning(message)
                else:
                    log_error(message)
            except queue.Empty:
                pass

        system_state = {}
        while self._pipe.poll():
            data = self._pipe.recv()
            if "state" in data:
                system_state = data["state"]
            if "config" in data:
                self._server_config = data["config"]
        self._page.update(system_state)

    def update_config(self, config):
        self._server_config = config
        self._pipe.send({"config": self._server_config})

    def get_config(self) -> Configuration:
        if self._server_config is not None:
            return self._server_config.copy()

    def save_config(self, path: Path = None):
        self._pipe.send({"save": path})

    def refresh_system(self):
        self._pipe.send({"refresh": None})

    def call_calibration(self, origin, pos_x, pos_y):
        self._pipe.send({"calibrate": (origin, pos_x, pos_y)})

    # Will Run the main gui
    def start(self):
        with window("Vive Server", autosize=True, x_pos=20, y_pos=20):
            self._page.show()

        set_render_callback(self.on_render)
        start_dearpygui()
