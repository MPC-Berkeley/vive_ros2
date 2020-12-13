from abc import ABC, abstractmethod
import numpy as np
import queue
import scipy.spatial.transform as transform

from dearpygui.simple import *
from dearpygui.core import *

RED = [255, 0, 0, 255]
GREEN = [0, 255, 0, 255]
BLUE = [0, 0, 255, 255]
GREY = [128, 128, 128, 255]
GRIDLINES = [128, 128, 128, 50]
BLACK = [0, 0, 0, 255]

TRACKER_COLOR = [0, 255, 255, 255]
REFERENCE_COLOR = [255, 0, 255, 255]
CONTROLLER_COLOR = [255, 255, 255, 255]


class Page(ABC):
    def __init__(self, name, main_gui):
        self.name = name
        self.main_gui = main_gui

    @abstractmethod
    def show(self):
        pass

    @abstractmethod
    def update(self, device_state):
        pass

    @abstractmethod
    def clear(self, sender, data):
        pass


# render 3d scene from the top down (size of dot represent the scale on the z)
# Moving in and out changes the x and y axis by changing the virtual camera configuration
class Scene:
    def __init__(self, width=1000, height=500, name="scene"):
        self.name = name
        self.width = width
        self.height = height
        self.scale_x, self.scale_y = self.width / 10, self.height / 10
        self.z_scale, self.z_offset = 10, 1.0

        self.center = [self.width / 2, self.height / 2]
        self.bottom_left = [self.width, self.height]

    def add(self):
        add_spacing()
        add_drawing(self.name, width=self.width, height=self.height)
        # set_mouse_click_callback(self.mouse_click)
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
        draw_line("scene", self.center, [self.center[0], self.center[1] - length], GREEN, 3, tag="axis1")
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
    def __init__(self, main_gui, name="devices"):
        super().__init__(name, main_gui)
        self.devices_shown = []

    def show(self):
        if not does_item_exist(self.name):
            add_window(self.name, autosize=True, on_close=self.clear)

    def update(self, device_state):
        for device in device_state:
            serial = device_state[device].serial_num
            if device not in self.devices_shown:
                self.devices_shown.append(device)
                add_input_text(f"{device}:{serial}##name", default_value=device_state[device].device_name,
                               on_enter=True, callback=self.update_device_name,
                               callback_data=(device, serial))
                add_text(f"{serial}_txt", color=GREY)
            else:
                set_value(f"{serial}_txt", f"x: {round(device_state[device].x, 2)}, "
                                           f"y: {round(device_state[device].y, 2)}, "
                                           f"z: {round(device_state[device].z, 2)}")

    def update_device_name(self, sender, data):
        device, serial = data
        new_name = get_value(f"{device}:{serial}##name")
        self.main_gui.update_device_name(serial, new_name)

    def clear(self, sender, data):
        delete_item(self.name)
        self.devices_shown = []


# Calibration page includes scene with special configuration
class CalibrationPage:
    def __init__(self):
        pass


class TestCalibrationPage:
    def __init__(self):
        pass


class VisualizationPage:
    def __init__(self, main_gui):
        self.main_gui = main_gui
        self.scene = Scene()
        self.devices_list = DevicesPage(name="Devices List", main_gui=self.main_gui)

    def show(self):
        add_button("Save Configuration", callback=self.save_config)
        add_same_line()
        add_button("Calibrate", callback=self.calibrate)
        add_same_line()
        add_button("Test Calibration", callback=self.test_calibration)
        add_same_line()
        add_button("List Devices", callback=self.list_devices)
        add_same_line()
        add_button("Logs", callback=self.logs)
        self.scene.add()

    def save_config(self, sender, data):
        self.main_gui.save_configuration()

    def calibrate(self, sender, data):
        pass

    def test_calibration(self, sender, data):
        pass

    def list_devices(self, sender, data):
        self.devices_list.show()

    def logs(self, sender, data):
        show_logger()

    def update(self, device_state):
        self.scene.draw(device_state)
        if does_item_exist("Devices List"):
            self.devices_list.update(device_state)

    def clear(self):
        pass


class MainGui:
    def __init__(self, pipe, logging_queue):
        self.pipe = pipe
        self.logging_queue = logging_queue
        self.page = VisualizationPage(self)

    def on_render(self, sender, data):
        while self.logging_queue.qsize() > 0:
            try:
                record = self.logging_queue.get_nowait()
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

        device_state = {}
        while self.pipe.poll():
            device_state = self.pipe.recv()

        self.page.update(device_state)

    def update_device_name(self, serial, new_name):
        self.pipe.send({"map_name": {serial: new_name}})

    def save_configuration(self):
        self.pipe.send({"save_configuration": ""})

    # Will Run the main gui
    def start(self):
        with window("Vive Server", autosize=True, x_pos=20, y_pos=20):
            self.page.show()

        set_render_callback(self.on_render)
        start_dearpygui()
