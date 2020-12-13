from dearpygui.simple import *
from dearpygui.core import *

# render 3d scene from the top down (size of dot represent the scale on the z)
# Moving in and out changes the x and y axis by changing the virtual camera configuration
class Scene:
    def __init__(self):
        pass


# Calibration page includes scene with special configuration
class CalibrationPage:
    def __init__(self):
        pass


class TestCalibrationPage:
    def __init__(self):
        pass


class VisualizationPage:
    def __init__(self):
        pass


class MainGui:
    def __init__(self, pipe, logging_queue):
        self.pipe = pipe
        self.logging_queue = logging_queue

    def on_render(self, sender, data):
        while self.logging_queue.qsize() > 0:
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

        while self.pipe.poll():
            print(self.pipe.recv())

    # Will Run the main gui
    def start(self):
        with window("Vive Server"):
            add_text(name="logg_outputWelcome to the main vive server window")


        show_logger()
        set_render_callback(self.on_render)
        start_dearpygui()
