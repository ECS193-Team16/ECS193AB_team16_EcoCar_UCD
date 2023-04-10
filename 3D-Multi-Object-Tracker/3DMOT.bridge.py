# ---------------- TEMPLATE ---------------------------------------
# This is a template to help you start writing PythonBridge code  -
# -----------------------------------------------------------------

import rtmaps.core as rt
import rtmaps.types
from rtmaps.base_component import BaseComponent  # base class
from tracker.tracker import Tracker3D
from tracker.config import cfg, cfg_from_yaml_file
# Python class that will be called from RTMaps.


class rtmaps_python(BaseComponent):
    # Constructor has to call the BaseComponent parent class
    def __init__(self):
        BaseComponent.__init__(self)  # call base class constructor
    # Dynamic is called frequently:
    # - When loading the diagram
    # - When connecting or disconnecting a wire
    # Here you create your inputs, outputs and properties

    def Dynamic(self):
        # Adding an input called "in" of ANY type
        self.add_input("objects", rtmaps.types.FLOAT32)  # define an input
        self.add_input("pose", rtmaps.types.FLOAT32)  # define an input
        self.add_input("det_scores", rtmaps.types.FLOAT32)  # define an input

        # Define the output. The type is set to AUTO which means that the
        # output will be typed automatically.
        # You don’t need to set the buffer_size, in that case it will be set
        # automatically.
        self.add_output("bbs", rtmaps.types.AUTO)
        self.add_output("ids", rtmaps.types.AUTO)

        self.add_property("config_file", "", subtype=rtmaps.types.FILE)

# Birth() will be called once at diagram execution startup
    def Birth(self):
        # Start the external binary
        yaml_file = self.properties["config_file"]
        yaml_file = yaml_file.data
        print(yaml_file)
        config = cfg_from_yaml_file(yaml_file, cfg)

        self.tracker = Tracker3D(box_type="Kitti",
                                 tracking_features=False,
                                 config=config)


# Core() is called every time you have a new inputs available, depending on
# your chosen reading policy
    def Core(self):
        # Communicate with the process through its standard input and output

        objects = self.inputs["objects"].ioelt.data
        det_scores = self.inputs["det_scores"].ioelt.data
        pose = self.inputs["pose"].ioelt.data
        timestamp = self.inputs["objects"].ioelt.ts

        bbs, ids = self.tracker.tracking(objects[:, :7],
                                         features=None,
                                         scores=det_scores,
                                         pose=pose,
                                         timestamp=timestamp)

        self.write("bbs", bbs)
        self.write("ids", ids)

# Death() will be called once at diagram execution shutdown
    def Death(self):
        print("Passing through Death()")
