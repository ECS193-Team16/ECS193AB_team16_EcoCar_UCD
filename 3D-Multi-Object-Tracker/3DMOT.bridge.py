# ---------------- TEMPLATE ---------------------------------------
# This is a template to help you start writing PythonBridge code  -
# -----------------------------------------------------------------

import rtmaps.core as rt
import rtmaps.types
from rtmaps.base_component import BaseComponent  # base class
from tracker.tracker import Tracker3D
from tracker.config import cfg, cfg_from_yaml_file
import numpy as np
import os
from tracker.box_op import *
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
        print("objects",self.inputs["objects"])
        print("det",self.inputs["det_scores"])
        print("pose",self.inputs["pose"])
        objects = self.inputs["objects"].ioelt.data.reshape(-1,7)
        det_scores = self.inputs["det_scores"].ioelt.data
        pose = self.inputs["pose"].ioelt.data.reshape(4,4)
        timestamp = self.inputs["objects"].ioelt.ts

        print("tracking",timestamp)
        print("object",objects)
        print("pose",pose)
        print("detscore",pose)

        bbs, ids = self.tracker.tracking(objects[:, :7],
                                         features=None,
                                         scores=det_scores,
                                         pose=pose,
                                         timestamp=timestamp)

        print("bbs",bbs)
        print("ids",ids)
        save_path = "~/tmp/rtmaps/pred.txt"
        tracking_type = ["Car"]
        if not os.path.exists(save_path):
            os.makedirs(save_path)

        tracks = self.tracker.active_trajectories
        for ob_id in tracks.keys():
            track = tracks[ob_id]
            ob = track.trajectory[timestamp]
            updated_state, score = np.array(ob.updated_state.T), ob.score
            box_template = np.zeros(shape=(1, 7))
            box_template[0, 0:3] = updated_state[0, 0:3]
            box_template[0, 3:7] = updated_state[0, 9:13]

            box = register_bbs(box_template, pose)

            box[:, 6] = -box[:, 6] - np.pi / 2
            box[:, 2] -= box[:, 5] / 2
            box[:, 0:3] = velo_to_cam(box[:, 0:3], V2C)[:, 0:3]

            box = box[0]

            box2d = bb3d_2_bb2d(box, P2)
            print("saving")
            with open(save_name, 'w+') as f:
                print('%d %d %s -1 -1 -10 %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f'
                      % (timestamp, ob_id, tracking_type,
                         box2d[0][0], box2d[0][1], box2d[0][2],
                         box2d[0][3],
                         box[5], box[4], box[3], box[0], box[1], box[2], box[6],
                         score), file=f)

            self.outputs["bbs"].write(bbs, timestamp)
            self.outputs["ids"].write(ids, timestamp)

# Death() will be called once at diagram execution shutdown

    def Death(self):
        print("Passing through Death()")
