# ---------------- TEMPLATE ---------------------------------------
# This is a template to help you start writing PythonBridge code  -
# -----------------------------------------------------------------

import rtmaps.core as rt
import rtmaps.types
from rtmaps.base_component import BaseComponent  # base class
from tracker.tracker import Tracker3D
import numpy as np
import os
from tracker.box_op import *
#from object import Object
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
        self.add_input("objects", rtmaps.types.REAL_OBJECT)  # define an input
        self.add_input("pose", rtmaps.types.FLOAT32)  # define an input
        #self.add_input("calib", rtmaps.types.CUSTOM_STRUCT,typename="Calib")  # define an input
        #self.add_input("P2", rtmaps.types.FLOAT32)
        self.add_input("V2C", rtmaps.types.FLOAT32)

        # Define the output. The type is set to AUTO which means that the
        # output will be typed automatically.
        # You don’t need to set the buffer_size, in that case it will be set
        # automatically.
        self.add_output("oobjects", rtmaps.types.REAL_OBJECT)
        self.outputs["oobjects"].alloc_output_buffer(20)
        #self.add_output("ids", rtmaps.types.AUTO)


# Birth() will be called once at diagram execution startup

    def Birth(self):
        # Start the external binary
        return

        # Core() is called every time you have a new inputs available, depending on
        # your chosen reading policy

    def Core(self):
        # Communicate with the process through its standard input and output
        objects = self.inputs["objects"].ioelt.data
        #calib = self.inputs["calib"].ioelt.data
        #P2 = self.inputs["P2"].ioelt.data.reshape(3, 4)
        V2C = self.inputs["V2C"].ioelt.data.reshape(4, 4)
        pose = self.inputs["pose"].ioelt.data.reshape(4, 4)
        ts = self.inputs["objects"].ioelt.ts

        objects = write_one_tick(ts, V2C, pose,
                                 objects)

        self.outputs["oobjects"].write(objects, ts)
# Death() will be called once at diagram execution shutdown

    def Death(self):
        #self.output_file.close()
        print("Passing through Death()")


def write_one_tick(ts, V2C, pose, objs):
    pose = np.mat(pose).I
    for real in objs:
        #updated_state, score = ob, 1
        #print(real)
        box = register_bbs(np.array(
            [[real.x, real.y, real.z, real.data.length, real.data.width, real.data.height,  real.data.theta]]), pose)

        box[:, 6] = -box[:, 6] - np.pi / 2
        box[:, 2] -= box[:, 5] / 2
        #box[:, 0:3] = velo_to_cam(box[:, 0:3], V2C)[:, 0:3]

        box = box[0]

        real.x = box[0]
        real.y = box[1]
        real.z = box[2]
        #self.color = 0
        #self.misc1 = 0
        #self.misc2 = 0
        #self.misc3 = 0
        real.data.theta = box[6]
        #veh.speed = 0.0
        real.data.width = box[4]
        real.data.height = box[5]
        real.data.length = box[3]
        #veh.model = 0
        #veh.braking = False
        #veh.confidence = 0.0
        #veh.dx = 0.0
        #veh.dy = 0.0
        #veh.dz = 0.0

    return objs


def velo_to_cam(cloud, vtc_mat):
    mat = np.ones(shape=(cloud.shape[0], 4), dtype=np.float32)
    mat[:, 0:3] = cloud[:, 0:3]
    mat = np.mat(mat)
    normal = np.mat(vtc_mat)
    normal = normal[0:3, 0:4]
    transformed_mat = normal * mat.T
    T = np.array(transformed_mat.T, dtype=np.float32)
    return T
