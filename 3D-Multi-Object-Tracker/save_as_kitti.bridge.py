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
        self.add_input("P2", rtmaps.types.FLOAT32)
        self.add_input("V2C", rtmaps.types.FLOAT32)

        # Define the output. The type is set to AUTO which means that the
        # output will be typed automatically.
        # You don’t need to set the buffer_size, in that case it will be set
        # automatically.
        #self.add_output("bbs", rtmaps.types.AUTO)
        #self.add_output("ids", rtmaps.types.AUTO)

        self.add_property(
            "output_file", "/home/lulu/Projects/EcoCar/3D-Multi-Object-Tracker/evaluation/test/rt.txt", subtype=rtmaps.types.FILE)

# Birth() will be called once at diagram execution startup
    def Birth(self):
        # Start the external binary
        output_file = self.properties["output_file"].data
        print(output_file)
        self.output_file = open(output_file, 'w+')


# Core() is called every time you have a new inputs available, depending on
# your chosen reading policy


    def Core(self):
        # Communicate with the process through its standard input and output
        objects = self.inputs["objects"].ioelt.data
        #calib = self.inputs["calib"].ioelt.data
        P2 = self.inputs["P2"].ioelt.data.reshape(3, 4)
        V2C = self.inputs["V2C"].ioelt.data.reshape(4, 4)
        pose = self.inputs["pose"].ioelt.data.reshape(4,4)
        ts = self.inputs["objects"].ioelt.ts

        write_one_tick(ts, P2, V2C, pose,
                       objects, self.output_file)
# Death() will be called once at diagram execution shutdown

    def Death(self):
        self.output_file.close()
        print("Passing through Death()")


def write_one_tick(ts, P2, V2C, pose, objs, file):
    pose = np.mat(pose).I
    for real in objs:
        #updated_state, score = ob, 1
        #print(real)
        box = register_bbs(np.array(
            [[real.x, real.y, real.z,real.data.length, real.data.width, real.data.height,  np.deg2rad(real.data.theta)]]), pose)

        box[:, 6] = -box[:, 6] - np.pi / 2
        box[:, 2] -= box[:, 5] / 2
        box[:, 0:3] = velo_to_cam(box[:, 0:3], V2C)[:, 0:3]

        box = box[0]

        box2d = bb3d_2_bb2d(box, P2)
        print('%d %d %s -1 -1 -10 %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f'
              % (ts, real.id, real.data.kind,
                 box2d[0][0], box2d[0][1], box2d[0][2],
                 box2d[0][3],
                 box[5], box[4], box[3], box[0], box[1], box[2], box[6],
                 real.data.confidence), file=file)

def velo_to_cam(cloud,vtc_mat):
    mat=np.ones(shape=(cloud.shape[0],4),dtype=np.float32)
    mat[:,0:3]=cloud[:,0:3]
    mat=np.mat(mat)
    normal=np.mat(vtc_mat)
    normal=normal[0:3,0:4]
    transformed_mat = normal * mat.T
    T=np.array(transformed_mat.T,dtype=np.float32)
    return T
