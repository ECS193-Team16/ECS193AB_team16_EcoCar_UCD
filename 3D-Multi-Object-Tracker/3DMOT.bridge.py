# ---------------- TEMPLATE ---------------------------------------
# This is a template to help you start writing PythonBridge code  -
# -----------------------------------------------------------------

import rtmaps.core as rt
import rtmaps.types
from rtmaps.real_objects import *
from rtmaps.base_component import BaseComponent  # base class
from tracker.tracker import Tracker3D
from tracker.config import cfg, cfg_from_yaml_file
import numpy as np
import os
from tracker.box_op import *
#from object import Object
#from calib import Calib
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
        #self.add_input("det_scores", rtmaps.types.FLOAT32)  # define an input

        # Define the output. The type is set to AUTO which means that the
        # output will be typed automatically.
        # You don’t need to set the buffer_size, in that case it will be set
        # automatically.
        self.add_output("oobjects", rtmaps.types.REAL_OBJECT)

        self.outputs["oobjects"].alloc_output_buffer(20)

        self.add_property("config_file", "/home/lulu/Projects/EcoCar/3D-Multi-Object-Tracker/config/global/virconv_mot.yaml", subtype=rtmaps.types.FILE)

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
        self.time=0


# Core() is called every time you have a new inputs available, depending on
# your chosen reading policy

    def Core(self):
        # Communicate with the process through its standard input and output
        #print("objects",self.inputs["objects"])
        #print("det",self.inputs["det_scores"])
        #print("pose",self.inputs["pose"])
        timestamp=self.time
        self.time+=1
        objects = self.inputs["objects"].ioelt.data
        #det_scores = self.inputs["det_scores"].ioelt.data
        pose = self.inputs["pose"].ioelt.data.reshape(4,4)
        timestamp = self.inputs["objects"].ioelt.ts
        print("track",timestamp)
        #print("tracking",timestamp)
        #print("object",objects)
        #print("pose",pose)
        #print("detscore",pose)
        #print("print",np.array(list(map(lambda real : [real.data.height,real.data.width,real.data.length,real.x,real.y,real.z,real.data.theta],objects)),np.float32),timestamp)
        bbs, ids = self.tracker.tracking(np.array(list(map(lambda real : [real.data.height,real.data.width,real.data.length,real.x,real.y,real.z,np.deg2rad(real.data.theta)],objects)),np.float32),
                                         features=None,
                                         scores=np.array(list(map(lambda real:real.data.confidence,objects)),np.float32),
                                         pose=pose,
                                         timestamp=timestamp)

        #print("bbs",bbs)
        #print("ids",ids)
        objs=[]
        for i,box in enumerate(bbs):
            obj=RealObject()
            obj.kind = 0  # 0 = Vehicle, 1 = Sign, 2 = Tree, 3 = Custom
            obj.id = ids[i]
            obj.x = box[0]
            obj.y = box[1]
            obj.z = box[2]
            #self.color = 0
            #self.misc1 = 0
            #self.misc2 = 0
            #self.misc3 = 0
            veh = Vehicle()
            veh.kind = 0  # 0 = Car, 1 = Bus, 2 = Truck, 3 = Bike, 4 = Motorcycle
            veh.theta = np.rad2deg( box[6])
            #veh.speed = 0.0
            veh.width = box[4]
            veh.height = box[5]
            veh.length = box[3]
            #veh.model = 0
            #veh.braking = False
            #veh.confidence = 0.0
            #veh.dx = 0.0
            #veh.dy = 0.0
            #veh.dz = 0.0
            obj.data = veh

            #print(obj)
            objs.append(obj)
        self.outputs["oobjects"].write(objs, timestamp)

# Death() will be called once at diagram execution shutdown

    def Death(self):
        print("Passing through Death()")
