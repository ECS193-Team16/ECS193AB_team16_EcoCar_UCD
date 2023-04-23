# This is a template to help you start writing PythonBridge code  -
# -----------------------------------------------------------------

import rtmaps.core as rt
import rtmaps.types
from rtmaps.real_objects import *
from rtmaps.base_component import BaseComponent  # base class

from collections import namedtuple
import utils
import numpy as np

from dataset.kitti_dataset import KittiTrackingDataset
from dataset.kitti_data_base import velo_to_cam

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
        #self.add_input("oxt", rtmaps.types.FLOAT64)  # define an input

        # Define the output. The type is set to AUTO which means that the
        # output will be typed automatically.
        # You don’t need to set the buffer_size, in that case it will be set
        # automatically.
        self.add_output("objects", rtmaps.types.REAL_OBJECT)  # define an input
        self.add_output("pose", rtmaps.types.FLOAT32)  # define an input
        #self.add_output("det_scores", rtmaps.types.FLOAT32)  # define an input
        self.add_output("P2", rtmaps.types.FLOAT32)
        self.add_output("V2C", rtmaps.types.FLOAT32)

        self.add_property(
            "dataset_path", "/home/lulu/Projects/EcoCar/Data/training")
        self.add_property("seq_id", 6)
        self.add_property(
            "detections_path", "/home/lulu/Projects/EcoCar/Data/detection/virconv/training")
        self.add_property("min_score", -0.5)
    # Birth() will be called once at diagram execution startup

    def Birth(self):
        dataset_path = self.properties["dataset_path"].data
        seq_id = self.properties["seq_id"].data
        detections_path = self.properties["detections_path"].data
        tracking_type = "Car"
        self.dataset = KittiTrackingDataset(
            dataset_path, seq_id=seq_id, ob_path=detections_path, type=[tracking_type])
        self.time = 0

        self.outputs["objects"].alloc_output_buffer(20)
        #self.outputs["det_scores"].alloc_output_buffer(20)

        self.outputs["pose"].alloc_output_buffer(16)
        return
        # Start the external binary

        # Core() is called every time you have a new inputs available, depending on
        # your chosen reading policy

    def Core(self):
        # Communicate with the process through its standard input and output
        #print(self.inputs["oxt"])
        input_score = self.properties["min_score"].data
        ts = self.time
        if ts >= len(self.dataset):
            return
        P2, V2C, points, image, objects, det_scores, pose = self.dataset[ts]
        #print(self.properties["detections_path"].data)
        #print(P2, V2C, points, image, objects, det_scores, pose)
        #print(ts)
        #print(objects)
        mask = det_scores > input_score
        bbs = objects[mask]
        det_scores = det_scores[mask]

        print("try:", bbs, mask)
        objs = []
        for i, box in enumerate(bbs):
            obj = RealObject()
            obj.kind = 0  # 0 = Vehicle, 1 = Sign, 2 = Tree, 3 = Custom
            #self.id = ids[i]
            obj.x = box[3]
            obj.y = box[4]
            obj.z = box[5]
            #self.color = 0
            #self.misc1 = 0
            #self.misc2 = 0
            #self.misc3 = 0
            veh = Vehicle()
            veh.kind = 0  # 0 = Car, 1 = Bus, 2 = Truck, 3 = Bike, 4 = Motorcycle
            veh.theta = box[6]
            #veh.speed = 0.0
            veh.width = box[1]
            veh.height = box[0]
            veh.length = box[2]
            #veh.model = 0
            #veh.braking = False
            veh.confidence = det_scores[i]
            #veh.dx = 0.0
            #veh.dy = 0.0
            #veh.dz = 0.0
            obj.data = veh

            objs.append(obj)

        #print("objects",objects)
        #print("pose",pose)
        #print("det_scores",det_scores)
        io = rtmaps.types.Ioelt()
        io.data = objs
        io.ts = ts
        io.vector_size = len(objs)
        print("kitti_read:", objs, ts)
        self.outputs["objects"].write(io)
        #self.outputs["objects"].write(objects, ts)
        #self.outputs["det_scores"].write(det_scores, ts)

        self.outputs["pose"].write(pose, ts)
        self.outputs["P2"].write(P2, ts)
        self.outputs["V2C"].write(np.array(V2C, np.float32), ts)

        self.time += 1
# Death() will be called once at diagram execution shutdown

    def Death(self):
        print("Passing through Death()")
