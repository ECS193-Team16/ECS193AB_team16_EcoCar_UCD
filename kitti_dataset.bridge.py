# This is a template to help you start writing PythonBridge code  -
# -----------------------------------------------------------------

import rtmaps.core as rt
import rtmaps.types
from rtmaps.real_objects import *
from rtmaps.base_component import BaseComponent  # base class
import os
#from collections import namedtuple
import utils
import numpy as np

from dataset.kitti_data_base import *
#from dataset.kitti_dataset import KittiTrackingDataset
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
        print(self.properties["detections_path"].data)
        #print(P2, V2C, points, image, objects, det_scores, pose)
        #print(ts)
        #print(objects)
        mask = det_scores > input_score
        bbs = objects[mask]
        det_scores = det_scores[mask]

        #print("try:", bbs, mask, ts)
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

        print("objects",objects)
        print("pose",pose)
        print("det_scores",det_scores)
        self.time += 1
        if len(objs)==0:
            return
        io = rtmaps.types.Ioelt()
        io.data = objs
        io.ts = ts
        io.vector_size = len(objs)
        print("kitti_read:", objs, ts)
        self.outputs["objects"].write(io)
        #self.outputs["objects"].write(objects, ts)
        #self.outputs["det_scores"].write(det_scores, ts)

        self.outputs["pose"].write(np.array(pose,np.float32), ts)
        self.outputs["P2"].write(P2, ts)
        self.outputs["V2C"].write(np.array(V2C, np.float32), ts)

# Death() will be called once at diagram execution shutdown

    def Death(self):
        print("Passing through Death()")

class KittiTrackingDataset:
    def __init__(self,root_path,seq_id,ob_path = None,load_image=False,load_points=False,type=["Car"]):
        self.seq_name = str(seq_id).zfill(4)
        self.root_path = root_path
        self.ob_path = os.path.join(ob_path,self.seq_name)
        self.velo_path = os.path.join(self.root_path,"velodyne",self.seq_name)
        self.image_path = os.path.join(self.root_path,"image_02",self.seq_name)
        self.calib_path = os.path.join(self.root_path,"calib",self.seq_name)
        self.pose_path = os.path.join(self.root_path, "pose", self.seq_name,'pose.txt')
        self.type = type

        self.all_ids = os.listdir(self.velo_path)
        calib_path = self.calib_path + '.txt'

        self.P2, self.V2C = read_calib(calib_path)
        self.poses = read_pose(self.pose_path)
        self.load_image = load_image
        self.load_points = load_points

        #self.ob_path = ob_path

        self.items=[]
        for i in range(len(self)):
            self.items.append(self.getitem(i))
            print(self.items[-1])

    def __len__(self):
        return len(self.all_ids)-1

    def __getitem__(self,item):
        print(self.items[item],"\n\n")
        return self.items[item]

    def getitem(self, item):

        name = str(item).zfill(6)

        velo_path = os.path.join(self.velo_path,name+'.bin')
        image_path = os.path.join(self.image_path, name+'.png')

        if self.load_points:
            points = read_velodyne(velo_path,self.P2,self.V2C)
        else:
            points = None
        if self.load_image:
            image = read_image(image_path)
        else:
            image = None

        if item in self.poses.keys():
            pose = self.poses[item]
        else:
            pose = None

        if self.ob_path is not None:
            ob_path = os.path.join(self.ob_path, name + '.txt')
            print(ob_path)
            if not os.path.exists(ob_path):
                objects = np.zeros(shape=(0, 7))
                det_scores = np.zeros(shape=(0,))
            else:
                objects_list = []
                det_scores = []
                with open(ob_path) as f:
                    for each_ob in f.readlines():
                        infos = re.split(' ', each_ob)
                        if infos[0] in self.type:
                            objects_list.append(infos[8:15])
                            det_scores.append(infos[15])
                if len(objects_list)!=0:
                    objects = np.array(objects_list,np.float32)
                    objects[:, 3:6] = cam_to_velo(objects[:, 3:6], self.V2C)[:, :3]
                    det_scores = np.array(det_scores,np.float32)
                else:
                    objects = np.zeros(shape=(0, 7))
                    det_scores = np.zeros(shape=(0,))
        else:
            objects = np.zeros(shape=(0,7))
            det_scores = np.zeros(shape=(0,))

        return self.P2,self.V2C,points,image,objects,det_scores,pose
