
# ---------------- TEMPLATE ---------------------------------------
# This is a template to help you start writing PythonBridge code  -
# -----------------------------------------------------------------

import rtmaps.core as rt
import rtmaps.types
from rtmaps.base_component import BaseComponent  # base class

from collections import namedtuple
import utils
import numpy as np
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
        self.add_input("oxt", rtmaps.types.FLOAT64)  # define an input

        # Define the output. The type is set to AUTO which means that the
        # output will be typed automatically.
        # You don’t need to set the buffer_size, in that case it will be set
        # automatically.
        self.add_output("pose", rtmaps.types.AUTO)

    # Birth() will be called once at diagram execution startup

    def Birth(self):
        self.t_0 = []
        self.scale = 0
        return
        # Start the external binary

        # Core() is called every time you have a new inputs available, depending on
        # your chosen reading policy

    def Core(self):
        # Communicate with the process through its standard input and output
        #print(self.inputs["oxt"])
        oxt = self.inputs["oxt"].ioelt.data

        oxt = parse_oxt(oxt)

        if not self.scale:
            self.scale = get_scale(oxt)

        pose = self.calculate_pose(oxt)
        self.last_pose = pose
        self.write("pose", pose)

# Death() will be called once at diagram execution shutdown
    def Death(self):
        print("Passing through Death()")

    def calculate_pose(self, packet):

        er = 6378137.  # earth radius (approx.) in meters

        scale = self.scale
        # Extract relevant oxts data
        tx = scale * packet.lon * np.pi * er / 180.
        ty = scale * er * np.log(np.tan((90. + packet.lat) * np.pi / 360.))
        tz = packet.alt
        t = np.array([tx, ty, tz])
        if len(self.t_0) == 0:
            self.t_0 = t
        # Use the Euler angles to get the rotation matrix
        Rx = utils.rotx(packet.roll)
        Ry = utils.roty(packet.pitch)
        Rz = utils.rotz(packet.yaw)
        R = Rz.dot(Ry.dot(Rx))

        # Combine the translation and rotation into a homogeneous transform
        return (utils.transform_from_rot_trans(R, t - self.t_0))


def parse_oxt(line):
    OxtsPacket = namedtuple('OxtsPacket',
                            'lat, lon, alt, ' +
                            'roll, pitch, yaw, ' +
                            'vn, ve, vf, vl, vu, ' +
                            'ax, ay, az, af, al, au, ' +
                            'wx, wy, wz, wf, wl, wu, ' +
                            'pos_accuracy, vel_accuracy, ' +
                            'navstat, numsats, ' +
                            'posmode, velmode, orimode')

    # Last five entries are flags and counts
    line[:-5] = [float(x) for x in line[:-5]]
    line[-5:] = [int(x) for x in line[-5:]]

    return OxtsPacket(*line)


def get_scale(t_0):
    return np.cos(t_0.lat * np.pi / 180.)
