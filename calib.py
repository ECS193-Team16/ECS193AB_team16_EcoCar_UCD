
import struct
import numpy as np
from rtmaps.types import RTMapsClass


class Calib(RTMapsClass):
    rtmaps_type = "Calib"

    def __init__(self, P2=np.zeros(shape=(3, 4)), V2C=np.zeros(shape=(3, 4))):
        self.P2 = P2
        self.V2C = V2C

    def pack(self):
        P2_bytes = self.P2.tobytes()
        V2C_bytes = self.V2C.tobytes()
        packed_data = struct.pack("<48s48s", P2_bytes, V2C_bytes)
        return packed_data

    @classmethod
    def unpack(cls, data):
        P2_bytes, V2C_bytes = struct.unpack("<48s48s", data)
        P2 = np.frombuffer(P2_bytes, dtype=np.float32).reshape((3, 4))
        V2C = np.frombuffer(V2C_bytes, dtype=np.float32).reshape((3, 4))
        return Calib(P2, V2C)
