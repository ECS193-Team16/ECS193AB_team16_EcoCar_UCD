import struct
import numpy as np

from rtmaps.types import RTMapsClass


class Object(RTMapsClass):
    rtmaps_type = "Object"

    def __init__(self, id=-1, type='', box=np.zeros(shape=(7)), box2D=np.zeros(shape=(4)), score=-1):
        self.id = id
        self.type = type
        self.box = box
        self.box2D = box2D
        self.score = score

    def pack(self):
        box_bytes = self.box.tobytes()
        box2D_bytes = self.box2D.tobytes()
        packed_data = struct.pack("<i6s7f4f", self.id, self.type.encode(
            'utf-8'), *self.box, *self.box2D, self.score)
        return packed_data

    @classmethod
    def unpack(cls, data):
        id, type_bytes, *box, *box2D, score = struct.unpack("<i6s7f4f", data)
        type = type_bytes.decode('utf-8').rstrip('\0')
        box = np.array(box)
        box2D = np.array(box2D)
        return cls(id, type, box, box2D, score)
