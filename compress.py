import bitstring as bs
import struct

cubesize = 32
cubes_per_frame = 901
framesize = cubesize * cubes_per_frame

class Iter:
    def __init__(self, target, size):
        self.size = size
        self.target = target
        self.current = 0

    def __next__(self):
        if self.current == self.size:
            raise StopIteration
        item = self.target[self.current]
        self.current += 1
        return item

class Data:
    def __init__(self, filename):
        with open(filename, 'rb') as f:
            self.bts = f.read()
        self.frames = [Frame(self.bts, i)
                       for i in range(0, len(self.bts), framesize)]
        self.num_frames = len(self.frames)

    def __getitem__(self, i):
        return self.frames[i]

    def __iter__(self):
        return Iter(self, self.num_frames)

class Frame:
    def __init__(self, bts, offset):
        self.cubes = [Cube(bts, i)
                      for i in range(offset, offset + framesize, cubesize)]

    def __getitem__(self, i):
        return self.cubes[i]

    def __iter__(self):
        return Iter(self, cubes_per_frame)

class Cube:
    def __init__(self, bts, offset):
        self.bts = bts
        self.offset = offset

    @property
    def values(self):
        return struct.unpack('<iiiiiiii',
                             self.bts[self.offset:self.offset + cubesize])
    @property
    def orientation_largest(self):
        return self.values[0]
    @property
    def orientation_a(self):
        return self.values[1]
    @property
    def orientation_b(self):
        return self.values[2]
    @property
    def orientation_c(self):
        return self.values[3]
    @property
    def position_x(self):
        return self.values[4]
    @property
    def position_y(self):
        return self.values[5]
    @property
    def position_z(self):
        return self.values[6]
    @property
    def interacting(self):
        return self.values[7]

    def __iter__(self):
        return self.values.__iter__()

class DeltaCube(Cube):
    def __init__(self, baseline, current):
        self._values = list(map(lambda x:x[1] - x[0],
                          zip(baseline.values, current.values)))
    @property
    def values(self):
        return self._values

def rl_enc(bits):
    def write_zeros():
        nonlocal res
        res += bs.Bits('0b0')
        x = bs.Bits(ue=zero_count)
        res += x
    res = bs.BitStream()
    zero_count = 0
    for b in bits:
        if b:
            # write zeros first
            if zero_count:
                write_zeros()
                zero_count = 0
            # just write 1's normally, they're rare
            res += bs.Bits('0b1')
        else:
            zero_count += 1
    # write any remaining zeros
    if zero_count:
        write_zeros()
    return res

def compress_frame_delta(baseline_frame, current_frame):
    res = bs.BitStream()
    deltas = [DeltaCube(baseline, current)
              for baseline, current in zip(baseline_frame, current_frame)]
    num_changed = sum(1 for d in deltas if any(d))
    print(num_changed)

if __name__ == '__main__':
    filename = '/home/dddsnn/Downloads/delta_data.bin'
    data = Data(filename)
#     frame = data[100]
#     print(sum(1 for frame in data[6:20] for cube in frame if not cube.position_x))
#     print(min(cube.orientation_a for frame in data for cube in frame))
    for baseline_frame, current_frame in zip(data[100:110], data[106:116]):
        compress_frame_delta(baseline_frame, current_frame)
#         for baseline, current in zip(baseline_frame, current_frame):
#             delta = DeltaCube(baseline, current)
#             print(delta.position_x)
