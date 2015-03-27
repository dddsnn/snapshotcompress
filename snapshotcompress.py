import bitstring as bs
import struct
import itertools
from bitstring import CreationError

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
        self._values = []
        # largest orientation index
        v = current.values[0] - baseline.values[0]
        if v < -2:
            v = -v
        self._values.append(v)
        # orientation, 9 bit unsigned
        for i in range(1, 4):
            v = current.values[i] - baseline.values[i]
            if v < -(2 ** 8):
                v = -v
            self._values.append(v)
        # position x, y, 18 bit signed
        for i in range(4, 6):
            v = current.values[i] - baseline.values[i]
            if v < -(2 ** 17):
                v = -v
            self._values.append(v)
        # position z, 14 bit unsigned
        v = current.values[6] - baseline.values[6]
        if v < -(2 ** 13):
            v = -v
        self._values.append(v)
        # interacting, 0 if same, 1 otherwise
        self._values.append(abs(current.values[7] - baseline.values[7]))

    @property
    def values(self):
        return self._values

class Configs:
    def __init__(self, position_config, orientation_config):
        self.pos = position_config
        self.ortn = orientation_config

def compress_delta_cube(cube, configs):
    res = bs.BitStream()
    # largest orientation index
    try:
        res += bs.Bits(int=cube.orientation_largest, length=2)
    except CreationError:
        res += bs.Bits(uint=cube.orientation_largest, length=2)
    # orientation
    for x in (cube.orientation_a, cube.orientation_b, cube.orientation_c):
        res += encode_varint(x, configs.ortn, 9)
    # position x,y
    for x in (cube.position_x, cube.position_y):
        res += encode_varint(x, configs.pos, 18)
    # position z
    res += encode_varint(cube.position_z, configs.pos, 14)
    # interacting
    res += bs.Bits(uint=cube.interacting, length=1)
    return res

def encode_varint(n, config, max_len):
    # TODO remove overlap
    # TODO uint version
    last = False
    for i, length in enumerate(config):
        try:
            bits = bs.Bits(int=n, length=length)
            config_idx = i
            break
        except CreationError:
            continue
    else:
        last = True
        config_idx = len(config)
        try:
            bits = bs.Bits(int=n, length=max_len)
        except CreationError:
            bits = bs.Bits(uint=n, length=max_len)
    header = bs.BitStream()
    # write config_idx 1s
    header += bs.pack('{0}*bool'.format(config_idx),
                      *itertools.repeat(True, config_idx))
    # if we're not using the last config, write a 0 to terminate
    if not last:
        header += bs.Bits(bool=False)
    return header + bits

def compress_delta_frame(baseline_frame, current_frame):
    res = bs.BitStream()
    deltas = [DeltaCube(baseline, current)
              for baseline, current in zip(baseline_frame, current_frame)]
    changed_deltas = [(i, d) for (i, d) in enumerate(deltas) if any(d)]
    num_configs = 1
    configs = Configs([5, 9], [5, 8])
    index_list_config = [3, 5]
    # write num of configs (zero unnecessary, always at least 1)
    res += bs.Bits(ue=num_configs - 1)
    # TODO write configs (num of settings and settings, for pos and ortn, last setting implicit (num of bits for the whole thing))
    # TODO make the following a list
    # for every config, the number of cubes encoded with it in the index list
    # can be 0 for one, then a changed bit setup is used, otherwise unchanged
    # cubes can simply be omitted
    num_in_index_list = len(changed_deltas)
    res += bs.Bits(ue=num_in_index_list)
    # TODO use uint version of varint
    res += encode_varint(changed_deltas[0][0], index_list_config, 10)
    res += compress_delta_cube(changed_deltas[0][1], configs)
    for b, c in zip(changed_deltas[:-1], changed_deltas[1:]):
        res += encode_varint(c[0] - b[0], index_list_config, 10)
        res += compress_delta_cube(c[1], configs)
    return res

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

if __name__ == '__main__':
    filename = '/home/dddsnn/Downloads/delta_data.bin'
    data = Data(filename)
#     print(sum(1 for frame in data[6:20] for cube in frame if not cube.position_x))
#     print(min(cube.position_z for frame in data for cube in frame))
    for baseline_frame, current_frame in zip(data[2000:2010], data[2006:2016]):
        print(len(compress_delta_frame(baseline_frame, current_frame)))
#         for baseline, current in zip(baseline_frame, current_frame):
#             delta = DeltaCube(baseline, current)
#             print(delta.position_x)
