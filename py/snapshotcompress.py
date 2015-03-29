import bitstring as bs
import struct
import itertools
from bitstring import CreationError
import numpy as np
import math

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

    def __getitem__(self, i):
        return self.values[i]

class DeltaCube(Cube):
    def __init__(self, baseline, current, prev_base):
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
        # expected
        # TODO does all this fit?
        gravity = 3
        ground_limit = 105
        self.expected_dx = baseline.values[4] - prev_base.values[4]
        self.expected_dy = baseline.values[5] - prev_base.values[5]
        self.expected_dz = baseline.values[6] - prev_base.values[6]
        drag_x = -math.ceil(self.expected_dx * 0.062)
        drag_y = -math.ceil(self.expected_dy * 0.062)
        drag_z = -math.ceil(self.expected_dz * 0.062)
        self.expected_x = baseline.values[4] + self.expected_dx + drag_x;
        self.expected_y = baseline.values[5] + self.expected_dy + drag_y;
        self.expected_z = max(baseline.values[6] + self.expected_dz - gravity + drag_z, ground_limit);

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
    if any(cube.values[1:4]):
        allsmall = all(x >= -2 and x < 2 for x in cube.values[1:4])
        for x in (cube.orientation_a, cube.orientation_b, cube.orientation_c):
            if allsmall:
                res += bs.Bits(int=x, length=2)
            else:
                res += encode_varint(x, configs.ortn, 9)
    # position x,y
    allsmall = all(e - a >= -8 and e - a < 8 for a, e in ((cube.position_x, cube.expected_dx),
                                 (cube.position_y, cube.expected_dy), (cube.position_z, cube.expected_dz)))
    if any(cube.values[4:7]):
        for actual, expected in ((cube.position_x, cube.expected_dx),
                                 (cube.position_y, cube.expected_dy)):
            if allsmall:
                res += bs.Bits(int=expected - actual, length=4)
            else:
                res += encode_varint(expected - actual, configs.pos, 18)
    # position z
    if allsmall:
        res += bs.Bits(int=cube.expected_dz - cube.position_z, length=4)
    else:
        res += encode_varint(cube.expected_z - cube.position_z, configs.pos, 14)
    # interacting
    res += bs.Bits(uint=cube.interacting, length=1)
    return res

def compute_bounds_signed(config):
    # compute the min and max possible value for every setting
    b = [(-(2 ** (config[0] - 1)), (2 ** (config[0] - 1) - 1))]
    for i, s in enumerate(config[1:]):
        prev_bounds = b[i]
        extension = 2 ** (s - 1)
        b.append((prev_bounds[0] - extension, prev_bounds[1] + extension))
    return b

def compute_bounds_unsigned(config):
    # compute the min and max possible value for every setting
    b = [(0, (2 ** (config[0]) - 1))]
    for i, s in enumerate(config[1:]):
        prev_bounds = b[i]
        extension = 2 ** s
        b.append((prev_bounds[1] + 1, prev_bounds[1] + extension))
    return b

def encode_varint(n, config, max_len, signed=True):
    if signed:
        bounds = compute_bounds_signed(config)
    else:
        bounds = compute_bounds_unsigned(config)
    # go through the bounds list and find the first fitting index
    for i, b in enumerate(bounds):
        if n >= b[0] and n <= b[1]:
            config_idx = i
            last = False
            break
    else:
        # if none fits, set index to use the full length of bits
        config_idx = len(config)
        last = True
    # figure out what number to actually write (avoid overlap, extend the range
    # a little)
    if config_idx == 0 or last:
        num_to_write = n
    else:
        if n < 0:
            num_to_write = n - bounds[config_idx - 1][0]
        else:
            num_to_write = n - bounds[config_idx - 1][1] - 1
    # compute the length to write
    if not last:
        length = config[config_idx]
    else:
        length = max_len
    # actually write the damn thing
    if signed:
        try:
            bits = bs.Bits(int=num_to_write, length=length)
        except CreationError:
            bits = bs.Bits(uint=num_to_write, length=length)
    else:
        bits = bs.Bits(uint=num_to_write, length=length)
    header = bs.BitStream()
    # write config_idx 1s
    header += bs.pack('{0}*bool'.format(config_idx),
                      *itertools.repeat(True, config_idx))
    # if we're not using the last config, write a 0 to terminate
    if not last:
        header += bs.Bits(bool=False)
    return header + bits

def select_configs(deltas):
    max_lens = [None, 9, 9, 9, 18, 18, 14, None]
    pos_sizes = [{} for _ in range(len(deltas))]
    ortn_sizes = [{} for _ in range(len(deltas))]
    static_overheads = {}
    for config_len in range(1, 4):
        for config in config_permutations(config_len, 3, 11, 3):
            # static overhead for this config
            overhead = len(bs.Bits(ue=config_len))
            for s in config:
                overhead += len(bs.Bits(ue=s))
            static_overheads[config] = overhead
            bounds = compute_bounds_signed(config)
            # int sizes for pos and ortn data
            for pos_size, ortn_size, (_, d) in zip(pos_sizes,
                                                   ortn_sizes, deltas):
                size = 0
                # orientation
                for i, field in enumerate(d[1:4], start=1):
                    for j in range(config_len):
                        if field >= bounds[j][0] and field <= bounds[j][1]:
                            # header bits
                            size += j + 1
                            # size of the actual int
                            size += config[j]
                            break
                    else:
                        size += config_len
                        size += max_lens[i]
                ortn_size[config] = size
                size = 0
                # position
                for i, (expected, actual) in enumerate([(d.expected_dx, d.position_x),
                                                      (d.expected_dy, d.position_y),
                                                      (d.expected_dz, d.position_z)],
                                                     start=4):
                    num = expected - actual
                    for j in range(config_len):
                        if num >= bounds[j][0] and num <= bounds[j][1]:
                            size += j + 1
                            size += config[j]
                            break
                    else:
                        size += config_len
                        size += max_lens[i]
                pos_size[config] = size
    # dicts config->overall performance
    pos_perfs = dict(static_overheads)
    ortn_perfs = dict(static_overheads)
    for d in pos_sizes:
        for k in pos_perfs.keys():
            pos_perfs[k] += d[k]
    for d in ortn_sizes:
        for k in ortn_perfs.keys():
            ortn_perfs[k] += d[k]
    best_pos = sorted(pos_perfs.items(), key=lambda x:x[1])[0][0]
    best_ortn = sorted(ortn_perfs.items(), key=lambda x:x[1])[0][0]
    configs = [Configs(best_pos, best_ortn)]
    # TODO more configs
    print('config: {0},{1}'.format(configs[0].ortn, configs[0].pos))
    return configs

def config_permutations(num, low, high, step):
    current = [low + (step * i) for i in range(num)]
    stop = [high - (step * i) for i in range(num - 1, -1, -1)]
    if current[-1] > high:
        raise ValueError('num of settings doesn\'t fit into the range')
    yield tuple(current)
    while(current != stop):
        for i in range(len(current) - 1):
            if current[i + 1] - current[i] > step:
                current[i] += 1
                yield tuple(current)
                break
        else:
            current[-1] += 1
            yield tuple(current)

def compress_delta_frame(baseline_frame, current_frame, prev_base_frame):
    res = bs.BitStream()
    deltas = [DeltaCube(baseline, current, prev_base)
              for baseline, current, prev_base in zip(baseline_frame, current_frame, prev_base_frame)]
    changed_deltas = [(i, d) for (i, d) in enumerate(deltas) if any(d)]
    print('{0} changes'.format(len(changed_deltas)))
    # flag whether anything has changed at all
    if not changed_deltas:
        res += bs.Bits(bool=False)
        return res
    else:
        res += bs.Bits(bool=True)
    configs = select_configs(changed_deltas)
#     configs = [Configs((3, 9), (5, 8))]
    num_configs = len(configs)
    index_delta_config = (1, 2, 3, 4, 5, 6)
    # write num of configs (zero unnecessary, always at least 1)
    res += bs.Bits(ue=num_configs - 1)
    # write configs
    for c in configs:
        # position
        res += bs.Bits(ue=len(c.pos))
        for s in c.pos:
            res += bs.Bits(ue=s)
        # orientation
        res += bs.Bits(ue=len(c.ortn))
        for s in c.ortn:
            res += bs.Bits(ue=s)
    # TODO decide which delta gets encoded with which config, maybe have select_configs return a dict?
    # decide whether it's better to use indices or change flags
    index_deltas = [encode_varint(changed_deltas[0][0], index_delta_config, 10,
                         signed=False)]
    for b, c in zip(changed_deltas[:-1], changed_deltas[1:]):
        index_deltas.append(encode_varint(c[0] - b[0], index_delta_config, 10,
                                          signed=False))
    if sum(len(x) for x in index_deltas) < cubes_per_frame:
        # use indices
        # TODO make the following a list
        # for every config, the number of cubes encoded with it in the index list
        # can be 0 for one, then a changed bit setup is used, otherwise unchanged
        # cubes can simply be omitted
        num_in_index_list = len(changed_deltas)
        res += bs.Bits(ue=num_in_index_list)
        for i, d in enumerate(changed_deltas):
            res += index_deltas[i]
            res += compress_delta_cube(d[1], configs[0])
    else:
        # use change flags
        num_in_index_list = 0
        res += bs.Bits(ue=num_in_index_list)
        it = iter(changed_deltas)
        d = next(it)
        for i in range(cubes_per_frame):
            if i == d[0]:
                # write the compressed cube
                res += bs.Bits(bool=True)
                res += compress_delta_cube(d[1], configs[0])
                try:
                    d = next(it)
                except StopIteration:
                    # no cubes left, make sure we don't get here again
                    d = (0,)
        else:
            # just write a zero to signal no change
            res += bs.Bits(bool=False)
    return res

if __name__ == '__main__':
    filename = '/home/dddsnn/Downloads/delta_data.bin'
    print(compute_bounds_unsigned((1, 2, 3, 4, 5, 6)))
    print(encode_varint(6, (1, 2, 3, 4, 5, 6), 10, signed=False).bin)
    data = Data(filename)
    compressed = []
    for baseline_frame, current_frame, prev_base_frame in zip(data[:-6], data[6:], data[:6] + data[:-6]):
#         deltas = [DeltaCube(baseline, current)
#               for baseline, current in zip(baseline_frame, current_frame)]
#         print(np.mean([abs(d.position_x) for d in deltas if d.orientation_a]))
#         print('\n'.join(str(d.orientation_a) for d in deltas if d.orientation_a))
        compressed.append(compress_delta_frame(baseline_frame, current_frame, prev_base_frame))
        print('compressed length: {0}'.format(len(compressed[-1])))
    print(sum(len(c) for c in compressed))
