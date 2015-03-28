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
        b.append((0, prev_bounds[1] + extension))
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
    pos_sizes = {}
    ortn_sizes = {}
    overheads = {}
    # one config
    for s in range(3, 10):
        pos_sizes[[s]] = []
        ortn_sizes[[s]] = []
        overhead = len(bs.Bits(ue=1))
        overhead += len(bs.Bits(ue=s))
        overheads[[s]] = overhead
        bounds = compute_bounds_signed([s])
        for _, d in deltas:
            size = 0
            for i, field in enumerate(d[1:4], start=1):
                # one for the bit telling which setting
                size += 1
                if field >= bounds[0] and field <= bounds[1]:
                    size += s
                else:
                    size += max_lens[i]
            ortn_sizes[[s]].append(size)
            size = 0
            for i, field in enumerate(d[4:7], start=4):
                # one for the bit telling which setting
                size += 1
                if field >= bounds[0] and field <= bounds[1]:
                    size += s
                else:
                    size += max_lens[i]
            pos_sizes[[s]].append(size)

def config_permutations(num, low, high, step):
    current = [low + (step * i) for i in range(num)]
    stop = [high - (step * i) for i in range(num - 1, -1, -1)]
    if current[-1] > high:
        raise ValueError('num of settigns doesn\'t fit into the range')
    yield current[:]
    while(current != stop):
        for i in range(len(current) - 1):
            if current[i + 1] - current[i] > step:
                current[i] += 1
                yield current[:]
                break
        else:
            current[-1] += 1
            yield current[:]

def compress_delta_frame(baseline_frame, current_frame):
    res = bs.BitStream()
    deltas = [DeltaCube(baseline, current)
              for baseline, current in zip(baseline_frame, current_frame)]
    changed_deltas = [(i, d) for (i, d) in enumerate(deltas) if any(d)]
    configs = select_configs(changed_deltas)
    num_configs = 1
    configs = Configs([5, 9], [5, 8])
    index_delta_config = [3, 5]
    # write num of configs (zero unnecessary, always at least 1)
    res += bs.Bits(ue=num_configs - 1)
    # TODO write configs (num of settings and settings, for pos and ortn, last setting implicit (num of bits for the whole thing))
    # TODO make the following a list
    # for every config, the number of cubes encoded with it in the index list
    # can be 0 for one, then a changed bit setup is used, otherwise unchanged
    # cubes can simply be omitted
    num_in_index_list = len(changed_deltas)
    # TODO this is only known after we decide, see below
    res += bs.Bits(ue=num_in_index_list)
    # decide whether it's better to use indices or change flags
    index_deltas = [encode_varint(changed_deltas[0][0], index_delta_config, 10,
                         signed=False)]
    for b, c in zip(changed_deltas[:-1], changed_deltas[1:]):
        index_deltas.append(encode_varint(c[0] - b[0], index_delta_config, 10,
                                          signed=False))
    if sum(len(x) for x in index_deltas) < cubes_per_frame:
        # use indices
        for i, d in enumerate(changed_deltas):
            res += index_deltas[i]
            res += compress_delta_cube(c[1], configs)
    else:
        # use change flags
        it = iter(changed_deltas)
        d = next(it)
        for i in range(cubes_per_frame):
            if i == d[0]:
                # write the compressed cube
                res += bs.Bits(bool=True)
                res += compress_delta_cube(d[1], configs)
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
    g = config_permutations(3, 3, 10, 3)
    l = []
    for x in g:
        l.append(x)
    print('\n'.join(str(x) for x in l))
#     data = Data(filename)
#     compressed = []
#     for baseline_frame, current_frame in zip(data[2000:2500], data[2006:2506]):
#         compressed.append(compress_delta_frame(baseline_frame, current_frame))
#     print(sum(len(c) for c in compressed))
