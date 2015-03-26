import bitstring as bs

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
    framesize = 901 * 32 * 8
    def __init__(self, filename):
        self.bits = bs.Bits(filename=filename)
        self.num_frames = len(self.bits) // (self.framesize)
        self.frames = [None for _ in range(self.num_frames)]

    def __getitem__(self, i):
        if not self.frames[i]:
            self.frames[i] = Frame(self.bits[i * self.framesize:
                                             (i + 1) * self.framesize])
        return self.frames[i]

    def __iter__(self):
        return Iter(self, self.framesize)

class Frame:
    cubesize = 32 * 8
    num_cubes = 901
    def __init__(self, bits):
        self.bits = bits
        self.cubes = [None for _ in range(self.num_cubes)]

    def __getitem__(self, i):
        if not self.cubes[i]:
            self.cubes[i] = Cube(self.bits[i * self.cubesize:
                                           (i + 1) * self.cubesize])
        return self.cubes[i]

    def __iter__(self):
        return Iter(self, self.num_cubes)

class Cube:
    def __init__(self, bits):
        self.orientation_largest = bits[:32]
        self.orientation_a = bits[32:64]
        self.orientation_b = bits[64:96]
        self.orientation_c = bits[96:128]
        self.position_x = bits[128:160]
        self.position_y = bits[160:192]
        self.position_z = bits[192:224]
        self.interacting = bits[224:256]

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
#     print(max(frame.get(i).position_y.uintle for i in range(901)))
#     print(any(frame.get(i).position_y for i in range(901)))
    for frame in data:
        for cube in frame:
            x = cube.orientation_a.uintle
            if x:
                print(x)
#     print(frame.bits.bin)
#     print(len(frame.bits))
#     print(len(rl_enc(frame.bits)))
#     for frame in (data.get(i) for i in range(2000, 2001)):
#         for cube in (frame.get(j) for j in range(len(frame.cubes))):
#             if any(cube.interacting) and cube.interacting != bs.Bits(bin='00000001000000000000000000000000'):
#                 print (cube.interacting.bin)
