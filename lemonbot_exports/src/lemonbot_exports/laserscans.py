import numpy
import yaml

class Laserscans(object):

    def __init__(self):

        self.timestamps = []
        self.ranges = []
        self.transforms = []
        self.parameters = {}
    
    def __len__(self):
        return len(self.timestamps)
    
    def append(self, laserscan, transform):
        self.timestamps.append(laserscan.header.stamp.to_nsec())
        self.ranges.append(laserscan.ranges)
        self.parameters["angles"] = [laserscan.angle_min, laserscan.angle_increment, laserscan.angle_max]
        self.parameters["ranges"] = [laserscan.range_min, laserscan.range_max]
        self.transforms.append(transform)

    def save(self, path):
        from os.path import join

        timestamps_path = join(path, "laserscans.timestamps.npy")
        ranges_path = join(path, "laserscans.ranges.npy")
        transforms_path = join(path, "laserscans.transforms.npy")

        timestamps = numpy.array(self.timestamps, dtype=numpy.uint64)
        ranges = numpy.array(self.ranges, dtype=numpy.float32)
        transforms = numpy.array(self.transforms, dtype=numpy.float32)

        numpy.save(timestamps_path, timestamps)
        numpy.save(ranges_path, ranges)
        numpy.save(transforms_path, transforms)
