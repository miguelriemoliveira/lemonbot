
import numpy

class Images(object):

    def __init__(self):
        self.timestamps = []
        self.images = []
        self.transforms = []
    
    def __len__(self):
        return len(self.timestamps)
    
    def append(self, timestamp, image, transform):
        self.timestamps.append(timestamp)
        self.images.append(image)
        self.transforms.append(transform)
    
    def save(self, path):
        from PIL import Image
        from os.path import join

        timestamps_path = join(path, "images.timestamps.npy")
        images_path = join(path, "images.image{:03d}.jpg")
        transforms_path = join(path, "images.transforms.npy")
        
        timestamps = numpy.array(self.timestamps, dtype=numpy.uint64)
        transforms = numpy.array(self.transforms, dtype=numpy.float32)

        numpy.save(timestamps_path, timestamps)
        numpy.save(transforms_path, transforms)

        for idx, image in enumerate(self.images):
            image = Image.fromarray(image)

            image.save(images_path.format(idx))
