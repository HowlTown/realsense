import numpy as np
import os
from show import show_pointcloud

date = "2024-12-27"
time = "20-04"
folders = ["depth","rgb"]

images = {}
for folder in folders:
    images[folder] = []
    folder_path = os.path.join("data",date,time,folder)
    for i in range(10):
        image_path = os.path.join(folder_path,"%d.npy" % i)
        image = np.load(image_path)
        images[folder].append(image)

