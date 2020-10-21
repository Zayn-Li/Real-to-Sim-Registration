import os
import sys
import numpy as np
import imageio
import argparse
import scipy.misc
import matplotlib.pyplot
import imageio
from PIL import Image



parser = argparse.ArgumentParser()
parser.add_argument('--src_dir', default='')
parser.add_argument('--st_idx', type=int, default=0)
parser.add_argument('--ed_idx', type=int, default=99)
parser.add_argument('--height', type=int, default=926)
parser.add_argument('--width', type=int, default=1448)

args = parser.parse_args()
st_x = 0
ed_x = 87

images = []

for i in range(st_x, ed_x):

    filename = os.path.join(args.src_dir, '00_L%02d.png' % i)
    print(filename)
    #img = imageio.imread(filename)
	
    img = Image.open(filename)
    img = img.resize((args.width, args.height), Image.ANTIALIAS)
    #img = cv2.resize(img, (args.width, args.height), interpolation=cv2.INTER_AREA)

    images.append(img)

imageio.mimsave(args.src_dir + 'mesh.gif', images, duration=1.0/10.0)
