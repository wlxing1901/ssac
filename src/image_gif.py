import cv2
import os
from PIL import Image

# Set the path and parameters
path = '/home/wlxing/Codes/ssac/output/test'
output_mp4_path = '/home/wlxing/Codes/ssac/output/output.mp4'
fps = 60  # Frames per second

# Get all .png files and sort them by their filenames
files = [f for f in os.listdir(path) if f.endswith('.png')]
files.sort(key=lambda x: int(x.split('_')[-1].split('.png')[0]))

# Initialize variables
frames = []
last_image = None

# Get the dimensions of the first image
first_image = Image.open(os.path.join(path, files[0]))
width, height = first_image.size

# Initialize the video writer
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter(output_mp4_path, fourcc, fps, (width, height))

# Loop from 1, according to natural numbers
for i in range(1, max([int(f.split('_')[-1].split('.png')[0]) for f in files]) + 1):
    filename = f"left_right_{i}.png"
    filepath = os.path.join(path, filename)

    # If the image corresponding to the frame does not exist, use the previous image
    if not os.path.exists(filepath):
        if last_image is not None:
            out.write(last_image)
        continue

    # Read the image and save
    img = cv2.imread(filepath)
    out.write(img)
    last_image = img

# Release the video writer
out.release()

print(f"Generated MP4 saved at {output_mp4_path}")
