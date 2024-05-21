import ast
import numpy as np
import matplotlib.pyplot as plt
import cv2

# Path to your text file
file_path = 'D:/Desktop/cam3d_data.txt'

# Function to extract metadata and the data list
def extract_metadata_and_data(file_path):
    metadata = {}
    data_list = []

    with open(file_path, 'r') as file:
        for line in file:
            line = line.strip()
            if line.startswith("height:"):
                metadata['height'] = int(line.split(": ")[1])
            elif line.startswith("width:"):
                metadata['width'] = int(line.split(": ")[1])
            elif line.startswith("encoding:"):
                metadata['encoding'] = line.split(": ")[1].strip('"')
            elif line.startswith("is_bigendian:"):
                metadata['is_bigendian'] = int(line.split(": ")[1])
            elif line.startswith("step:"):
                metadata['step'] = int(line.split(": ")[1])
            elif line.startswith("data:"):
                # Extract the list part from the line and convert to actual list
                data_str = line.split("data: ", 1)[1]
                data_list = ast.literal_eval(data_str)
    
    return metadata, data_list

# Get metadata and data list
metadata, data_list = extract_metadata_and_data(file_path)

# Convert the data list to a NumPy array
image_data = np.array(data_list, dtype=np.uint16).reshape(metadata['height']*2, metadata['width'])

# Display the image using matplotlib
plt.imshow(image_data, cmap='Greens')
plt.title('Depth Image')
plt.axis('off')
plt.show()

