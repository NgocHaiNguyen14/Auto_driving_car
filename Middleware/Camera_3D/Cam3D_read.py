import ast
import numpy as np
import matplotlib.pyplot as plt

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

# Convert the data list to 16-bit values considering little-endian format
def convert_to_16bit(data):
    pixel_values = []
    for i in range(0, len(data), 2):
        lower_byte = data[i]
        upper_byte = data[i + 1]
        # Combine bytes (little-endian)
        pixel_value = lower_byte + (upper_byte << 8)
        pixel_values.append(pixel_value)
    return pixel_values

# Get metadata and data list
metadata, data_list = extract_metadata_and_data(file_path)
# Convert data list
pixel_values = convert_to_16bit(data_list)

# # Debugging prints
# print(f"Metadata: {metadata}")
# print(f"Data list length: {len(data_list)}")
# print(f"Expected size: {metadata['height'] * metadata['width']}")

# # Check if the data list size matches the expected size
# expected_size = metadata['height'] * metadata['width']
# if len(data_list) != expected_size:
#     print("Warning: The size of the data list does not match the expected dimensions from the metadata.")
#     # Trim the data list to the expected size
#     data_list = data_list[:expected_size]
#     print(f"Trimmed data list length: {len(data_list)}")

# Convert the data list to a NumPy array
try:
    image_data = np.array(pixel_values, dtype=np.uint16).reshape(metadata['height'], metadata['width'])
    # Count the number of elements in the data list
except ValueError as e:
    print(f"Error reshaping data: {e}")
    exit(1)

# Display the image using matplotlib
plt.imshow(image_data, cmap='gray')
plt.title('Depth Image')
plt.axis('off')
plt.show()


