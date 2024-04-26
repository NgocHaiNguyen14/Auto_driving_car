import cv2
import numpy as np
from tensorflow.keras.models import load_model
import tensorflow as tf
import matplotlib.pyplot as plt

def resize_image(image, new_height, new_width):
    resized_image = cv2.resize(image, (new_width, new_height))
    return resized_image

model = tf.keras.models.load_model('model.h5')

cap = cv2.VideoCapture("harder_challenge_video.mp4")

while(cap.isOpened()):
    _, frame = cap.read()
    a = frame.shape[0]
    b = frame.shape[1]
    resized_image = resize_image(frame, 80, 160)
    image_tf = tf.convert_to_tensor(resized_image, dtype=tf.float32)
    prediction = model.predict(np.expand_dims(image_tf, axis=0))
    class_label = np.argmax(prediction)
    predicted_values = prediction[0]
    mask_resize = resize_image(predicted_values,a,b)
    cv2.imshow("ketqua", mask_resize)
    cv2.waitKey(1)
    """plt.imshow(predicted_values)
    plt.axis('off')  # Turn off axis
    plt.show()"""