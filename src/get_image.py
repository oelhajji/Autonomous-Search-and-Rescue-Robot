#!/usr/bin/env python
import face_recognition
from gtts import gTTS
import os
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


known_face_encodings = []
known_face_names = []


        
"""
Load encoding images from path
param images_path:
:return:
"""
# Load Images



current_directory = os.path.dirname(os.path.abspath(__file__))


images_path = [
    os.path.join(current_directory, "Ewan.jpg"),
    os.path.join(current_directory, "Omar.jpg")
]


print("{} encoding images found.".format(len(images_path)))

# Store image encoding and names
for img_path in images_path:
    img = cv2.imread(img_path)
    rgb_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    # Get the filename only from the initial file path.
    basename = os.path.basename(img_path)
    (filename, ext) = os.path.splitext(basename)
    # Get encoding
    img_encoding = face_recognition.face_encodings(rgb_img)[0]

    # Store file name and file encoding
    known_face_encodings.append(img_encoding)
    known_face_names.append(filename)
print("Encoding images loaded")

def detect_known_faces(frame):
        frame_resizing = 0.25
        small_frame = cv2.resize(frame, (0, 0), fx=frame_resizing, fy=frame_resizing)
        # Find all the faces and face encodings in the current frame of video
        # Convert the image from BGR color (which OpenCV uses) to RGB color (which face_recognition uses)
        rgb_small_frame = cv2.cvtColor(small_frame, cv2.COLOR_BGR2RGB)
        face_locations = face_recognition.face_locations(rgb_small_frame)
        face_encodings = face_recognition.face_encodings(rgb_small_frame, face_locations)

        face_names = []
        for face_encoding in face_encodings:
            # See if the face is a match for the known face(s)
            matches = face_recognition.compare_faces(known_face_encodings, face_encoding)
            name = "Unknown"


            # Or instead, use the known face with the smallest distance to the new face
            face_distances = face_recognition.face_distance(known_face_encodings, face_encoding)
            best_match_index = np.argmin(face_distances)
            if matches[best_match_index]:
                name = known_face_names[best_match_index]
            face_names.append(name)

        # Convert to numpy array to adjust coordinates with frame resizing quickly
        face_locations = np.array(face_locations)
        face_locations = face_locations /frame_resizing
        return face_locations.astype(int), face_names

ewanfound=False
omarfound=False

def callback(data):
    global ewanfound 
    global omarfound
    bridge = CvBridge()
    frame = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")

    cv2.namedWindow("face_detection", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("face_detection", frame.shape[1]//3, frame.shape[0]//3)

    # Detect Faces
    face_locations, face_names = detect_known_faces(frame)

    for face_loc, name in zip(face_locations, face_names):
        y1, x2, y2, x1 = face_loc[0], face_loc[1], face_loc[2], face_loc[3]
        
        cv2.putText(frame, name, (x1, y1 - 10), cv2.FONT_HERSHEY_DUPLEX, 1, (0, 0, 200), 2)
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 200), 4)
        if not ewanfound and name=="Ewan":
            word ="Ewan is here"  # Change this to the word you want to say

            # Initialize the gTTS object with the word and language (e.g., 'en' for English)
            tts = gTTS(text=word, lang='en')

            # Save the generated speech as an audio file (e.g., hello.mp3)
            tts.save("name.mp3")

            # Use the operating system to play the audio file
            os.system("mpg123 name.mp3")  # You may need to install 'mpg123' for this to work

            # Clean up the temporary audio file
            os.remove("name.mp3")

            ewanfound=True
        if not omarfound and name=="Omar":
            word ="Omar is here"  # Change this to the word you want to say

            # Initialize the gTTS object with the word and language (e.g., 'en' for English)
            tts = gTTS(text=word, lang='en')

            # Save the generated speech as an audio file (e.g., hello.mp3)
            tts.save("name.mp3")

            # Use the operating system to play the audio file
            os.system("mpg123 name.mp3")  # You may need to install 'mpg123' for this to work

            # Clean up the temporary audio file
            os.remove("name.mp3")

            omarfound=True


    # Show the image after processing all faces
    cv2.imshow("face_detection", frame)
    cv2.waitKey(1)  



def listener():    
    rospy.init_node('image_subscriber', anonymous=True)
    rospy.Subscriber('/my_first_robot/camera/rgb/image_raw', Image, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
