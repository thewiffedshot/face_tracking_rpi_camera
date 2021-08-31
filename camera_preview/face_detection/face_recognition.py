#Program to Detect the Face and Recognise the Person based on the data from face-trainner.yml

import cv2 #For Image processing 
import numpy as np #For converting Images to Numerical array 
import os #To handle directories 
import time
from PIL import Image #Pillow lib for handling images 
from servo_controller import ServoControl
import functools

labels = ["Simeon"] 

face_cascade = cv2.CascadeClassifier('face_detection/haarcascade_frontalface_default.xml')
servos = ServoControl()
recognizer = cv2.face.LBPHFaceRecognizer_create()
recognizer.read("face_detection/face-trainer.yml")
deadzone_threshold_percentage = .1
rotation_velocity = 6 # Degrees per second
prev_time = 0
target_vector = (0, 0)

cap = cv2.VideoCapture(0) #Get video feed from the Camera
frame_centerpoint = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) / 2, int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) / 2)
position_centerpoint = frame_centerpoint

def look_at(vector, delta):
	if vector[0] < 0:
		servos.moveYawDegrees(rotation_velocity * delta)
	else:
		servos.moveYawDegrees(-rotation_velocity * delta)

	if vector[1] < 0:
		servos.movePitchDegrees(rotation_velocity * delta)
	else:
		servos.movePitchDegrees(-rotation_velocity * delta)

while(True):

	ret, img = cap.read() # Break video into frames 
	gray  = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) #convert Video frame to Greyscale
	faces = face_cascade.detectMultiScale(gray, scaleFactor=1.5, minNeighbors=5) #Recog. faces

	if len(faces) > 0:
		face_positions = map(lambda x : (x[0] + x[2] / 2, x[1] + x[3] / 2), faces)
		position_centerpoint = functools.reduce(lambda x, y : ((x[0] + y[0]) / 2, (x[1] + y[1]) / 2), face_positions)
		target_vector = (position_centerpoint[0] - frame_centerpoint[0], position_centerpoint[1] - frame_centerpoint[1])

	if len(faces) > 0 and (abs(target_vector[0]) > deadzone_threshold_percentage * frame_centerpoint[0] or abs(target_vector[1]) > deadzone_threshold_percentage * frame_centerpoint[1] or target_vector == (0, 0)):
		print(target_vector)
		look_at(target_vector, time.time() - prev_time)

	prev_time = time.time()

	for (x, y, w, h) in faces:
		roi_gray = gray[y:y+h, x:x+w] #Convert Face to greys

		id_, conf = recognizer.predict(roi_gray) #recognize the Face
	
		if conf >= 60:
			font = cv2.FONT_HERSHEY_SIMPLEX #Font style for the name 
			name = labels[id_] #Get the name from the List using ID number 
			cv2.putText(img, name, (x,y), font, 1, (0,0,255), 2)
		
		cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)

	cv2.imshow('Preview',img) #Display the Video
	if cv2.waitKey(20) & 0xFF == ord('q'):
		break

# When everything done, release the capture
cap.release()
servos.destroy()
cv2.destroyAllWindows()
