#Program to Detect the Face and Recognise the Person based on the data from face-trainner.yml

import cv2 #For Image processing 
import numpy as np #For converting Images to Numerical array 
import os #To handle directories 
import time
from PIL import Image #Pillow lib for handling images 
from servo_controller import ServoControl
import functools
import asyncio
from threading import Thread, Timer
from asyncio import Event

labels = ["Simeon"] 

face_cascade = cv2.CascadeClassifier('face_detection/haarcascade_frontalface_default.xml')
servos = ServoControl()
recognizer = cv2.face.LBPHFaceRecognizer_create()
recognizer.read("face_detection/face-trainer.yml")
deadzone_threshold_percentage = .1
velocity_damping_threshold = .3
rotation_velocity = 8 # Degrees per second
prev_time = 0
target_vector = (0, 0)
scan_cancellation_event = Event()
timeout_length_until_scan = 5.0
scan_rotation_velocity = 10

cap = cv2.VideoCapture(0) #Get video feed from the Camera
frame_centerpoint = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) / 2, int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) / 2)
position_centerpoint = frame_centerpoint

def look_at(vector, delta):
	dampened_velocity = (rotation_velocity if abs(vector[0]) / frame_centerpoint[0] > velocity_damping_threshold else abs(vector[0]) * rotation_velocity / (frame_centerpoint[0] * velocity_damping_threshold), \
		rotation_velocity if abs(vector[1]) / frame_centerpoint[1] > velocity_damping_threshold else abs(vector[1]) * rotation_velocity / (frame_centerpoint[1] * velocity_damping_threshold))

	if vector[0] < 0:
		servos.moveYawDegrees(dampened_velocity[0] * delta)
	else:
		servos.moveYawDegrees(-dampened_velocity[0] * delta)

	if vector[1] < 0:
		servos.movePitchDegrees(dampened_velocity[1] * delta)
	else:
		servos.movePitchDegrees(-dampened_velocity[1] * delta)

def main():
	scan_thread: Thread = Thread(target=servos.scan, args=(scan_rotation_velocity, scan_cancellation_event))
	last_found_face_time: float = 0 

	def start_scan():
		scan_thread = Thread(target=servos.scan, args=(scan_rotation_velocity, scan_cancellation_event))
		scan_thread.start()
	
	scan_thread.start()

	while(True):
		ret, img = cap.read() # Break video into frames 
		gray  = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) #convert Video frame to Greyscale
		faces = face_cascade.detectMultiScale(gray, scaleFactor=1.5, minNeighbors=5) #Recog. faces

		if len(faces) > 0:
			last_found_face_time = time.time()
			scan_cancellation_event.set()
			scan_thread.join()
				
			face_positions = map(lambda x : (x[0] + x[2] / 2, x[1] + x[3] / 2), faces)
			position_centerpoint = functools.reduce(lambda x, y : ((x[0] + y[0]) / 2, (x[1] + y[1]) / 2), face_positions)
			target_vector = (position_centerpoint[0] - frame_centerpoint[0], position_centerpoint[1] - frame_centerpoint[1])
		else:
			if not scan_thread.is_alive() and time.time() - last_found_face_time > timeout_length_until_scan:
				start_scan()

		if len(faces) > 0 and (abs(target_vector[0]) > deadzone_threshold_percentage * frame_centerpoint[0] or abs(target_vector[1]) > deadzone_threshold_percentage * frame_centerpoint[1] or target_vector == (0, 0)):
			look_at(target_vector, time.time() - prev_time)

		prev_time = time.time()

		for (x, y, w, h) in faces:
			roi_gray = gray[y:y+h, x:x+w] #Convert Face to greys

			id_, conf = recognizer.predict(roi_gray) #recognize the Face
		
			#if conf >= 80:
			#	font = cv2.FONT_HERSHEY_SIMPLEX #Font style for the name 
			#	name = labels[id_] #Get the name from the List using ID number 
			#	cv2.putText(img, name, (x,y), font, 1, (0,0,255), 2)
			
			cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)

		cv2.imshow('Preview',img) #Display the Video
		if cv2.waitKey(20) & 0xFF == ord('q'):
			break

	# When everything done, release the capture
	cap.release()
	servos.destroy()
	cv2.destroyAllWindows()

if __name__ == "__main__":
	main()