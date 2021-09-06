#Program to Detect the Face and Recognise the Person based on the data from face-trainer.yml

import cv2 #For Image processing
import time
from servo_controller import ServoControl
import functools
from threading import Thread
from asyncio import Event

class Program:
	labels = ["Simeon"] 

	def __init__(self):
		self.face_cascade = cv2.CascadeClassifier('face_detection/haarcascade_frontalface_default.xml')
		self.servos = ServoControl()
		self.recognizer = cv2.face.LBPHFaceRecognizer_create()
		self.recognizer.read("face_detection/face-trainer.yml")
		self.deadzone_threshold_percentage = .1
		self.velocity_damping_threshold = .3
		self.rotation_velocity = 8 # Degrees per second
		self.prev_time = 0
		self.target_vector = (0, 0)
		self.scan_cancellation_event = Event()
		self.scan_started_event = Event()
		self.timeout_length_until_scan = 5.0
		self.scan_rotation_velocity = 10.0

		self.cap = cv2.VideoCapture(0) #Get video feed from the Camera
		self.frame_centerpoint = (int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)) / 2, int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) / 2)
		self.position_centerpoint = self.frame_centerpoint

	def look_at(self, vector, delta):
		dampened_velocity = (self.rotation_velocity if abs(vector[0]) / self.frame_centerpoint[0] > self.velocity_damping_threshold else abs(vector[0]) * self.rotation_velocity / (self.frame_centerpoint[0] * self.velocity_damping_threshold), \
			self.rotation_velocity if abs(vector[1]) / self.frame_centerpoint[1] > self.velocity_damping_threshold else abs(vector[1]) * self.rotation_velocity / (self.frame_centerpoint[1] * self.velocity_damping_threshold))

		if vector[0] < 0:
			self.servos.moveYawDegrees(dampened_velocity[0] * delta)
		else:
			self.servos.moveYawDegrees(-dampened_velocity[0] * delta)

		if vector[1] < 0:
			self.servos.movePitchDegrees(dampened_velocity[1] * delta)
		else:
			self.servos.movePitchDegrees(-dampened_velocity[1] * delta)

	def main(self):
		scan_thread: Thread = Thread(target=self.servos.scan, args=(self.scan_rotation_velocity, self.scan_started_event, self.scan_cancellation_event))
		last_found_face_time: float = 0 

		def start_scan():
			if self.scan_started_event.is_set():
				return

			self.scan_started_event = Event()
			self.scan_cancellation_event = Event()

			scan_thread = Thread(target=self.servos.scan, args=(self.scan_rotation_velocity, self.scan_started_event, self.scan_cancellation_event))
			scan_thread.start()
			self.scan_started_event.wait()
		
		start_scan()

		while(True):
			ret, img = self.cap.read() # Break video into frames 
			gray  = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) #convert Video frame to Greyscale
			faces = self.face_cascade.detectMultiScale(gray, scaleFactor=1.5, minNeighbors=5) #Recog. faces

			if len(faces) > 0:
				last_found_face_time = time.time()
				self.scan_cancellation_event.set()

				if scan_thread.is_alive():
					scan_thread.join()
					
				face_positions = map(lambda x : (x[0] + x[2] / 2, x[1] + x[3] / 2), faces)
				position_centerpoint = functools.reduce(lambda x, y : ((x[0] + y[0]) / 2, (x[1] + y[1]) / 2), face_positions)
				target_vector = (position_centerpoint[0] - self.frame_centerpoint[0], position_centerpoint[1] - self.frame_centerpoint[1])
			else:
				if not self.scan_started_event.is_set() and time.time() - last_found_face_time > self.timeout_length_until_scan:
					start_scan()

			if len(faces) > 0 and (abs(target_vector[0]) > self.deadzone_threshold_percentage * self.frame_centerpoint[0] or abs(target_vector[1]) > self.deadzone_threshold_percentage * self.frame_centerpoint[1] or target_vector == (0, 0)):
				self.look_at(target_vector, time.time() - prev_time)

			prev_time = time.time()

			for (x, y, w, h) in faces:
				roi_gray = gray[y:y+h, x:x+w] #Convert Face to greys

				id_, conf = self.recognizer.predict(roi_gray) #recognize the Face
			
				#if conf >= 80:
				#	font = cv2.FONT_HERSHEY_SIMPLEX #Font style for the name 
				#	name = labels[id_] #Get the name from the List using ID number 
				#	cv2.putText(img, name, (x,y), font, 1, (0,0,255), 2)
				
				cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)

			cv2.imshow('Preview',img) #Display the Video

			if cv2.waitKey(20) & 0xFF == ord('q'):
				break

		# When everything done, release the capture
		self.cap.release()
		self.servos.destroy()
		cv2.destroyAllWindows()

if __name__ == "__main__":
	Program.main(Program())