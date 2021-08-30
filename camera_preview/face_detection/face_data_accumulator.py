import cv2 #For Image processing 
import numpy as np #For converting Images to Numerical array 
import os #To handle directories 
import uuid
from PIL import Image #Pillow lib for handling images 

person_data_name = input("Enter the person's name: ")
face_cascade = cv2.CascadeClassifier('face_detection/haarcascade_frontalface_default.xml')
recognizer = cv2.face.LBPHFaceRecognizer_create()
recognizer.read("face_detection/face-trainer.yml")

cap = cv2.VideoCapture(0) #Get video feed from the Camera
while(True):

	ret, img = cap.read() # Break video into frames
	gray  = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) #convert Video frame to Greyscale
	faces = face_cascade.detectMultiScale(gray, scaleFactor=1.5, minNeighbors=5) #Recog. faces

	for (x, y, w, h) in faces:
		image = Image.frombuffer("RGB", (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))), img)
		b, g, r = image.split()
		colorspace_corrected_image = Image.merge("RGB", (r, g, b))
		colorspace_corrected_image = colorspace_corrected_image.transpose(Image.FLIP_TOP_BOTTOM)
		colorspace_corrected_image.save("faces/" + person_data_name + "/" + str(uuid.uuid4()) + ".jpg")
		cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)

	cv2.imshow('Preview',img) #Display the Video

	if cv2.waitKey(20) & 0xFF == ord('q'):
		break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
