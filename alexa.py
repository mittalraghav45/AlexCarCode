import requests
import json
import serial
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera

webcam = cv2.VideoCapture(0)
camera=PiCamera()
camera.resolution= (1280,720)
camera.framerate=32
rawCapture = PiRGBArray(camera,size =(1280,720))

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    webcam = frame.array
    serial_data=serial.Serial('/dev/ttyUSB0',9600)
    serial_data.flush()
    response = json.loads(requests.get("https://api.thingspeak.com/channels/1274437/fields/1.json?api_key=ZNB2A3Y0CF4AFTUP&results=1").text)
    command =int(response['feeds'][0]['field1'])
    
    if(command==1):
        print("Forward")
        cv2.imshow("Carfeed",webcam)
        serial_data.write(b"forward\n")        
    elif(command==-1):
        cv2.imshow("Carfeed",webcam)
        print("Backward")
        serial_data.write(b"backward\n")        
    else:
        print("Stop")
        serial_data.write(b"stop\n")
    rawCapture.truncate(0)
    if cv2.waitKey(10) & 0xFF == ord('q'):
        cap.release()
        cv2.destroyAllWindows()
        break     
