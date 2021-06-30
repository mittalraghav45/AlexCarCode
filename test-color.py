import RPi.GPIO as GPIO
import numpy as np
import cv2
import serial
import time


GPIO.setmode(GPIO.BCM)

serial_data=serial.Serial('/dev/ttyUSB0',9600,timeout=1)
serial_data.flush()
webcam = cv2.VideoCapture(0)
while(1):
    if serial_data.in_waiting > 0:
        line = serial_data.readline()
        if not line:
            print(" ")            
        else:  
            print(line.decode('utf-8').rstrip())
            
    _, imageFrame = webcam.read()
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)

    red_lower = np.array([136, 87, 111], np.uint8)
    red_upper = np.array([180, 255, 255], np.uint8)
    red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)

    green_lower = np.array([25, 41, 133], np.uint8)
    green_upper = np.array([40, 150, 255], np.uint8)
    green_mask = cv2.inRange(hsvFrame, green_lower, green_upper)

    yellow_lower = np.array([22, 93, 0], np.uint8)
    yellow_upper = np.array([45, 255, 255], np.uint8)
    yellow_mask = cv2.inRange(hsvFrame, yellow_lower, yellow_upper)

    kernal = np.ones((5, 5), "uint8")

    # For red color
    red_mask = cv2.dilate(red_mask, kernal)
    res_red = cv2.bitwise_and(imageFrame, imageFrame,
                              mask=red_mask)

    # For green color
    green_mask = cv2.dilate(green_mask, kernal)
    res_green = cv2.bitwise_and(imageFrame, imageFrame,
                                mask=green_mask)

    # For Yellow Color
    yellow_mask = cv2.dilate(yellow_mask, kernal)
    res_yellow = cv2.bitwise_and(imageFrame, imageFrame,
                                mask=yellow_mask)


    # Creating contour to track red color
    contours, hierarchy = cv2.findContours(red_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)

    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if(area > 10000):
            serial_data.write(b"stop\n")
            print("Red encountered, Stop!")
            time.sleep(0.1)
            x, y, w, h = cv2.boundingRect(contour)
            imageFrame = cv2.rectangle(imageFrame, (x, y),
                                       (x + w, y + h),
                                       (0, 0, 255), 2)

            cv2.putText(imageFrame, "Red Colour", (x, y),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                                    (0, 0, 255))

    contours, hierarchy = cv2.findContours(green_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)

    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if(area > 10000):
            serial_data.write(b"forward\n")
            print("Green encountered, moving forward")
            time.sleep(0.1)
            x, y, w, h = cv2.boundingRect(contour)
            imageFrame = cv2.rectangle(imageFrame, (x, y),
                                       (x + w, y + h),
                                       (0, 255, 0), 2)

            cv2.putText(imageFrame, "Green Colour", (x, y),
                                    cv2.FONT_HERSHEY_SIMPLEX,
                                    1.0, (0, 255, 0))




    contours, hierarchy = cv2.findContours(yellow_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)

    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if(area > 10000):
            serial_data.write(b"backward\n")
            print("Yellow encountered, moving backward")
            time.sleep(0.1)
            x, y, w, h = cv2.boundingRect(contour)
            imageFrame = cv2.rectangle(imageFrame, (x, y),
                                       (x + w, y + h),
                                       (0, 255, 0), 2)

            cv2.putText(imageFrame, "Yellow Colour", (x, y),
                                    cv2.FONT_HERSHEY_SIMPLEX,
                                    1.0, (0, 255, 0))
            
      
    cv2.imshow("Multiple Color Detection in Real-TIme", imageFrame)
    if cv2.waitKey(10) & 0xFF == ord('q'):
        cap.release()
        cv2.destroyAllWindows()
        break
