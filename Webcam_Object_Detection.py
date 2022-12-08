import numpy as np
import cv2
import random as rng
from djitellopy import Tello
import time
from imutils import  paths
import imutils
if __name__ == '__main__':

    cap = cv2.VideoCapture(0)

    #Vi laver en detector
    detector = cv2.FastFeatureDetector_create()
    detector.setThreshold(25)
    valg = input("Valg af program \n1 - Brug af hjemmelavet\n2 - Brug af FastFeatureDetector\n3 - Brug af ANDET \n")
    print(valg)
    if valg == '2':
        detector = cv2.FastFeatureDetector_create()
        # detector.set
        detector.setThreshold(30)

    if valg == '3':
        orb = cv2.ORB_create()



    while True:
        # Får fat i en frame fra dronen
        ret, frame = cap.read(0)

        #buffer til at få hele billedet
        if not ret:
            print("bygger stream igen")
            cap.release()
            cap = cv2.VideoCapture(0)
            frame = cap.read(0)
        #Vi bruger den hjemmelavede
        if valg == '1':
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            #thresh = cv2.threshold(gray, 128, 255, cv2.THRESH_BINARY)[1]
            thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 25, 10)  # 5th parameter = pixel neighbourhood size, 6th parameter = finetuning     #note, vi kan fra tælle alle firkanter mindre end X
            result = frame.copy()

            contours = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours = contours[0] if len(contours) == 2 else contours[1]
            for cntr in contours:
                x, y, w, h = cv2.boundingRect(cntr)

                if((w)*(h)> 100):
                    #print(str((w)*(h)))
                    cv2.rectangle(result, (x, y), (x + w, y + h), (0, 0, 255), 2)
                    # Indsæt område firkanter //KUN TIL DEBUG
                    cv2.rectangle(result, (0, 0), (320, 720), (0, 255, 0), 2)
                    cv2.rectangle(result, (640, 0), (960, 720), (0, 255, 0), 2)
                    cv2.rectangle(result, (320, 0), (640, 240), (0, 255, 0), 2)
                    cv2.rectangle(result, (320, 240), (640, 480), (0, 255, 0), 2)
                    cv2.rectangle(result, (320, 480), (640, 720), (0, 255, 0), 2)
        if valg == '2':
            #vi finder keypoints
            kp = detector.detect(frame,None) #der findes flere typer som Harris, DENSE og GFTT
            #cv2.imshow('Keypoints',kp)



            #Sammenlign keypoints

            result = cv2.drawKeypoints(frame,kp,None, color=(255,0,0))

        if valg == '3':
            kp = orb.detect(frame,None)
            kp, des = orb.compute(frame, kp)
            result = cv2.drawKeypoints(frame, kp, None, color=(0,255,0), flags=0)

        #Hvis billedet på skærmen
        cv2.imshow('frame', result)
        #quit med Q
        if cv2.waitKey(1) == ord('q'):
            #drone.streamoff()
            break


    cv2.destroyAllWindows()

