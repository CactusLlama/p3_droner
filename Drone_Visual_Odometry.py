import queue

import numpy as np
import cv2
import random as rng
from djitellopy import Tello
import time
from imutils import paths
import imutils

q = queue.Queue()
NUM_OF_FRAMES = 24
FRAME_CENT_X = 480
AVOID_FLAG = 0


class Rect:
    def __init__(self, x, y, w, h):
        self.x = x
        self.y = y
        self.w = w
        self.h = h

    def __str__(self):
        return f"Cord:({self.x},{self.y}) Height: {self.h}  Width: {self.w}"
    def getx(self):
        return self.x
    def gety(self):
        return self.y
    def geth(self):
        return self.h
    def getw(self):
        return self.w



def findcontours(thres, fram, rectList):
    contour = cv2.findContours(thres, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    global AVOID_FLAG
    contour = contour[0] if len(contour) == 2 else contour[1]
    for cntr in contour:
        x, y, w, h = cv2.boundingRect(cntr)

        if ((w) * (h) > 100) and h > 50 and w > 50:
            # print(str((w)*(h)))
            cv2.rectangle(fram, (x, y), (x + w, y + h), (0, 0, 255), 2)  # selve firkanten sættes i en liste

            # indsæt område til at finde ud af om firkanten er i ROI (aka midterområdet)

            if x <= 580 <= x + w and y <= 360 <= y + h and h * w >= 160:
                # vi har en firkant som er større end midterfirkanten, så vi slår recthandler til
                AVOID_FLAG = 1
                #print("AVOID_FLAG == 1")
                # vi tilføjer kun firkant til listen hvis denne har en chance for at være stor nok til at være i vejen - Dette gør at vi undgår unødvendige firkanter i listen

                rectList.append(Rect(x, y, w, h))

            # Indsæt område firkanter //KUN TIL DEBUG
            cv2.rectangle(fram, (0, 0), (320, 720), (0, 255, 0), 2)
            cv2.rectangle(fram, (640, 0), (960, 720), (0, 255, 0), 2)
            cv2.rectangle(fram, (320, 0), (640, 240), (0, 255, 0), 2)
            cv2.rectangle(fram, (320, 240), (640, 480), (0, 255, 0), 2)
            cv2.rectangle(fram, (320, 480), (640, 720), (0, 255, 0), 2)
    return fram, rectList


def rectHandler():
    # TODO Lav denne håndtering af de tre frames trekant og lav et forslag til hvordan den skal dreje.

    # Hvordan gør sammenligner vi trekanterne?
    # lav kun denne hvis der er noget i vejen
    left = 0;
    right = 0;
    for i in range(currentframe - 1):


       # print(temp[0].w)
       # if len(temp) > 0: temp.pop(0)
       # print(type(temp))
        
        for j in range(len(rectHandlerList[i])):
            # find centrum af firkanten
            # brug x til at finde ud af hvor meget der skal drejes
            centX = rectHandlerList[i][j].x + (rectHandlerList[i][j].w / 2)
            centY = rectHandlerList[i][j].y + (rectHandlerList[i][j].h / 2)
            # TODO hvad hvis der er flere frames med forskellige origo?
            if centX > FRAME_CENT_X:
                # drej til venstre for at undgå
                left = +1
            else:
                # drej til højre for at undgå
                right = +1

    if (left > right):
        print("Drej til venstre")
    else:
        print("Drej til Højre")

    return


if __name__ == '__main__':

    # Størrelse på drone billedet er 720x960

    # Vi kan lave områder på left fra (0,0) -> (720,320)   right fra (0,640) -> (720,960)   top fra (0,320)-> (240,640)    mid fra (240, 320) -> (480,640)   bot fra (480,320) -> (720,640)

    #Følgene linje kan
    drone = Tello(); drone.connect(); drone.streamon(); cap = drone.get_video_capture()

    #cap = cv2.VideoCapture(0)


    rng.seed(1234)
    currentframe = 1
    rectHandlerList = []
    rectHandlerList.clear()
    # valg = input("Valg af program \n1 - Brug af hjemmelavet\n2 - Brug af FastFeatureDetector\n3 - Brug af ANDET ")
    # print(valg)
    valg = '1'



    # Vi laver en detector

    # drone.set_video_bitrate(drone.BITRATE_5MBPS)

    # if not cam.isOpened():
    #   print("cam not open")
    #  exit()
    fullrectList = []  # list til at håndtere hver frames liste. kan muligvis gøres smartere
    while True:
        # Får fat i en frame fra dronen, enten via drone eller webcam
        ret, frame = cap.read(0)

        # vi laver en liste med rektangler, dette skal gøres hver frame
        rectList = []

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 23,
                                           10)  # 5th parameter = pixel neighbourhood size, 6th parameter = finetuning     #note, vi kan fra tælle alle firkanter mindre end X
            # thresh3 = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 23,
            #                               10)  # 5th parameter = pixel neighbourhood size, 6th parameter = finetuning     #note, vi kan fra tælle alle firkanter mindre end X
        frame1, rectList = findcontours(thresh, frame, rectList)
        cv2.imshow("adaptive - MEAN", frame1)  # MEAN fungerer umiddelbart bedst

        if (rectList != 0):
            rectHandlerList.append(rectList) #burde tilføje listen til den korrekte

        if currentframe == NUM_OF_FRAMES:
            # gennemgå listen med recthandler

            rectHandler()

            rectHandlerList.clear()

            currentframe = 0
            AVOID_FLAG = 0

        if cv2.waitKey(1) == ord('q'):
            drone.streamoff()
            break
        if AVOID_FLAG == 1:
            currentframe += 1

    cv2.destroyAllWindows()
