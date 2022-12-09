import math
import queue

from djitellopy import tello
import numpy as np
import cv2
from time import sleep

import threading as th #for doing driving and vision at the same time




############### Parameters for Threading ##############
q = queue.Queue()


################ Parameters ##################
forwardS = 200/10  # Forward Speed in cm/s (at the speed 15cm/s = 11.7cm/10s) TEST THIS!
rotationS = 360/10  # Rotation speed in degrees/second (10s to rotate 360 degrees) (at 50d/s)
interval = 0.25

distInterval = forwardS*interval  # 20*0.25= 5
rotationInterval = rotationS*interval  # 36 * 0.25 = 9 (this is only used for keyboard input)

########### VISION Parameters #################
NUM_OF_FRAMES = 24
FRAME_CENT_X = 480
AVOID_FLAG = 0
rectHandlerList = []
currentframe = 0
##############################################

################ Functions From Vision ############
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
                # print("AVOID_FLAG == 1")
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

            if centX > FRAME_CENT_X:
                # drej til venstre for at undgå
                left = +1
            else:
                # drej til højre for at undgå
                right = +1


    if (left > right):

        #q bliver brugt som kommunikation mellem threads
        q.put("TURN")

        #LOCK indtil at der er drejet fra mapping metoden
        print("Drej til venstre")
    else:
        print("Drej til Højre")

    return




if __name__ == "__main__":
    ############## Start before parallelism ############
    x = 500
    y = 500
    a = 0
    yaw = 0
    dist = [int, int]

    print("Input the point where the drone has to go (in cm):\n")
    dist[0] = int(input("Input x: "))
    dist[1] = int(input("Input y: "))

    print(dist)

    total_distance = dist[0] * dist[0] + dist[1] * dist[1]
    total_distance = math.sqrt(total_distance)
    print("total distance from start to distination:", total_distance)

    plot_point = [dist[0] + 500, -1 * dist[1] + 500]

    # me = tello.Tello()
    # me.connect()

    points = [(0, 0), (0, 0)]

    ########## Start Parallelism ##########
    drone = tello()
    drone.connect()
    drone.streamon()
    cap = drone.get_video_capture()

    # skal mainthread bare køre mapping selv og vision thread
    Thread_vision = th.Thread(target=vision, args=(cap))
    Thread_vision.run()
    mapping()


def drawPoints(img, points):
    for point in points:
        cv2.circle(img, point, 5, (0, 0, 255), cv2.FILLED)

    cv2.circle(img, points[-1], 8, (255, 0, 0), cv2.FILLED)
    cv2.putText(img, f'({(points[-1][0]-500)/100}, {-1*((points[-1][1]-500)/100)})m',
                (points[-1][0]+10, points[-1][1]+30), cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 255), 1)


def mapping():
    vals = [0, 0, 0, 0, 0, x, y]
    # drone.takeoff()
    # sleep(4)

    while True:
        # print(drone.get_battery())
        print(vals)
        #TODO should make another calculation to find the finish spot if front is towards target, choose either left or right before turning
        if not q.empty():
            #få svar fra queue om hvad den skal gøre
            svar = q.get()
            if svar == "TURN":

                q.queue.clear()

                #TODO HOW DO WE DO THE TURN?
            if svar == "FORWARD":
                #Continue forwards
                print()

            #TODO RECALC LOCATION AFTER A TURN
        location = [x, y]
        destination = [dist[0], dist[1]]
        destMinusLoca = [dist[0]-(location[0]-500), dist[1]-(location[1]-500)]

        # Calculate angle differences and adjust to 0
        ang_adjust_radians = math.atan2(destMinusLoca[0], destMinusLoca[1])
        ang_adjust_degrees = (180/math.pi)*ang_adjust_radians

        ang_adjust_degrees = int(ang_adjust_degrees)
        print(ang_adjust_degrees)
        print(-ang_adjust_degrees)

        if yaw != ang_adjust_degrees and ang_adjust_degrees > 0:
            # drone.rotate_clockwise(ang_adjust_degrees)
            # sleep(6)
            yaw = ang_adjust_degrees
        elif yaw != ang_adjust_degrees and ang_adjust_degrees < 0:
            # drone.rotate_counter_clockwise(-ang_adjust_degrees)
            # sleep(6)
            yaw = ang_adjust_degrees

        # Fly forward at the speed of 15cm/s until destination is reached
        distance = 0
        if vals[5] == plot_point[0] and vals[6] == plot_point[1] or vals[5] == plot_point[0]+1 and vals[6] == plot_point[1]+1 \
                or vals[5] == plot_point[0]-1 and vals[6] == plot_point[1]-1:
            print("Landing")
            # drone.land()
        else:
            vals[1] = 10
            distance = distInterval
            a = 270

        a += yaw
        vals[5] += int(distance * math.cos(math.radians(a)))
        vals[6] += int(distance * math.sin(math.radians(a)))

        # drone.send_rc_control(vals[0], vals[1], vals[2], vals[3])
        print(vals)
        sleep(interval)

        # Display map, mapping drone on screen
        img = np.zeros((1000, 1000, 3), np.uint8)
        points.append((vals[5], vals[6]))
        cv2.circle(img, plot_point, 5, (0, 255, 0), cv2.FILLED)
        drawPoints(img, points)
        cv2.imshow("Output", img)
        cv2.waitKey(1)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            # drone.streamoff()
            # drone.land()
            print("landing")
            break


def vision(cap):

    # Størrelse på drone billedet er 720x960

    # Vi kan lave områder på left fra (0,0) -> (720,320)   right fra (0,640) -> (720,960)   top fra (0,320)-> (240,640)    mid fra (240, 320) -> (480,640)   bot fra (480,320) -> (720,640)
    #cap = cv2.VideoCapture(0)


    currentframe = 1

    rectHandlerList.clear()

    while True:
        # Får fat i en frame fra dronen, enten via drone eller webcam
        ret, frame = cap.read(0)

        # vi laver en liste med rektangler, dette skal gøres hver frame
        rectList = []

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 23,10)  # 5th parameter = pixel neighbourhood size, 6th parameter = finetuning     #note, vi kan fra tælle alle firkanter mindre end X
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
        else:
            #FORWARD gets interpreted by mapping function to just continue / there could be a smarter way to do this
            q.put("FORWARD")


    cv2.destroyAllWindows()

    return