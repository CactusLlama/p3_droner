import math
import queue

from djitellopy import tello
import numpy as np
import cv2
from time import sleep

import threading as th #for doing driving and vision at the same time
import multiprocessing as mp



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
            #if (320 < x < 640 or 320 < x+h < 640) and (240 < y < 480 or 240 < y+h < 480):
                # vi har en firkant som er større end midterfirkanten, så vi slår recthandler til
                AVOID_FLAG = 1
                # print("AVOID_FLAG == 1")
                # vi tilføjer kun firkant til listen hvis denne har en chance for at være stor nok til at være i vejen - Dette gør at vi undgår unødvendige firkanter i listen
                rectList.append(Rect(x, y, w, h))
                # print("rectlist tilføjet"+str(len(rectList)))

    # Indsæt område firkanter //KUN TIL DEBUG
    cv2.rectangle(fram, (0, 0), (320, 720), (0, 255, 0), 2)
    cv2.rectangle(fram, (640, 0), (960, 720), (0, 255, 0), 2)
    cv2.rectangle(fram, (320, 0), (640, 240), (0, 255, 0), 2)
    cv2.rectangle(fram, (320, 240), (640, 480), (0, 255, 0), 2)
    cv2.rectangle(fram, (320, 480), (640, 720), (0, 255, 0), 2)
    return fram, rectList


def rectHandler():

    global rectHandlerList
    # Hvordan gør sammenligner vi trekanterne?
    # lav kun denne hvis der er noget i vejen
    left = 0
    right = 0
    for i in range(NUM_OF_FRAMES):

        # print(temp[0].w)
        # if len(temp) > 0: temp.pop(0)
        print("STR PÅ LISTE"+str(type(rectHandlerList[i])))
        print("range of rect per frame"+ str(len(rectHandlerList[i])))
        for j in range(len(rectHandlerList[i])-1):
            print(str(type(rectHandlerList[i][j]))+" tpe af recthandlist")
            if type(rectHandlerList[i][j]) != Rect:
                print("list should be empty "+rectHandlerList[i][j])

                #håndterer hvis der ikke blev fundet nogen firkanter i framen

                continue
            # find centrum af firkanten
            print("J is running")
            # brug x til at finde ud af hvor meget der skal drejes
            centX = rectHandlerList[i][j].x + (rectHandlerList[i][j].w / 2)
            centY = rectHandlerList[i][j].y + (rectHandlerList[i][j].h / 2)

            if centX > FRAME_CENT_X :
                # drej til venstre for at undgå
                left+=1
                print("left ++ left is: "+str(left))
            else:
                # drej til højre for at undgå
                right += 1
                print("right++ right: "+str(right))
                print("højre " + str(right) + "Venstre = " + str(left))


    if (left > right):

        #q bliver brugt som kommunikation mellem threads
        q.put("L")

        #LOCK indtil at der er drejet fra mapping metoden
        print("Drej til venstre")
        print("højre " + str(right) + "Venstre = " + str(left))
    elif (left == 0 and right == 0) or left == right:
        drone.land()
        print("Value not reported run again")

    else:
        print("Drej til Højre")
        print("højre " + str(right) + "Venstre = " + str(left))
        q.put("R")

    return





def displayMap(vals, points):
    # Display map, mapping drone on screen
    img = np.zeros((1000, 1000, 3), np.uint8)
    points.append((vals[1], vals[2]))
    cv2.circle(img, plot_point, 5, (0, 255, 0), cv2.FILLED)
    img = draw_points(img, points)
    return img

def draw_points(img, points):
    for point in points:
        cv2.circle(img, point, 5, (0, 0, 255), cv2.FILLED)

    cv2.circle(img, points[-1], 8, (255, 0, 0), cv2.FILLED)
    cv2.putText(img, f'({(points[-1][0]-500)/100}, {-1*((points[-1][1]-500)/100)})m',
                (points[-1][0]+10, points[-1][1]+30), cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 255), 1)
    return img

def mapping():
    vals = [0, x, y] #[0] = speed [1] and [2] = x and y,
    plot_point = [dest[0] + 500, -1 * dest[1] + 500]  # Destination point to plot in the window
    points = [(0, 0), (0, 0)]  # Current location and previous locations used for plotting
    drone.takeoff()
    drone.streamon()
    sleep(3)
    yaw = 0
    a = 270
    # Show the map
    img = displayMap(vals, points)
    cv2.imshow("Output", img)
    cv2.waitKey(1)
    global q
    q.put("FORWARD")

    while True:
        # print(drone.get_battery())
        #sleep to make room for other thread
        #sleep(3)
        print(vals)
        #TODO should make another calculation to find the finish spot if front is towards target, choose either left or right before turning
        if not q.empty():

            #få svar fra queue om hvad den skal gøre
            svar = q.get()
            print("queue is going through: "+str(svar))
            if svar == "L":

                #drej 90 grader til venstre
                #og flyv en lille smule frem
                #drone.rotate_counter_clockwise(90)
                #drone.move_left(15)
                drone.send_rc_control(-15,0,0,0)
                distance = -distInterval
                print("turning left")
                #update yaw
                # yaw = yaw - 90

                # update map of drones current location TODO needs to be updated to update to the side
                a += yaw
                vals[1] += int(distance * math.cos(math.radians(a)))
                vals[2] += int(distance * math.sin(math.radians(a)))

            if svar == "R":
                # drone.rotate_clockwise(90)
                # yaw = yaw + 90
                print("turning right")
                #drone.move_right(15)
                drone.send_rc_control(15,0,0,0)
                distance = distInterval
                # update map of drones current location
                a += yaw
                vals[1] += int(distance * math.cos(math.radians(a)))
                vals[2] += int(distance * math.sin(math.radians(a)))


            if svar == "FORWARD":
                # skal dette i en funktion for sig selv?
                location = [x, y]
                destination = [dest[0], dest[1]]
                destination_minus_location = [dest[0] - (location[0] - 500), dest[1] - (location[1] - 500)]

                # Calculate angle differences current angle(forward) to angle towards destination
                ang_adjust_radians = math.atan2(destination_minus_location[0], destination_minus_location[1])
                ang_adjust_degrees = (180 / math.pi) * ang_adjust_radians

                # print(ang_adjust_degrees)
                # print(-ang_adjust_degrees)

                # Change angle so drone points towards the destination point
                if yaw != ang_adjust_degrees and ang_adjust_degrees > 0:
                    drone.rotate_clockwise(int(ang_adjust_degrees))
                    sleep(6)
                    yaw = ang_adjust_degrees
                elif yaw != ang_adjust_degrees and ang_adjust_degrees < 0:
                    drone.rotate_counter_clockwise(int(-ang_adjust_degrees))
                    sleep(6)
                    yaw = ang_adjust_degrees

                # Fly forward at the speed of 15 cm/s until destination is reached
                distance = 0
                if vals[1] >= plot_point[0] - 3 and vals[1] <= plot_point[0] + 3 and vals[2] >= plot_point[1] - 3 and \
                        vals[2] <= plot_point[1] + 3:
                    print("Landing")
                    drone.streamoff()
                    drone.land()
                else:
                    vals[0] = 15
                    distance = distInterval
                    a = 270
                drone.send_rc_control(0, vals[0], 0, 0)
                sleep(2)
                print(vals)
                a += yaw
                vals[1] += int(distance * math.cos(math.radians(a)))
                vals[2] += int(distance * math.sin(math.radians(a)))

            else:
                drone.send_rc_control(0,0,0,0) #could not get image so we stop
            #clear Q if there has been a turn

            q.queue.clear()







        # send command to drone with updated speed forward

        #drone.send_rc_control(0,25,0,0)
        print(vals)
        #sleep(2)
        displayMap(vals, points)
        #sleep(interval)

        #drone.send_rc_control(0,0,0,0)
        #sleep(3)
        #print("stop")
        #drone.send_rc_control(0, 0,0,0) #should stop and wait for video input

        #update the drones x and y coordinates TODO Where should this be located?
       # vals[1] += int(distance * math.cos(math.radians(a)))
       # vals[2] += int(distance * math.sin(math.radians(a)))

        # Emergency type q to land and exit while loop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            drone.streamoff()
            drone.land()
            print("landing")
            break
        drone.send_rc_control(0,0,0,0)
        vision()
        sleep(3)

def vision():

    # Størrelse på drone billedet er 720x960
    #drone.streamon()

    #cap = drone.get_video_capture()
    # Vi kan lave områder på left fra (0,0) -> (720,320)   right fra (0,640) -> (720,960)   top fra (0,320)-> (240,640)    mid fra (240, 320) -> (480,640)   bot fra (480,320) -> (720,640)
    #cap = cv2.VideoCapture(0)
    #vent nogle sekunder på at dronen og mapping starter op  | måske lav et handshake mellem threads

    currentframe = 1
    global rectHandlerList
    rectHandlerList.clear()

    while True:
        global AVOID_FLAG
        # Får fat i en frame fra dronen, enten via drone eller webcam
        #ret, frame = cap.read(0)
        frame = drone.get_frame_read().frame
        # vi laver en liste med rektangler, dette skal gøres hver frame
        rectList = []

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 443,30)  # 5th parameter = pixel neighbourhood size, 6th parameter = finetuning     #note, vi kan fra tælle alle firkanter mindre end X
            # thresh3 = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 23,
            #                               10)  # 5th parameter = pixel neighbourhood size HAS TO BE PRIME NUMBER, 6th parameter = finetuning     #note, vi kan fra tælle alle firkanter mindre end X
        frame1, rectList = findcontours(thresh, frame, rectList)

        cv2.imshow("adaptive - MEAN", frame1)  # MEAN fungerer umiddelbart bedst

        if (rectList != 0):
            rectHandlerList.append(rectList) #burde tilføje listen til den korrekte



        if cv2.waitKey(1) == ord('q'):
            drone.streamoff()
            drone.land()
            break
        if AVOID_FLAG == 1:
            if currentframe == NUM_OF_FRAMES:
                # gennemgå listen med recthandler

                rectHandler()

                rectHandlerList.clear()

                currentframe = 0
                AVOID_FLAG = 0
                return
            currentframe += 1
        else:
            if currentframe == NUM_OF_FRAMES:
                q.put("FORWARD")
                return
            #FORWARD gets interpreted by mapping function to just continue / there could be a smarter way to do this
            currentframe += 1


    #cv2.destroyAllWindows()

    return


if __name__ == "__main__":
    ############## Start before parallelism ############
    x = 500
    y = 500
    a = 0
    yaw = 0
    dest = [int, int]

    print("Input the point where the drone has to go (in cm):\n")
    dest[0] = int(input("Input x: "))
    dest[1] = int(input("Input y: "))

    #print(dist)

    total_distance = dest[0] * dest[0] + dest[1] * dest[1]
    total_distance = math.sqrt(total_distance)
    print("total distance from start to destination:", total_distance)

    plot_point = [dest[0] + 500, -1 * dest[1] + 500]


    points = [(0, 0), (0, 0)]

    ########## Start Parallelism ##########
    drone = tello.Tello()
    drone.connect()


    # skal mainthread bare køre mapping selv og vision thread
    #Thread_vision = th.Thread(target=vision, args=([cap]))
    #Thread_vision.start()
    mapping()
#    process_vis = mp.Process(target=vision,args=([cap]))
 #   process_vis.start()
 #   mapping()
 #   process_vis.join()
