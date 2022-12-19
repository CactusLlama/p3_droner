from djitellopy import Tello
import cv2
import numpy as np
import math
from time import sleep


######################################################################
width = 640  # WIDTH OF THE IMAGE
height = 480  # HEIGHT OF THE IMAGE
deadZone = 100
######################################################################
forwardS = 200/10  # Forward Speed in cm/s (at the speed 15cm/s = 11.7cm/10s) TEST THIS!
rotationS = 360/10  # Rotation speed in degrees/second (10s to rotate 360 degrees) (at 50d/s)
interval = 0.25

distInterval = forwardS*interval  # 20*0.25= 5
rotationInterval = rotationS*interval  # 36 * 0.25 = 9 (this is only used for keyboard input)
############################################

startCounter = 1

# CONNECT TO TELLO
me = Tello()
me.connect()
me.for_back_velocity = 0
me.left_right_velocity = 0
me.up_down_velocity = 0
me.yaw_velocity = 0
me.speed = 0


print(me.get_battery())

me.streamoff()
me.streamon()
########################

frameWidth = width
frameHeight = height
# cap = cv2.VideoCapture(1)
# cap.set(3, frameWidth)
# cap.set(4, frameHeight)
# cap.set(10,200)


global imgContour
global dir;


def empty(a):
    pass


cv2.namedWindow("HSV")
cv2.resizeWindow("HSV",640,240)
cv2.createTrackbar("HUE Min","HSV",20,179,empty)
cv2.createTrackbar("HUE Max","HSV",40,179,empty)
cv2.createTrackbar("SAT Min","HSV",148,255,empty)
cv2.createTrackbar("SAT Max","HSV",255,255,empty)
cv2.createTrackbar("VALUE Min","HSV",89,255,empty)
cv2.createTrackbar("VALUE Max","HSV",255,255,empty)

cv2.namedWindow("Parameters")
cv2.resizeWindow("Parameters",640,240)
cv2.createTrackbar("Threshold1","Parameters",166,255,empty)
cv2.createTrackbar("Threshold2","Parameters",171,255,empty)
cv2.createTrackbar("Area","Parameters",1750,30000,empty)


def stackImages(scale,imgArray):
    rows = len(imgArray)
    cols = len(imgArray[0])
    rowsAvailable = isinstance(imgArray[0], list)
    width = imgArray[0][0].shape[1]
    height = imgArray[0][0].shape[0]
    if rowsAvailable:
        for x in range ( 0, rows):
            for y in range(0, cols):
                if imgArray[x][y].shape[:2] == imgArray[0][0].shape [:2]:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (0, 0), None, scale, scale)
                else:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (imgArray[0][0].shape[1], imgArray[0][0].shape[0]), None, scale, scale)
                if len(imgArray[x][y].shape) == 2: imgArray[x][y]= cv2.cvtColor( imgArray[x][y], cv2.COLOR_GRAY2BGR)
        imageBlank = np.zeros((height, width, 3), np.uint8)
        hor = [imageBlank]*rows
        hor_con = [imageBlank]*rows
        for x in range(0, rows):
            hor[x] = np.hstack(imgArray[x])
        ver = np.vstack(hor)
    else:
        for x in range(0, rows):
            if imgArray[x].shape[:2] == imgArray[0].shape[:2]:
                imgArray[x] = cv2.resize(imgArray[x], (0, 0), None, scale, scale)
            else:
                imgArray[x] = cv2.resize(imgArray[x], (imgArray[0].shape[1], imgArray[0].shape[0]), None,scale, scale)
            if len(imgArray[x].shape) == 2: imgArray[x] = cv2.cvtColor(imgArray[x], cv2.COLOR_GRAY2BGR)
        hor= np.hstack(imgArray)
        ver = hor
    return ver


def getContours(img,imgContour):
    global dir
    contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        areaMin = cv2.getTrackbarPos("Area", "Parameters")
        if area > areaMin:
            cv2.drawContours(imgContour, cnt, -1, (255, 0, 255), 7)
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
            #print(len(approx))
            x , y , w, h = cv2.boundingRect(approx)
            cx = int(x + (w / 2))  # CENTER X OF THE OBJECT
            cy = int(y + (h / 2))  # CENTER Y OF THE OBJECT
            #kris: deadzone removed
            #if the object is large, use the center until it is out of the picture
            if(x+h > 200):
                if (cx < int(frameWidth / 2)):
                    cv2.putText(imgContour, " GO RIGHT ", (20, 50), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 3)
                    cv2.rectangle(imgContour, (0, int(frameHeight / 2 - deadZone)),
                                  (int(frameWidth / 2) - deadZone, int(frameHeight / 2) + deadZone), (0, 0, 255),
                                  cv2.FILLED)
                    dir = 1
                elif (cx > int(frameWidth / 2)):
                    cv2.putText(imgContour, " GO LEFT ", (20, 50), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 3)
                    cv2.rectangle(imgContour, (int(frameWidth / 2 + deadZone), int(frameHeight / 2 - deadZone)),
                                  (frameWidth, int(frameHeight / 2) + deadZone), (0, 0, 255), cv2.FILLED)
                    dir = 2

            else:
                if (int(frameWidth/2)-deadZone < x+w <int(frameWidth/2)) :
                    cv2.putText(imgContour, " GO RIGHT " , (20, 50), cv2.FONT_HERSHEY_COMPLEX,1,(0, 0, 255), 3)
                    cv2.rectangle(imgContour,(0,int(frameHeight/2-deadZone)),(int(frameWidth/2)-deadZone,int(frameHeight/2)+deadZone),(0,0,255),cv2.FILLED)
                    dir = 1
                elif (int(frameWidth/2)+deadZone > x > int(frameWidth / 2)):
                    cv2.putText(imgContour, " GO LEFT ", (20, 50), cv2.FONT_HERSHEY_COMPLEX,1,(0, 0, 255), 3)
                    cv2.rectangle(imgContour,(int(frameWidth/2+deadZone),int(frameHeight/2-deadZone)),(frameWidth,int(frameHeight/2)+deadZone),(0,0,255),cv2.FILLED)
                    dir = 2
                elif (cy < int(frameHeight / 2) - deadZone):
                    cv2.putText(imgContour, " GO UP ", (20, 50), cv2.FONT_HERSHEY_COMPLEX,1,(0, 0, 255), 3)
                    cv2.rectangle(imgContour,(int(frameWidth/2-deadZone),0),(int(frameWidth/2+deadZone),int(frameHeight/2)-deadZone),(0,0,255),cv2.FILLED)
                    dir = 3
                elif (cy > int(frameHeight / 2) + deadZone):
                    cv2.putText(imgContour, " GO DOWN ", (20, 50), cv2.FONT_HERSHEY_COMPLEX, 1,(0, 0, 255), 3)
                    cv2.rectangle(imgContour,(int(frameWidth/2-deadZone),int(frameHeight/2)+deadZone),(int(frameWidth/2+deadZone),frameHeight),(0,0,255),cv2.FILLED)
                    dir = 4
                else: dir=0

            cv2.line(imgContour, (int(frameWidth/2),int(frameHeight/2)), (cx,cy),(0, 0, 255), 3)
            cv2.rectangle(imgContour, (x, y), (x + w, y + h), (0, 255, 0), 5)
            cv2.putText(imgContour, "Points: " + str(len(approx)), (x + w + 20, y + 20), cv2.FONT_HERSHEY_COMPLEX, .7,(0, 255, 0), 2)
            cv2.putText(imgContour, "Area: " + str(int(area)), (x + w + 20, y + 45), cv2.FONT_HERSHEY_COMPLEX, 0.7,(0, 255, 0), 2)
            cv2.putText(imgContour, " " + str(int(x)) + " " + str(int(y)), (x - 20, y - 45), cv2.FONT_HERSHEY_COMPLEX,0.7,(0, 255, 0), 2)
        else: dir=0


def display(img):
    cv2.line(img,(int(frameWidth/2)-deadZone,0),(int(frameWidth/2)-deadZone,frameHeight),(255,255,0),3)
    cv2.line(img,(int(frameWidth/2)+deadZone,0),(int(frameWidth/2)+deadZone,frameHeight),(255,255,0),3)
    cv2.circle(img,(int(frameWidth/2),int(frameHeight/2)),5,(0,0,255),5)
    cv2.line(img, (0,int(frameHeight / 2) - deadZone), (frameWidth,int(frameHeight / 2) - deadZone), (255, 255, 0), 3)
    cv2.line(img, (0, int(frameHeight / 2) + deadZone), (frameWidth, int(frameHeight / 2) + deadZone), (255, 255, 0), 3)


def draw_points(img, points):
    for point in points:
        cv2.circle(img, point, 5, (0, 0, 255), cv2.FILLED)

    cv2.circle(img, points[-1], 8, (255, 0, 0), cv2.FILLED)
    cv2.putText(img, f'({(points[-1][0]-500)/100}, {-1*((points[-1][1]-500)/100)})m',
                (points[-1][0]+10, points[-1][1]+30), cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 255), 1)


x = 500  # x start coordinate (500 since it´s the middle of the display width)
y = 500  # y start coordinate (500 since it´s the middle of the display height)
a = 0   # a is the angle from the y-axis in the positive direction when set to 270 degrees
yaw = 0 # this is the angle offset from the current direction of the drone
dest = [int, int]

print("Input the point where the drone has to go (in cm):\n")
dest[0] = int(input("Input x: "))  # Destination x coordinate in cm
dest[1] = int(input("Input y: "))  # Destination y coordinate in cm

# print(dist)

# Calculate the total distance to destination from drone´s current position in a straight line
total_distance = dest[0] * dest[0] + dest[1] * dest[1]
total_distance = math.sqrt(total_distance)
print("total distance from start to distination:", total_distance)

# Enable variables and drone take of
plot_point = [dest[0] + 500, -1 * dest[1] + 500]  # Destination point to plot in the window
points = [(0, 0), (0, 0)]  # Current location and previous locations used for plotting
vals = [x, y]  # vals[0] used as the speed, x and y are used as plotting points for drones current location
me.for_back_velocity = 0

while True:
    # GET THE IMAGE FROM TELLO
    frame_read = me.get_frame_read()
    myFrame = frame_read.frame
    img = cv2.resize(myFrame, (width, height))
    imgContour = img.copy()
    imgHsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    h_min = cv2.getTrackbarPos("HUE Min","HSV")
    h_max = cv2.getTrackbarPos("HUE Max", "HSV")
    s_min = cv2.getTrackbarPos("SAT Min", "HSV")
    s_max = cv2.getTrackbarPos("SAT Max", "HSV")
    v_min = cv2.getTrackbarPos("VALUE Min", "HSV")
    v_max = cv2.getTrackbarPos("VALUE Max", "HSV")

    lower = np.array([h_min,s_min,v_min])
    upper = np.array([h_max,s_max,v_max])
    mask = cv2.inRange(imgHsv,lower,upper)
    result = cv2.bitwise_and(img,img, mask = mask)
    mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

    imgBlur = cv2.GaussianBlur(result, (7, 7), 1)
    imgGray = cv2.cvtColor(imgBlur, cv2.COLOR_BGR2GRAY)
    threshold1 = cv2.getTrackbarPos("Threshold1", "Parameters")
    threshold2 = cv2.getTrackbarPos("Threshold2", "Parameters")
    imgCanny = cv2.Canny(imgGray, threshold1, threshold2)
    kernel = np.ones((5, 5))
    imgDil = cv2.dilate(imgCanny, kernel, iterations=1)
    getContours(imgDil, imgContour)
    display(imgContour)

    if cv2.waitKey(1) & 0xFF == ord('f'):
        startCounter = 0

    # Display map, mapping drone on screen
    img_map = np.zeros((1000, 1000, 3), np.uint8)
    points.append((vals[0], vals[1]))
    cv2.circle(img_map, plot_point, 5, (0, 255, 0), cv2.FILLED)
    draw_points(img_map, points)
    cv2.imshow("Output", img_map)
    cv2.waitKey(1)

    # Calculate distance from drone location to destination for x and y
    location = [x, y]
    destination = [dest[0], dest[1]]
    destination_minus_location = [dest[0] - (location[0] - 500), dest[1] - (location[1] - 500)]

    # Calculate angle differences current angle(forward) to angle towards destination
    ang_adjust_radians = math.atan2(destination_minus_location[0], destination_minus_location[1])
    ang_adjust_degrees = (180 / math.pi) * ang_adjust_radians

    ################# FLIGHT
    if startCounter == 0:
        me.takeoff()
        startCounter = 2

    #Deadzone removed and yaw reversed
    if dir == 1:
        me.left_right_velocity = 17
        distance = distInterval
        yaw = 180
    elif dir == 2:
        me.left_right_velocity = -17
        distance = distInterval
        yaw = -180
    elif dir == 3:
        me.up_down_velocity= 60
    elif dir == 4:
        me.up_down_velocity= -60
    elif startCounter == 2:
        # Change angle so drone points towards the destination point
        if yaw != ang_adjust_degrees and ang_adjust_degrees > 0:
            # me.rotate_clockwise(int(ang_adjust_degrees))
            sleep(4)
            yaw = ang_adjust_degrees
        elif yaw != ang_adjust_degrees and ang_adjust_degrees < 0:
            # me.rotate_counter_clockwise(int(-ang_adjust_degrees))
            sleep(4)
            yaw = ang_adjust_degrees

        # Fly forward at the speed of 15 cm/s until destination is reached
        distance = 0
        if vals[1] >= plot_point[0] - 3 and vals[1] <= plot_point[0] + 3 and vals[2] >= plot_point[1] - 3 and vals[2] <= plot_point[1] + 3:
            print("Landing")
            # me.land()
        else:
            # me.for_back_velocity = 17
            distance = distInterval
            a = 270

        # update map of drones current location
        a += yaw
        vals[0] += int(distance * math.cos(math.radians(a)))
        vals[1] += int(distance * math.sin(math.radians(a)))
    else:
        me.left_right_velocity = 0; me.for_back_velocity = 0; me.up_down_velocity = 0; me.yaw_velocity = 0

    # SEND VELOCITY VALUES TO TELLO
    if me.send_rc_control:
        me.send_rc_control(me.left_right_velocity, me.for_back_velocity, me.up_down_velocity, me.yaw_velocity)
    print(dir)

    stack = stackImages(0.9, ([img, result], [imgDil, imgContour]))
    cv2.imshow('Horizontal Stacking', stack)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        me.land()
        break

# cap.release()
cv2.destroyAllWindows()