from djitellopy import Tello
import cv2
import numpy as np
import math
from time import sleep


#############################Parameters#########################################
imageWidth = 640  # The image width
imageHeight = 480  # The image height
margin = 100

forwardS = 200/10  # Forward Speed in cm/s (at the speed 15cm/s = 11.7cm/10s) TEST THIS!
rotationS = 360/10  # Rotation speed in degrees/second (10s to rotate 360 degrees) (at 50d/s)
interval = 0.14128

distInterval = forwardS*interval  # 20*0.14128 = 2.8256
rotationInterval = rotationS*interval  # 36 * 0.25 = 9 (this is only used for keyboard input)
############################################

startCounter = 1

# CONNECT TO TELLO
drone = Tello()
drone.connect()
drone.forBackSpeed = 0
drone.leftRightSpeed = 0

# Not used yet
drone.upDownSpeed = 0
drone.yawSpeed = 0
drone.speed = 0


print(drone.get_battery())

drone.streamoff()
drone.streamon()
########################

frameWidth = imageWidth
frameHeight = imageHeight


global image_contour
global direction


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


def stackImages(sizeAdjustment, image_array):
    rows = len(image_array)
    cols = len(image_array[0])
    rowsAvailable = isinstance(image_array[0], list)
    width = image_array[0][0].shape[1]
    height = image_array[0][0].shape[0]
    if rowsAvailable:
        for x in range ( 0, rows):
            for y in range(0, cols):
                if image_array[x][y].shape[:2] == image_array[0][0].shape [:2]:
                    image_array[x][y] = cv2.resize(image_array[x][y], (0, 0), None, sizeAdjustment, sizeAdjustment)
                else:
                    image_array[x][y] = cv2.resize(image_array[x][y], (image_array[0][0].shape[1], image_array[0][0].shape[0]), None, sizeAdjustment, sizeAdjustment)
                if len(image_array[x][y].shape) == 2: image_array[x][y]= cv2.cvtColor(image_array[x][y], cv2.COLOR_GRAY2BGR)
        imageBlank = np.zeros((height, width, 3), np.uint8)
        hor = [imageBlank]*rows
        hor_con = [imageBlank]*rows
        for x in range(0, rows):
            hor[x] = np.hstack(image_array[x])
        ver = np.vstack(hor)
    else:
        for x in range(0, rows):
            if image_array[x].shape[:2] == image_array[0].shape[:2]:
                image_array[x] = cv2.resize(image_array[x], (0, 0), None, sizeAdjustment, sizeAdjustment)
            else:
                image_array[x] = cv2.resize(image_array[x], (image_array[0].shape[1], image_array[0].shape[0]), None, sizeAdjustment, sizeAdjustment)
            if len(image_array[x].shape) == 2: image_array[x] = cv2.cvtColor(image_array[x], cv2.COLOR_GRAY2BGR)
        hor= np.hstack(image_array)
        ver = hor
    return ver


def retrieve_contours(image, image_contour):
    global direction
    contours, hierarchy = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        areaMin = cv2.getTrackbarPos("Area", "Parameters")
        if area > areaMin:
            cv2.drawContours(image_contour, cnt, -1, (255, 0, 255), 7)
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
                    cv2.putText(image_contour, " GO RIGHT ", (20, 50), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 3)
                    cv2.rectangle(image_contour, (0, int(frameHeight / 2 - margin)),
                                  (int(frameWidth / 2) - margin, int(frameHeight / 2) + margin), (0, 0, 255),
                                  cv2.FILLED)
                    direction = 1
                elif (cx > int(frameWidth / 2)):
                    cv2.putText(image_contour, " GO LEFT ", (20, 50), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 3)
                    cv2.rectangle(image_contour, (int(frameWidth / 2 + margin), int(frameHeight / 2 - margin)),
                                  (frameWidth, int(frameHeight / 2) + margin), (0, 0, 255), cv2.FILLED)
                    direction = 2

            else:
                if (int(frameWidth/2)-margin < x+w <int(frameWidth / 2)) :
                    cv2.putText(image_contour, " GO RIGHT ", (20, 50), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 3)
                    cv2.rectangle(image_contour, (0, int(frameHeight / 2 - margin)), (int(frameWidth / 2) - margin, int(frameHeight / 2) + margin), (0, 0, 255), cv2.FILLED)
                    direction = 1
                elif (int(frameWidth/2) + margin > x > int(frameWidth / 2)):
                    cv2.putText(image_contour, " GO LEFT ", (20, 50), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 3)
                    cv2.rectangle(image_contour, (int(frameWidth / 2 + margin), int(frameHeight / 2 - margin)), (frameWidth, int(frameHeight / 2) + margin), (0, 0, 255), cv2.FILLED)
                    direction = 2
                elif (cy < int(frameHeight / 2) - margin):
                    cv2.putText(image_contour, " GO UP ", (20, 50), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 3)
                    cv2.rectangle(image_contour, (int(frameWidth / 2 - margin), 0), (int(frameWidth / 2 + margin), int(frameHeight / 2) - margin), (0, 0, 255), cv2.FILLED)
                    direction = 3
                elif (cy > int(frameHeight / 2) + margin):
                    cv2.putText(image_contour, " GO DOWN ", (20, 50), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 3)
                    cv2.rectangle(image_contour, (int(frameWidth / 2 - margin), int(frameHeight / 2) + margin), (int(frameWidth / 2 + margin), frameHeight), (0, 0, 255), cv2.FILLED)
                    direction = 4
                else: direction=0

            cv2.line(image_contour, (int(frameWidth / 2), int(frameHeight / 2)), (cx, cy), (0, 0, 255), 3)
            cv2.rectangle(image_contour, (x, y), (x + w, y + h), (0, 255, 0), 5)
            cv2.putText(image_contour, "Points: " + str(len(approx)), (x + w + 20, y + 20), cv2.FONT_HERSHEY_COMPLEX, .7, (0, 255, 0), 2)
            cv2.putText(image_contour, "Area: " + str(int(area)), (x + w + 20, y + 45), cv2.FONT_HERSHEY_COMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(image_contour, " " + str(int(x)) + " " + str(int(y)), (x - 20, y - 45), cv2.FONT_HERSHEY_COMPLEX, 0.7, (0, 255, 0), 2)
        else: direction=0


def display(droneFrames):
    cv2.line(droneFrames, (int(frameWidth / 2) - margin, 0), (int(frameWidth / 2) - margin, frameHeight), (255, 255, 0), 3)
    cv2.line(droneFrames, (int(frameWidth / 2) + margin, 0), (int(frameWidth / 2) + margin, frameHeight), (255, 255, 0), 3)
    cv2.circle(droneFrames, (int(frameWidth / 2), int(frameHeight / 2)), 5, (0, 0, 255), 5)
    cv2.line(droneFrames, (0, int(frameHeight / 2) - margin), (frameWidth, int(frameHeight / 2) - margin), (255, 255, 0), 3)
    cv2.line(droneFrames, (0, int(frameHeight / 2) + margin), (frameWidth, int(frameHeight / 2) + margin), (255, 255, 0), 3)


def draw_points(image, droneLocations):
    for point in droneLocations:
        cv2.circle(image, point, 5, (0, 0, 255), cv2.FILLED)

    cv2.circle(image, droneLocations[-1], 8, (255, 0, 0), cv2.FILLED)
    cv2.putText(image, f'({(droneLocations[-1][0] - 500) / 100}, {-1 * ((droneLocations[-1][1] - 500) / 100)})m',
                (droneLocations[-1][0] + 10, droneLocations[-1][1] + 30), cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 255), 1)


x = 500  # x start coordinate (500 since it´s the middle of the display width)
y = 500  # y start coordinate (500 since it´s the middle of the display height)
a = 0   # a is the angle from the y-axis in the positive direction when set to 270 degrees
yaw = 0  # this is the angle offset from the current direction of the drone
startRoute = 0  # Wait for this to become 1 before starting mapping and flight input
dest = [int, int]

print("Input the point where the drone has to go (in cm):\n")
dest[0] = int(input("Input x: "))  # Destination x coordinate in cm
dest[1] = int(input("Input y: "))  # Destination y coordinate in cm


# Calculate the total distance to destination from drone´s current position in a straight line
total_distance = dest[0] * dest[0] + dest[1] * dest[1]
total_distance = math.sqrt(total_distance)
print("total distance from start to distination:", total_distance)

# Enable variables and drone take of
plot_point = [dest[0] + 500, -1 * dest[1] + 500]  # Destination point to plot in the window
points = [(0, 0), (0, 0)]  # Current location and previous locations used for plotting
vals = [x, y]  # y are used as plotting points for drones current location
drone.forBackSpeed = 0
mapCounter = 0  # counter for updating map
rotationTime = 0
side_movement_count = 0
forward_move_count = 0

while True:
    print(drone.get_battery())

    # GET THE IMAGE FROM TELLO
    readFrame = drone.get_frame_read()
    droneFrame = readFrame.frame
    image = cv2.resize(droneFrame, (imageWidth, imageHeight))
    image_contour = image.copy()
    image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    h_min = cv2.getTrackbarPos("HUE Min","HSV")
    h_max = cv2.getTrackbarPos("HUE Max", "HSV")
    s_min = cv2.getTrackbarPos("SAT Min", "HSV")
    s_max = cv2.getTrackbarPos("SAT Max", "HSV")
    v_min = cv2.getTrackbarPos("VALUE Min", "HSV")
    v_max = cv2.getTrackbarPos("VALUE Max", "HSV")

    lower = np.array([h_min,s_min,v_min])
    upper = np.array([h_max,s_max,v_max])
    mask = cv2.inRange(image_hsv, lower, upper)
    result = cv2.bitwise_and(image, image, mask = mask)
    mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

    imgBlur = cv2.GaussianBlur(result, (7, 7), 1)
    imgGray = cv2.cvtColor(imgBlur, cv2.COLOR_BGR2GRAY)
    threshold1 = cv2.getTrackbarPos("Threshold1", "Parameters")
    threshold2 = cv2.getTrackbarPos("Threshold2", "Parameters")
    imgCanny = cv2.Canny(imgGray, threshold1, threshold2)
    kernel = np.ones((5, 5))
    imgDil = cv2.dilate(imgCanny, kernel, iterations=1)
    retrieve_contours(imgDil, image_contour)
    display(image_contour)

    if cv2.waitKey(1) & 0xFF == ord('f'):
        startCounter = 0
        startRoute = 1

    # Display map, mapping drone on screen
    img_map = np.zeros((1000, 1000, 3), np.uint8)
    points.append((vals[0], vals[1]))
    cv2.circle(img_map, plot_point, 5, (0, 255, 0), cv2.FILLED)
    draw_points(img_map, points)

    ################# FLIGHT
    if startCounter == 0:
        # Calculate distance from drone location to destination for x and y
        destination = [dest[0], dest[1]]
        destination_minus_location = [dest[0] - (vals[0] - 500), dest[1] - (vals[1] - 500)]

        # Calculate angle differences current angle(forward) to angle towards destination
        ang_adjust_radians = math.atan2(destination_minus_location[0], destination_minus_location[1])
        ang_adjust_degrees = (180 / math.pi) * ang_adjust_radians
        print("dest-location", destination_minus_location)
        print("start up adjust angle", ang_adjust_degrees)
        print("start yaw", yaw)
        drone.takeoff()
        sleep(4)
        drone.move_up(20)

        # Change angle so drone points towards the destination point
        if yaw != ang_adjust_degrees and ang_adjust_degrees > 0:
            drone.rotate_clockwise(int(ang_adjust_degrees))
            sleep(6)
            print("rotated?")
            yaw = ang_adjust_degrees
        elif yaw != ang_adjust_degrees and ang_adjust_degrees < 0:
            drone.rotate_counter_clockwise(int(-ang_adjust_degrees))
            sleep(6)
            print("rotated?")
            yaw = ang_adjust_degrees

        startCounter = 1

    distance = 0  # initialize distance for mapping

    if startRoute == 1:
        #Deadzone removed and yaw reversed
        if direction == 1:
            drone.forBackSpeed = 0
            drone.leftRightSpeed = 15
            a = 180
            distance = -distInterval
            side_movement_count += 1
            forward_move_count += 4
        elif direction == 2:
            drone.forBackSpeed = 0
            drone.leftRightSpeed = -15
            a = -180
            distance = distInterval
            side_movement_count -= 1
            forward_move_count -= 4
        # elif dir == 3:
            # me.up_down_velocity= 60
        # elif dir == 4:
            # me.up_down_velocity= -60
        else:
            # Fly forward at the speed of 15 cm/s until destination is reached
            if vals[0] >= plot_point[0] - 3 and vals[0] <= plot_point[0] + 3 and vals[1] >= plot_point[1] - 3 and vals[1] <= plot_point[1] + 3:
                print("Landing")
                drone.land()
            else:
                if forward_move_count != 0:
                    drone.leftRightSpeed = 0
                    drone.forBackSpeed = 15
                    distance = distInterval
                    a = 270
                    if forward_move_count > 0:
                        forward_move_count -= 1
                    else:
                        forward_move_count += 1
                elif side_movement_count != 0 and side_movement_count > 0 and forward_move_count == 0:
                    drone.forBackSpeed = 0
                    drone.leftRightSpeed = -15
                    distance = distInterval
                    a = -180
                    side_movement_count -= 1
                elif side_movement_count != 0 and side_movement_count < 0 and forward_move_count == 0:
                    drone.forBackSpeed = 0
                    drone.leftRightSpeed = 15
                    distance = -distInterval
                    a = 180
                    side_movement_count += 1
                else:
                    drone.leftRightSpeed = 0
                    drone.forBackSpeed = 15
                    distance = distInterval
                    a = 270
    else:
        drone.leftRightSpeed = 0
        drone.forBackSpeed = 0
        drone.upDownSpeed = 0
        drone.yawSpeed = 0


    if drone.send_rc_control:
        drone.send_rc_control(drone.leftRightSpeed, drone.forBackSpeed, drone.upDownSpeed, drone.yawSpeed)


    # update map of drones current location
    a += yaw
    print("yaw: ", yaw)
    print("angle: ", a)
    print("dist: ", distance)
    mapCounter += 1
    if mapCounter == 4:
        vals[0] += int(distance * math.cos(math.radians(a)))
        vals[1] += int(distance * math.sin(math.radians(a)))
        mapCounter = 0
        print("vals: ", vals)

    # SEND VELOCITY VALUES TO TELLO
    stack = stackImages(0.9, ([image, result], [imgDil, image_contour]))
    cv2.imshow('Horizontal Stacking', stack)
    cv2.imshow("Output", img_map)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        drone.land()
        break

# cap.release()
cv2.destroyAllWindows()
