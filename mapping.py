import math
from djitellopy import tello
import numpy as np
import cv2
from time import sleep

################ Parameters ##################
forwardS = 200/10  # Forward Speed in cm/s (at the speed 15cm/s = 11.7cm/10s) TEST THIS!
rotationS = 360/10  # Rotation speed in degrees/second (10s to rotate 360 degrees) (at 50d/s)
interval = 0.25

distInterval = forwardS*interval  # 20*0.25= 5
rotationInterval = rotationS*interval  # 36 * 0.25 = 9 (this is only used for keyboard input)
##############################################
x = 500
y = 500
a = 0
yaw = 0
dist = [int, int]


print("Input the point where the drone has to go (in cm):\n")
dist[0] = int(input("Input x: "))
dist[1] = int(input("Input y: "))

print(dist)

total_distance  = dist[0]*dist[0]+dist[1]*dist[1]
total_distance  = math.sqrt(total_distance)
print("total distance from start to distination:", total_distance)

plot_point = [dist[0]+500, -1*dist[1]+500]


# me = tello.Tello()
# me.connect()

points = [(0, 0), (0, 0)]



def drawPoints(img, points):
    for point in points:
        cv2.circle(img, point, 5, (0, 0, 255), cv2.FILLED)

    cv2.circle(img, points[-1], 8, (255, 0, 0), cv2.FILLED)
    cv2.putText(img, f'({(points[-1][0]-500)/100}, {-1*((points[-1][1]-500)/100)})m',
                (points[-1][0]+10, points[-1][1]+30), cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 255), 1)


if __name__ == "__main__":
    vals = [0, 0, 0, 0, 0, x, y]
    # me.takeoff()
    # sleep(4)

    while True:
        # print(me.get_battery())
        print(vals)

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
            # me.rotate_clockwise(ang_adjust_degrees)
            # sleep(6)
            yaw = ang_adjust_degrees
        elif yaw != ang_adjust_degrees and ang_adjust_degrees < 0:
            # me.rotate_counter_clockwise(-ang_adjust_degrees)
            # sleep(6)
            yaw = ang_adjust_degrees

        # Fly forward at the speed of 15cm/s until destination is reached
        distance = 0
        if vals[5] == plot_point[0] and vals[6] == plot_point[1] or vals[5] == plot_point[0]+1 and vals[6] == plot_point[1]+1 \
                or vals[5] == plot_point[0]-1 and vals[6] == plot_point[1]-1:
            print("Landing")
            # me.land()
        else:
            vals[1] = 10
            distance = distInterval
            a = 270

        a += yaw
        vals[5] += int(distance * math.cos(math.radians(a)))
        vals[6] += int(distance * math.sin(math.radians(a)))

        # me.send_rc_control(vals[0], vals[1], vals[2], vals[3])
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
            # me.land()
            print("landing")
            break


