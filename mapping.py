import math
from djitellopy import tello
import numpy as np
import cv2
from time import sleep

################ Parameters #################
forwardS = 200/10  # Forward Speed in cm/s (at the speed 15cm/s = 11.7cm/10s) TEST THIS!
rotationS = 360/10  # Rotation speed in degrees/second (10s to rotate 360 degrees) (at 50d/s)
interval = 0.25

distInterval = forwardS*interval  # 20*0.25= 5
rotationInterval = rotationS*interval  # 36 * 0.25 = 9 (this is only used for keyboard input)
############################################

me = tello.Tello()
me.connect()


def draw_points(img, points):
    for point in points:
        cv2.circle(img, point, 5, (0, 0, 255), cv2.FILLED)

    cv2.circle(img, points[-1], 8, (255, 0, 0), cv2.FILLED)
    cv2.putText(img, f'({(points[-1][0]-500)/100}, {-1*((points[-1][1]-500)/100)})m',
                (points[-1][0]+10, points[-1][1]+30), cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 255), 1)


if __name__ == "__main__":
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
    vals = [0, x, y] # vals[0] used as the speed, x and y are used as plotting points for drones current location
    me.takeoff()
    sleep(4)

    while True:
        # Display map, mapping drone on screen
        img = np.zeros((1000, 1000, 3), np.uint8)
        points.append((vals[1], vals[2]))
        cv2.circle(img, plot_point, 5, (0, 255, 0), cv2.FILLED)
        draw_points(img, points)
        cv2.imshow("Output", img)
        cv2.waitKey(1)

        # print(me.get_battery())
        print(vals)

        # Calculate distance from drone location to destination for x and y
        location = [x, y]
        destination = [dest[0], dest[1]]
        destination_minus_location = [dest[0] - (location[0] - 500), dest[1] - (location[1] - 500)]

        # Calculate angle differences current angle(forward) to angle towards destination
        ang_adjust_radians = math.atan2(destination_minus_location[0], destination_minus_location[1])
        ang_adjust_degrees = (180/math.pi)*ang_adjust_radians

        # print(ang_adjust_degrees)
        # print(-ang_adjust_degrees)

        # Change angle so drone points towards the destination point
        if yaw!= ang_adjust_degrees and ang_adjust_degrees > 0:
            me.rotate_clockwise(int(ang_adjust_degrees))
            sleep(6)
            yaw = ang_adjust_degrees
        elif yaw != ang_adjust_degrees and ang_adjust_degrees < 0:
            me.rotate_counter_clockwise(int(-ang_adjust_degrees))
            sleep(6)
            yaw = ang_adjust_degrees

        # Fly forward at the speed of 15 cm/s until destination is reached
        distance = 0
        if vals[1] >= plot_point[0]-3 and vals[1] <= plot_point[0]+3 and vals[2] >= plot_point[1]-3 and vals[2] <= plot_point[1]+3:
            print("Landing")
            me.land()
        else:
            vals[0] = 15
            distance = distInterval
            a = 270

        # send command to drone with updated speed forward
        me.send_rc_control(0, vals[0], 0, 0)
        print(vals)
        sleep(interval)

        # update map of drones current location
        a += yaw
        vals[1] += int(distance * math.cos(math.radians(a)))
        vals[2] += int(distance * math.sin(math.radians(a)))

        # Emergency type q to land and exit while loop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            me.land()
            print("landing")
            break


