from djitellopy import tello
import KeyPressModule as kp
from time import sleep

kp.init()
me = tello.Tello()
me.connect()


def getKeyboardInput():
    lr, fb, ud, yv, tl = 0, 0, 0, 0, 0
    speed = 50

    if kp.getKey("LEFT"): lr = -speed
    elif kp.getKey("RIGHT"): lr = speed

    if kp.getKey("UP"): fb = speed
    elif kp.getKey("DOWN"): fb = -speed

    if kp.getKey("w"): ud = speed
    elif kp.getKey("s"): ud = -speed

    if kp.getKey("a"): yv = speed
    elif kp.getKey("d"): yv = -speed

    if kp.getKey("q"): tl = 1
    if kp.getKey("e"): tl = 2

    return [lr, fb, ud, yv, tl]

while True:
    print(me.get_battery())
    vals = getKeyboardInput()

    if vals[4] == 1: me.takeoff()
    if vals[4] == 2: me.land()

    me.send_rc_control(vals[0], vals[1], vals[2], vals[3])
    sleep(0.05)
