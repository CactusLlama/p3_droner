#testing for capabilities for the drone.
import queue

import djitellopy as tello
import threading as th
q = queue.Queue()


def connect():
    #create object and try to connect if it works put 1 in the queue, if it does not work put -1 in queue
    drone = tello.Tello()
    try:
        drone.connect()
        print(str(drone.get_current_state()))
        q.put(1)
    except:
        q.put(-1)


if __name__ == "__main__":
    number_of_connections = input("How many times should the test be run?")
    #create the thread a 100 times and keep running it until it has finished. when all 100 threads have happened sum up the queue and display it
    for i in range(int(number_of_connections)):
        test = th.Thread(target=connect, args=())
        test.start()
        while True:
            if test.is_alive() == True:
                continue
            else:
                break

    summa = 0
    while not q.empty():
        summa += q.get()
    print("The drone has connected "+ str(summa)+ " times")

