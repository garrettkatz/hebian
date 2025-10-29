import numpy as np
from hardware import RosieWrapper

rosie = RosieWrapper()

while True:

    print("""
    lower/Upper case for big/Small moves
    i: forward
    k: backward
    j: left
    l: right
    u: turn left
    o: turn right
    q: end this script
    """)

    command = input("Enter command: ")

    small = command.isupper()
    command = command.lower()

    if command == "q": break

    if command in "ijkl":
        xy_delta = np.array([
            [+1, 0],
            [0, +1],
            [-1, 0],
            [0, -1.],
        ])["ijkl".index(command)]

        xy_delta *= (.1 if small else .5)
        duration = (.8 if small else 4.)
        rosie.move_base(xy_delta, duration)

    if command in "uo":
        angle = np.array([+1, -1.])["uo".index(command)]

        angle *= (.1 if small else .5)
        duration = (.4 if small else 2.)
        rosie.rotate_base(angle, duration) 

