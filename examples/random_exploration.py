"""
Script to make the drone explore the room in order to find the landing platform.
It will take off, fly forward till it is reaches a wall. Afterwards it will yaw (random angle) and
fly forward again till it reaches another obstacle.  This process will be repeated till the drone finds the target.

The demo is ended by pressing Ctrl-C.

For the example to run the following hardware is needed:
 * Crazyflie 2.0
 * Crazyradio PA
 * Flow deck
 * Multiranger deck
"""
import logging
import sys
import time
import math
import random

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multiranger import Multiranger

URI = 'radio://0/80/2M'

if len(sys.argv) > 1:
    URI = sys.argv[1]

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


# Determine whether the drone is closed to an obstacle or not. Let's say the threshold is 0.3
def is_close(range):
    MIN_DISTANCE = 0.5  # meters

    if range is None:
        return False
    else:
        return range < MIN_DISTANCE


def go_outward():
    delta = (multiranger.left) / 2
    motion_commander.move_distance(0, delta, 0)
    motion_commander.move_distance(0, -delta, 0)

if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    cf = Crazyflie(rw_cache='./cache')
    with SyncCrazyflie(URI, cf=cf) as scf:
        # 1. Takes off (1st step) when the commander is created. Hovers at 1 m
        with MotionCommander(scf,0.3) as motion_commander:
            with Multiranger(scf) as multiranger:
                keep_flying = True
                ranges = ["front", "left", "right", "back"]
                switch = "next_is_right"
                HEIGHT = 0.3  # meter

                forward_velocity= 0.2
                wall_distance = 0.3
                outward_time = 40

                #
                outward_timer = outward_time
                time.sleep(3)
                first_wall_is_found = False

                # align to wall

                motion_commander.start_forward(forward_velocity)
                while not first_wall_is_found:
                    # if the drone is close to a wall in front, then it stops etc. And it will set
                    # "first_wall_is_found" to true to exit this while loop
                    # if there is nothing close in front, then it just doesn't do any action, it will continue
                    # the start_forward move that has been called after the take off.
                    # if is_close(multiranger.up):
                    #         keep_flying = False

                    if is_close(multiranger.front):
                        motion_commander.stop()

                        front_distance = multiranger.front
                        right_distance = multiranger.right
                        left_distance = multiranger.left

                        # if there is an obstacle closer to the drone on the right side than on the left side
                        if multiranger.right < multiranger.left and right_distance < 1:
                            # computing the required angle to yaw in order to fly parallel to the wall.
                            angle_to_yaw = math.degrees(math.atan(right_distance / front_distance))
                            # turn left and be parallel to the wall
                            motion_commander.turn_left(angle_to_yaw)
                            first_wall_is_found = True
                            angle_to_yaw_radians = math.radians(angle_to_yaw)

                        # if there is an obstacle closer to the drone on the left side than on the left side
                        elif multiranger.left < multiranger.right and left_distance < 1:
                            # computing the required angle to yaw in order to fly parallel to the wall.
                            angle_to_yaw = math.degrees(math.atan(left_distance / front_distance))
                            # turn right and be parallel to the wall
                            motion_commander.turn_right(angle_to_yaw)
                            first_wall_is_found = True
                            angle_to_yaw_radians = math.radians(angle_to_yaw)

                        # In case the left and right distances are equal or if the distance is smaller than 1m,
                        # then it will turn 90 degrees in a random direction (left or right)
                        else:
                            r1 = random.randint(0, 1)
                            if r1 == 0:
                                motion_commander.turn_left(90)
                            else:
                                motion_commander.turn_right(90)
                            first_wall_is_found = True
                        # TODO add the corner case


                # exploration
                while True:

                    motion_commander.start_forward(forward_velocity)

                    if multiranger.right < wall_distance: 
                        print("wall too close to the right")
                        motion_commander.move_distance(0, wall_distance - multiranger.right, 0)

                    if multiranger.front < wall_distance:
                        motion_commander.turn_left(90)
                        outward_timer = outward_time

                    outward_timer -= 1
                    
                    if(outward_timer == 0):
                        go_outward()
                        outward_timer = outward_time
                    time.sleep(0.1)                        