"""
Script to make the drone explore the room in order to find the landing platform.
It will take off, fly forward till it is close to a wall. Then it will turn and fly parallel to the wall.
Afterwards it will fly in a "rectangular pattern".
This approach should work for simple squared rooms.

The demo is ended by either pressing Ctrl-C or by holding your hand above the
Crazyflie.

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

def is_within(range, distance):
    if range is None:
        return False
    else:
        return range < distance

def target_is_found():
    # TODO
    return False

# Determine whether the drone detects something around him with the multi-ranger deck
def is_in_range(range):
    MIN_DISTANCE = 0.75  # meters
    if range is None:
        return False
    else:
        return range < MIN_DISTANCE

if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    cf = Crazyflie(rw_cache='./cache')
    with SyncCrazyflie(URI, cf=cf) as scf:
        # 1. Takes off (1st step) when the commander is created. Hovers at 1 m
        with MotionCommander(scf,0.5) as motion_commander:
            with Multiranger(scf) as multiranger:
                print("took off")
                keep_flying = True
                switch = "next_is_right"
                HEIGHT = 0.5  # meter
                VELOCITY = 0.2
                is_parallel_to_first_wall = False
                first_wall_is_found = False
                start_following_target = False

                while(keep_flying):
                    # First starting technique: Doesn't check the surroundings when hovering after the take off.
                    # It goes straight
                    motion_commander.start_forward(VELOCITY)

                    while not first_wall_is_found:
                        if is_close(multiranger.front):
                            motion_commander.stop()
                            front_distance = multiranger.front
                            right_distance = multiranger.right
                            left_distance = multiranger.left
                            print("ranges (front, right, left): ", front_distance, right_distance,
                                 left_distance)
                            if is_within(right_distance, 4):
                                print("going to turn to be parallel to the wall")
                                # computing the required angle to yaw in order to fly parallel to the wall.
                                angle_to_yaw = math.degrees(1.57 - math.atan(front_distance / right_distance))
                                # turn left and be parallel to the wall
                                motion_commander.turn_left(angle_to_yaw)
                                print("turned")
                                first_wall_is_found = True
                                angle_to_yaw_radians = math.radians(angle_to_yaw)
                                # check whether the drone is // or not by checking the right distance between
                                # the drone and the wall
                                if multiranger.right == (right_distance * math.sin(angle_to_yaw_radians) + 0.1) or \
                                        multiranger.right == (right_distance * math.sin(angle_to_yaw) - 0.1):
                                    is_parallel_to_first_wall = True
                                    print("Is parallel to the wall: ", is_parallel_to_first_wall)

                            # In case the left and right distances are equal or if the distance is smaller than 1m,
                            # then it will turn 90 degrees in a random direction (left or right)
                            else:
                                motion_commander.turn_left(90)

                                first_wall_is_found = True

                    print("Reached the first wall")
                    # The next step is to fly parallel to the wall. The quadcopter has now been rotated
                    # and the front is pointing parallel to the encountered wall.
                    if not is_close(multiranger.front):
                        motion_commander.start_forward(VELOCITY)

                    while not target_is_found():
                        if is_close(multiranger.front):
                            motion_commander.stop()

                            if not is_close(multiranger.left):
                                motion_commander.turn_left(90)
                                time.sleep(0.2)

                            if not is_close(multiranger.front):
                                motion_commander.start_forward(VELOCITY)

                    if target_is_found():
                        print('The drone has found the target!')
                        motion_commander.stop()
                        motion_commander.land()
                        keep_flying = False

                    if not keep_flying:
                        print("security")
                        motion_commander.stop()
                        motion_commander.land()

                    print('Demo terminated!')
