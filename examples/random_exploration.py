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
    MIN_DISTANCE = 0.3  # meters

    if range is None:
        return False
    else:
        return range < MIN_DISTANCE


def target_is_found():
    # TODO
    return False


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
                VELOCITY = 0.1
                field_of_view = 0.5
                distance_parallel_to_wall_btw_turns = 0.5
                is_parallel_to_first_wall = False
                first_wall_is_found = False
                start_following_target = False

                # TODO, if the target is already in the field of vision of the quadcopter, then the drone should enter
                #  the following_target step already and not try to find the 1st wall. We need to check for the target
                #  after every step of the algo !!!

                time.sleep(3)
                while keep_flying and not target_is_found():
                    # TODO add the check target
                    if not is_close(multiranger.front):
                        motion_commander.start_forward(VELOCITY)
                        time.sleep(1)

                    if is_close(multiranger.front):
                        motion_commander.stop()
                        rand = random.randint(0, 180)
                        motion_commander.turn_right(rand)

                        front_distance = multiranger.front
                        right_distance = multiranger.right
                        left_distance = multiranger.left
                        # if there is an obstacle closer to the drone on the right side than on the left side
                        # 1 is arbitrary, need to change the value
                        if multiranger.right < multiranger.left and right_distance < 1:
                            # computing the required angle to yaw in order to fly parallel to the wall.
                            angle_to_yaw = math.degrees(math.atan(right_distance / front_distance))
                            # Compute a random angle which is the required angle to be parallel to the wall +
                            # a random value which is between 0 and 180
                            randAngle = angle_to_yaw + rand
                            motion_commander.turn_left(randAngle)
                            time.sleep(1)

                        # if there is an obstacle closer to the drone on the left side than on the left side
                        elif multiranger.left < multiranger.right and left_distance < 1:
                            # computing the required angle to yaw in order to fly parallel to the wall.
                            angle_to_yaw = math.degrees(math.atan(left_distance / front_distance))
                            # Compute a random angle which is the required angle to be parallel to the wall +
                            # a random value which is between 0 and 180
                            randAngle = angle_to_yaw + rand
                            motion_commander.right(randAngle)
                            time.sleep(1)

                        else:
                            motion_commander.right(rand)
                            time.sleep(1)

                if target_is_found():
                    print('The drone has found the target!')
                    motion_commander.stop()
                    motion_commander.land()
                    keep_flying = False

                # security measure
                if not keep_flying:
                    motion_commander.stop()
                    motion_commander.land()
                    # TODO not sure about the landing technique and velocity
                # To avoid: "quadcopter often crashed once the battery level reached below
                # a certain unspecified threshold. "
                # TODO add a method to check the battery life. If the battery is too low,
                #  then the drone has to land before it crashes by itself because of the empty battery.

                print('Demo terminated!')
