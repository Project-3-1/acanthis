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

# Determine whether the drone detects something around him with the multi-ranger deck
def is_in_range(range):
    MIN_DISTANCE = 0.75  # meters
    if range is None:
        return False
    else:
        return range < MIN_DISTANCE

def target_is_found():
    # TODO
    return False

def turn_at_wall(direction):
    # 0 means right
    if direction == 0:
        motion_commander.turn_right(90)
    # 1 means left
    if direction == 1:
        motion_commander.turn_left(90)



if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    cf = Crazyflie(rw_cache='./cache')
    with SyncCrazyflie(URI, cf=cf) as scf:
        # 1. Takes off (1st step) when the commander is created. Hovers at 1 m
        with MotionCommander(scf,0.3) as motion_commander:
            with Multiranger(scf) as multiranger:
                keep_flying = True
                ranges = ["front","left","right","back"]
                switch = "next_is_right"
                HEIGHT = 0.3  # meter
                VELOCITY = 0.1
                field_of_view = 0.5
                distance_parallel_to_wall_btw_turns = 0.5
                is_parallel_to_first_wall = False
                first_wall_is_found = False
                start_following_target = False


                # TODO, if the target is already in the field of vision of the quadcopter, then the drone should enter the following_target step already and not try to find the 1st wall. We need to check for the target after every step of the algo !!!

                ########################
                # First starting technique: Doesn't check the surroundings when hovering after the take off. It goes straight
                time.sleep(3)
                while(keep_flying):
                    motion_commander.start_forward(VELOCITY)
                    ########################

                    """"
                    ########################
                    # second starting technique: Check once if there is any obstacle around the drone. If yes, it will move to this obstacle without yawing.
                    for range in ranges:
                        if is_in_range(range):
                            if is_close(range):
                                motion_commander.stop()
                            else:
                                if range == "front":
                                    motion_commander.start_forward(VELOCITY)
                                elif range == "back":
                                    motion_commander.start_back(VELOCITY)
                                elif range == "right":
                                    motion_commander.start_right(VELOCITY)
                                elif range == "left":
                                    motion_commander.start_left(VELOCITY)
                        else:
                            motion_commander.start_forward(VELOCITY)
                    #####################
                    """

                    """"
                    ########################
                    # third starting technique: Check all the time if there is any obstacle around the drone. If yes, it will move to this obstacle without yawing.
                    # TODO implement a for loop to do this many times
                    for range in ranges:
                        if is_in_range(range):
                            if is_close(range):
                                motion_commander.stop()
                            else:
                                if range == "front":
                                    motion_commander.start_forward(VELOCITY)
                                elif range == "back":
                                    motion_commander.start_back(VELOCITY)
                                elif range == "right":
                                    motion_commander.start_right(VELOCITY)
                                elif range == "left":
                                    motion_commander.start_left(VELOCITY)
                        else:
                            motion_commander.start_forward(VELOCITY)
                    #####################
                    """

                    # To find a wall so it goes straight till it finds one
                    # This is one option to compute the required angle to yaw in order to fly parallel to the wall.
                    # I (Selim) have another approach in mind to compute it in case this one doesn't work.

                    # TODO if we do the 2nd or the 3rd starting approach, then we need to check all ranges and not only the front one.
                    # However, if we perform the 1st starting approach, then only the front range is required to check.
                    while not first_wall_is_found:
                        # if the drone is close to a wall in front, then it stops etc. And it will set "first_wall_is_found" to true to exit this while loop
                        # if there is nothing close in front, then it just doesn't do any action, it will continue the start_forward move that has been called after the take off.
                        if is_close(multiranger.front):
                            motion_commander.stop()
                            # correct the height if necessary
                            # difference_of_height_btw_desired_and_actual = HEIGHT - motion_commander.get_height()
                            # if difference_of_height_btw_desired_and_actual > 0.05:
                            #     motion_commander.up(difference_of_height_btw_desired_and_actual,VELOCITY)
                            # if difference_of_height_btw_desired_and_actual < -0.05:
                            #     motion_commander.down(-difference_of_height_btw_desired_and_actual,VELOCITY)

                            front_distance = multiranger.front
                            right_distance = multiranger.right
                            left_distance = multiranger.left
                            # if there is an obstacle closer to the drone on the right side than on the left side
                            if multiranger.right < multiranger.left and right_distance < 1:  # 1 is arbitrary, need to change the value
                                # computing the required angle to yaw in order to fly parallel to the wall.
                                angle_to_yaw = math.degrees(math.atan(right_distance / front_distance))
                                # turn left and be parallel to the wall
                                motion_commander.turn_left(angle_to_yaw)
                                first_wall_is_found = True
                                angle_to_yaw_radians = math.radians(angle_to_yaw)
                                # check whether the drone is // or not by checking the right distance between the drone and the wall
                                if multiranger.right == (right_distance * math.sin(angle_to_yaw_radians) + 0.1) or multiranger.right == (right_distance * math.sin(angle_to_yaw) - 0.1):
                                    is_parallel_to_first_wall = True

                            # if there is an obstacle closer to the drone on the left side than on the left side
                            elif multiranger.left < multiranger.right and left_distance < 1:
                                # computing the required angle to yaw in order to fly parallel to the wall.
                                angle_to_yaw = math.degrees(math.atan(left_distance / front_distance))
                                # turn right and be parallel to the wall
                                motion_commander.turn_right(angle_to_yaw)
                                first_wall_is_found = True
                                angle_to_yaw_radians = math.radians(angle_to_yaw)
                                # check whether the drone is // or not by checking the right distance between the drone and the wall
                                if multiranger.left == (left_distance * math.sin(angle_to_yaw_radians) + 0.1) or multiranger.left == (left_distance * math.sin(angle_to_yaw) - 0.1):
                                    is_parallel_to_first_wall = True

                            # In case the left and right distances are equal or if the distance is smaller than 1m, then it will turn 90 degrees in a random direction (left or right)
                            else:
                                r1 = random.randint(0, 1)
                                if r1 == 0:
                                    motion_commander.turn_left(90)
                                else:
                                    motion_commander.turn_right(90)
                                first_wall_is_found = True
                            # TODO add the corner case

                    # correct the height if necessary
                    # difference_of_height_btw_desired_and_actual = HEIGHT - motion_commander.get_height()
                    # if difference_of_height_btw_desired_and_actual > 0.05:
                    #     motion_commander.up(difference_of_height_btw_desired_and_actual, VELOCITY)
                    # if difference_of_height_btw_desired_and_actual < -0.05:
                    #     motion_commander.down(-difference_of_height_btw_desired_and_actual, VELOCITY)

                    # The next step is to fly parallel to the wall. The quadcopter has now been rotated and the front is pointing parallel to the encountered wall.
                    if not is_close(multiranger.front):
                        motion_commander.start_forward(VELOCITY)
                    # now it should have met the second wall and it has to turn once, 90 degrees, go straight // to the wall and
                    # then turn again 90 degrees to go back to the same direction as before but other way around and a bit further away.

                    # start the Left Left - Right Right pattern to explore the room.
                    # The drone turns 90 degrees every time it is close to as wall and then flies for a certain pre determined distance before turning again (same direction as before)

                    #Initialize switch direction
                    if multiranger.right < multiranger.left:
                        switch = "next_is_left"
                    else:
                        switch = "next_is_right"

                    while not target_is_found():
                        # safety method to stop the drone by holding a hand above it.
                        if is_close(multiranger.up):
                            keep_flying = False
                        # TODO add another command to set keepflying to false

                        # correct the height if necessary
                        # difference_of_height_btw_desired_and_actual = HEIGHT - motion_commander.get_height()
                        # if difference_of_height_btw_desired_and_actual > 0.05:
                        #     motion_commander.up(difference_of_height_btw_desired_and_actual, VELOCITY)
                        # if difference_of_height_btw_desired_and_actual < -0.05:
                        #     motion_commander.down(-difference_of_height_btw_desired_and_actual, VELOCITY)

                        if is_close(multiranger.front):
                            motion_commander.stop()

                            if switch == "next_is_left" and multiranger.left > multiranger.right and not is_close(multiranger.left):
                                motion_commander.turn_left(90)
                                time.sleep(0.1)
                                if not is_close(multiranger.front):
                                    motion_commander.forward(distance_parallel_to_wall_btw_turns,VELOCITY)
                                    time.sleep(0.1)
                                    motion_commander.turn_left(90)
                                    switch = "next_is_right"
                                else:
                                    motion_commander.turn_left(180)

                            elif switch == "next_is_right" and multiranger.right > multiranger.left and not is_close(multiranger.right):
                                motion_commander.turn_right(90)
                                time.sleep(0.1)
                                if not is_close(multiranger.front):
                                    motion_commander.forward(distance_parallel_to_wall_btw_turns,VELOCITY)
                                    time.sleep(0.1)
                                    motion_commander.turn_right(90)
                                    switch = "next_is_left"
                                else:
                                    motion_commander.turn_right(180)

                            if not is_close(multiranger.front):
                                motion_commander.start_forward(VELOCITY)

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
                    # To avoid: "quadcopter often crashed once the battery level reached below a certain unspecified threshold. "
                    # TODO add a method to check the battery life. If the battery is too low, then the drone has to land before it crashes by itself because of the empty battery.

                    print('Demo terminated!')
