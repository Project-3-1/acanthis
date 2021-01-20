import numpy as np
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

def wraptopi(number):
    return (number + np.pi) % (2 * np.pi) - np.pi

# to check if the drone is close to an obstacle
def logicIsCloseTo(self, real_value=0.0, checked_value=0.0, margin=0.05):
    if (real_value > checked_value - margin) and (real_value < checked_value + margin):
        return True
    else:
        return False

# Determine whether the drone is closed to an obstacle or not. Let's say the threshold is 0.3
def is_close(range):
    MIN_DISTANCE = 0.3  # meters
    if range is None:
        return False
    else:
        return range < MIN_DISTANCE

# Transition state and restart the timer
def transition(newState):
    state = newState
    state_start_time = time.time()
    return state


if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    cf = Crazyflie(rw_cache='./cache')
    with SyncCrazyflie(URI, cf=cf) as scf:
        # 1. Takes off (1st step) when the commander is created. Hovers at 1 m
        with MotionCommander(scf, 0.4) as motion_commander:
            with Multiranger(scf) as multiranger:

                keep_flying = True
                ranges = ["front", "left", "right", "back"]
                switch = "next_is_right"
                HEIGHT = 0.4  # meter
                VELOCITY = 0.5
                field_of_view = 0.5
                distance_parallel_to_wall_btw_turns = 0.5
                is_parallel_to_first_wall = False
                first_wall_is_found = False
                target_found = False
                start_following_target = False
                ref_distance_from_wall = 0.5
                max_speed = 0.2
                max_rate = 0.2
                state_start_time = 0
                state = "FORWARD"
                previous_heading = 0.0
                sizeOfRoom = 3
                distanceToGoAwayFromWall = sizeOfRoom / 2
                calculate_angle_first_time = True
                around_corner_first_turn = True
                direction = 1
                around_corner_go_back = False

                while keep_flying and not target_found:

                    if is_close(multiranger.up):
                        keep_flying = False

                    ##### handle state transitions #######
    

                    elif state == "FORWARD":
                        # if close enough to the front wall, turn to find it and be in front of it.
                        if is_close(multiranger.front):
                            state = transition("TURN_TO_FIND_WALL")

                    elif state == "HOVER":
                        print(state)

                    elif state == "TURN_TO_FIND_WALL":
                        print("ranges (front, right, left): ", multiranger.front, multiranger.right, multiranger.left)
                        side_range = multiranger.right
                        front_range = multiranger.front
                        # if in front of the wall
                        if (side_range < ref_distance_from_wall / math.cos(0.78) + 0.2 and front_range < ref_distance_from_wall / math.cos(0.78) + 0.2):
                            # previous_heading = current_heading
                            angle = direction * (1.57 - math.atan(front_range / side_range))
                            # time to rotate in order to be parallel to the wall
                            state = transition("TURN_TO_ALLIGN_TO_WALL")
                            print("got angle ", angle)

                        # TODO verify this
                        # if there is no corner closer than 2m and we're next to a wall, it means we should
                        # rotate around the wall
                        # careful to the 2.0 value, we might want to change this value depending on the
                        # size of th walls (room)
                        if (side_range < 1.0 and front_range > 2.0):
                            around_corner_first_turn = True
                            around_corner_go_back = False
                            # previous_heading = current_heading
                            state = transition("ROTATE_AROUND_WALL")

                    # TODO verify this
                    elif state == "TURN_TO_ALLIGN_TO_WALL":
                        # print(current_heading)
                        # print(wraptopi(current_heading - previous_heading), angle)
                        # if logicIsCloseTo(wraptopi(current_heading - previous_heading), angle, 0.1):
                        if is_close(multiranger.right):
                            state = transition("FORWARD_ALONG_WALL")

                    elif state == "FORWARD_ALONG_WALL":
                        # if the drone is too far from the wall, it means that there was an opening or a corner
                        # so it should turn there to explore it.
                        if multiranger.right > 2:
                            around_corner_first_turn = True
                            state = transition("ROTATE_AROUND_WALL")
                        # if it gets close to a wall in front of it, it means that it reached a corner and
                        # it has to rotate to
                        # follow the next wall
                        if front_range < ref_distance_from_wall:
                            state = transition("ROTATE_IN_CORNER")
                            # previous_heading = current_heading
                        # if 3 sec have been elapsed, then it will explore the center area for a it and
                        # then come back to the wall.
                        if state_start_time > 3:
                            state = transition("GET_AWAY_FROM_WALL")

                    # fly perpendicular to the wall to explore the center of the room while not too
                    # far from the wall
                    elif state == "GET_AWAY_FROM_WALL":
                        if multiranger.right >= distanceToGoAwayFromWall:
                            # goes back to the wall to continue the wall following
                            state = transition("GO_BACK_TO_WALL")

                    # TODO Check this
                    # goes back to the wall to continue the wall following
                    elif state == "GO_BACK_TO_WALL":
                        if multiranger.right <= ref_distance_from_wall + 0.1:
                            # if close enough to the wall again, then forward along wall again
                            state = transition("FORWARD_ALONG_WALL")

                    elif state == "ROTATE_AROUND_WALL":
                        if multiranger.front < ref_distance_from_wall + 0.2:
                            # If it's close from a wall (front), then it's time to rotate and place itself
                            # in front of the wall.
                            state = transition("TURN_TO_FIND_WALL")

                    elif state == "ROTATE_IN_CORNER":
                        #  print(current_heading - previous_heading)
                        if is_close(multiranger.right) and not is_close(multiranger.front):
                            state = transition("TURN_TO_FIND_WALL")

                    print(state)



                    ##### handle state ations ########
                    if state == "TAKE_OFF":
                        twist = motion_commander.take_off()

                    elif state == "FORWARD":
                        twist = motion_commander.start_forward()

                    elif state == "HOVER":
                        twist = motion_commander.stop()

                    elif state == "TURN_TO_FIND_WALL":
                        twist = motion_commander.stop()
                        if (time.time() - state_start_time) > 1:
                            twist = motion_commander.start_turn_left(max_rate)

                    elif state == "TURN_TO_ALLIGN_TO_WALL":
                        # hover for 2 sec, perform the calculus and then turn
                        twist = motion_commander.stop()
                        if (time.time() - state_start_time) > 2:
                            twist = motion_commander.start_turn_left(max_rate)

                    elif state == "FORWARD_ALONG_WALL":
                        # twist = twistForwardAlongWall(side_range)
                        twist = motion_commander.start_forward()

                    elif state == "GET_AWAY_FROM_WALL":
                        twist = motion_commander.left(distanceToGoAwayFromWall)

                    elif state == "GO_BACK_TO_WALL":
                        twist = motion_commander.right(distanceToGoAwayFromWall)

                    # elif state == "ROTATE_AROUND_WALL":
                    #     if around_corner_first_turn:
                    #         print("regular_turn_first")
                    #         # if side_range>self.ref_distance_from_wall+0.5 and self.around_corner_first_turn:
                    #         twist = motion_commander.start_turn_left(max_rate)
                    #         if multiranger.right <= ref_distance_from_wall + 0.5:
                    #             around_corner_first_turn = False
                    #             # previous_heading = current_heading
                    #     else:
                    #         if multiranger.right > ref_distance_from_wall + 0.5:
                    #             print("twistTurnandAdjust")
                    #             # twist = self.twistTurnandAdjust(self.max_rate,side_range)
                    #             # if wraptopi(abs(current_heading - previous_heading)) > 0.3:
                    #                 around_corner_go_back = True
                    #             if around_corner_go_back:
                    #                 twist = twistTurnandAdjust(max_rate, side_range)
                    #                 print("go back")
                    #             else:
                    #                 twist = twistTurnandAdjust(-1 * max_rate, side_range)
                    #                 print("forward")
                    #         else:
                    #             print("twistTurnAroundCorner")
                    #             previous_heading = current_heading;
                    #             if around_corner_go_back:
                    #                 twist = twistTurnAroundCorner(ref_distance_from_wall + 0.2, 0.05)
                    #             else:
                    #                 twist = twistTurnAroundCorner(ref_distance_from_wall + 0.2, -0.05)
                    #             previous_heading = current_heading
                    #             around_corner_go_back = False

                    elif state == "ROTATE_IN_CORNER":
                        twist = motion_commander.stop()
                        twist = motion_commander.turn_left(90)

                motion_commander.land()
