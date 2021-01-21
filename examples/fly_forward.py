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




if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    cf = Crazyflie(rw_cache='./cache')
    with SyncCrazyflie(URI, cf=cf) as scf:
        # 1. Takes off (1st step) when the commander is created. Hovers at 1 m
        with MotionCommander(scf,1) as motion_commander:
            with Multiranger(scf) as multiranger:

                motion_commander.move_distance(2, 0, 0)
                motion_commander.land()
                