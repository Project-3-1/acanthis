import cflib.crtp
import rospy
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

URI = 'radio://0/80/2M'

from acanthis.msg import CmdVelocity

motion_commander = None


def callback(data):
    motion_commander.start_linear_motion(
        data.velocity.x, data.velocity.x, data.velocity.z)


def listener():
    global motion_commander
    rospy.init_node('cmd_vel')
    rospy.Subscriber("cmd_velocity_sub", CmdVelocity, callback)

    cflib.crtp.init_drivers(enable_debug_driver=False)

    cf = Crazyflie(rw_cache='./cache')
    with SyncCrazyflie(URI, cf=cf) as scf:
        with MotionCommander(scf) as m_c:
            motion_commander = m_c

    rospy.spin()
