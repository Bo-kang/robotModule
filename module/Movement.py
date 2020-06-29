import rospy  # import order is important
from geometry_msgs.msg import Twist
import time
import math
def shutdown():
    cmd_vel.publish(Twist())  # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
    rospy.sleep(1)  # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script

rospy.init_node('GoForward', anonymous=False)  # initialize
cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist,
                          queue_size=10)  # Create a publisher which can "talk" to TurtleBot and tell it to move
rospy.on_shutdown(shutdown)
move_cmd = Twist()



def move():
    speed = 0.2
    move_cmd.linear.x = speed
    cmd_vel.publish(move_cmd)
    time.sleep(0.5)

def stop():
    move_cmd.linear.x = 0
    cmd_vel.publish(move_cmd)

def rotate3(angle):
    relative_angle = abs(angle * math.pi / 180)

    current_angle = 0

    angular_acceleration = 0.005

    while current_angle < relative_angle :                                              # angular vel == 0.6 or -0.6 setting (- : CW, + : CCW)
        if move_cmd.linear.x - 0.001 <= 0:
            move_cmd.linear.x = 0
        move_cmd.linear.x -= 0.001
        t0 = rospy.Time.now().to_sec()
        if abs(move_cmd.angular.z) < 0.6:  # maximum angle velocity
            if angle > 0:
                move_cmd.angular.z += -angular_acceleration  # accelerate angle velocity
            else:
                move_cmd.angular.z += angular_acceleration  # accelerate angle velocity
        else:
            if angle > 0:
                move_cmd.angular.z = -0.6  # maximum angle velocity
            else:
                move_cmd.angular.z = 0.6  # maximum angle velocity
        cmd_vel.publish(move_cmd)
        time.sleep(0.01)
        t1 = rospy.Time.now().to_sec()
        current_angle += abs(move_cmd.angular.z) * (t1 - t0)


    # 2) rotate more for smooth movement
    while abs(move_cmd.angular.z) > angular_acceleration:                                                   # slow down angular vel to angular_accel(0.0025)
        if angle > 0:
            move_cmd.angular.z += angular_acceleration
        else:
            move_cmd.angular.z += -angular_acceleration
        cmd_vel.publish(move_cmd)
        time.sleep(0.01)

    # Forcing our robot to stop
    move_cmd.linear.x = 0
    move_cmd.angular.z = 0
    cmd_vel.publish(move_cmd)

