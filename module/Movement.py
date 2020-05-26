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
mov_cmd = Twist()

def rotate(angle):
    speed = math.pi/4
    if angle < 0 : speed *= -1
    mov_cmd.angular.z = speed
    angle = abs(angle)
    current = 0
    while True:
        t0 = rospy.Time.now().to_sec()
        cmd_vel.publish(mov_cmd)
        t1 = rospy.Time.now().to_sec()
        current += abs(speed * (t1-t0))
        #print(current/math.pi)
        #time.sleep(0.01)
        if(current >= angle): break

def rotate2(angle):
    speed = math.pi/4
    if angle < 0 : speed *= -1
    mov_cmd.angular.z = speed
    angle = abs(angle)
    current = 0
    t0 = rospy.Time.now().to_sec()
    while True:
        cmd_vel.publish(mov_cmd)
        t1 = rospy.Time.now().to_sec()
        if(t1 - t0 >= 1) : break


def move(distance):
    speed = 0.2
    mov_cmd.linear.x = speed
    target = distance / speed
    t0 = rospy.Time.now().to_sec()
    while True:
        cmd_vel.publish(mov_cmd)
        t1 = rospy.Time.now().to_sec()
        print(t1, end=" ")
        print(t0, end = " ")
        print(t1 - t0)
        if(t1 - t0 >= target): break






