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

current = 0
while True :
    t0 = rospy.Time.now().to_sec()
    mov_cmd = Twist()
    angle = math.pi*0.15
    mov_cmd.angular.z = angle
    cmd_vel.publish(mov_cmd)
    #time.sleep(1)
    t1 = rospy.Time.now().to_sec()

    current = current + angle * (t1-t0)
    print(current)
    if(current >= math.pi/2): break;

