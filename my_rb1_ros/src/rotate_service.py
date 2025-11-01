#! /usr/bin/env python3

import rospy
from my_rb1_ros.srv import Rotate, RotateResponse
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

def ang_wrap(a):
    """
    Wrap angle to [-pi, pi]
    """
    return (a + math.pi)%(2.0*math.pi)-math.pi

def ang_diff(a, b):
    """
    Shortest signed differenrce a-b in [-pi, pi]
    """
    return ang_wrap(a-b)

class RotateServer(object):
    def __init__(self):
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.yaw = None
        self.prev_yaw = None
        self.sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)

        #control paraneters
        self.kp = rospy.get_param("kp", 1.5)
        self.max_w = rospy.get_param("max_w", 1.0)   # rad/s
        self.min_w = rospy.get_param("min_w", 0.2)   # rad/s
        self.rate = rospy.Rate(20)

        rospy.Service("/rotate_robot", Rotate, self.handle_rotate)
    
    def odom_callback(self, msg):
        # quaternion -> yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        if self.yaw is None:
            self.prev_yaw = yaw
        self.yaw = yaw
    
    def stop(self):
        self.cmd_pub.publish(Twist())
    
    def handle_rotate(self, req):
        # Wait for first odom
        t0 = rospy.Time.now()
        while not rospy.is_shutdown() and self.yaw is None:
            if (rospy.Time.now() - t0).to_sec() > 3.0:
                return RotateResponse("ERROR: no odom received")
            self.rate.sleep()
        
        target_delta = math.radians(req.degrees)
        if abs(target_delta) < 1e-6:
            return RotateResponse("OK: 0 deg requested; nothing to do")
        
        # Integrate relative rotation
        self.prev_yaw = self.yaw
        turned = 0.0

        timeout = max(3.0, 23.0 * abs(target_delta)/max(self.min_w, 1e-3))
        start = rospy.Time.now()
        while not rospy.is_shutdown():
            # update relative rotation
            dyaw = ang_diff(self.yaw, self.prev_yaw)
            turned += dyaw
            self.prev_yaw = self.yaw
            
            error = target_delta - turned
            if abs(error) < math.radians(1.0):
                self.stop()
                return RotateResponse("OK: rotation complete")
            
            w_cmd = self.kp * error
            w_cmd = max(-self.max_w, min(self.max_w, w_cmd))
            if abs(w_cmd) < self.min_w:
                w_cmd = math.copysign(self.min_w, w_cmd)
            
            cmd = Twist()
            cmd.angular.z = w_cmd
            self.cmd_pub.publish(cmd)

            if (rospy.Time.now() - start).to_sec() > timeout:
                self.stop()
                return RotateResponse("ERROR: timeout while rotating")
            self.rate.sleep()

def main():
    rospy.init_node("rotate_service_server")
    node = RotateServer()
    rospy.on_shutdown(node.stop)
    rospy.spin()

if __name__ == "__main__":
    main()