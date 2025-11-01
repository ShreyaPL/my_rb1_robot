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
        rospy.loginfo("Rotate Service READY")

        
        self.yaw = None
        self.prev_yaw = None
        self.busy = False
        self.rate = rospy.Rate(20)

        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)

        #control paraneters
        self.kp = rospy.get_param("kp", 1.2)
        self.max_w = rospy.get_param("max_w", 0.8)
        self.min_w = rospy.get_param("min_w", 0.15)
        self.deadband_deg = rospy.get_param("deadband_deg", 1.0)
        self.slow_radius_deg = rospy.get_param("slow_radius_deg", 10.0)
        self.first_odom_timeout = rospy.get_param("first_odom_timeout", 10.0)
        self.settle_time = rospy.get_param("settle_time", 0.2)

        self.service = rospy.Service("/rotate_robot", Rotate, self.handle_rotate)
    
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
        z = Twist()
        for _ in range(5):
            self.cmd_pub.publish(z)
            self.rate.sleep()
    
    def handle_rotate(self, req):
        if self.busy:
            return RotateResponse("ERROR: service busy; try again")
        
        self.busy = True
        rospy.loginfo("Rotate Service REQUESTED: %d deg", req.degrees)

        try:
            # Wait for first odom
            t0 = rospy.Time.now()
            while not rospy.is_shutdown() and self.yaw is None:
                if (rospy.Time.now() - t0).to_sec() > self.first_odom_timeout:
                    rospy.logwarn("No /odom messages received")
                    return RotateResponse("ERROR: no odom received")
            
            target_delta = math.radians(req.degrees)
            if abs(target_delta) < 1e-3:
                rospy.loginfo("Rotate Service COMPLETED: 0 deg (nothing to do)")
                return RotateResponse("OK: 0 deg requested; nothing to do")

            self.prev_yaw = self.yaw
            turned = 0.0
            tol = math.radians(self.deadband_deg)
            slow_r = math.radians(self.slow_radius_deg)
            timeout = max(3.0, 3.0 * abs(target_delta) / max(self.min_w, 1e-3))
            start = rospy.Time.now()

            while not rospy.is_shutdown():
                # update relative rotation
                dyaw = ang_diff(self.yaw, self.prev_yaw)
                turned += dyaw
                self.prev_yaw = self.yaw
                error = target_delta - turned

                if abs(error) < tol:
                    self.stop()
                    self.rate.sleep(self.settle_time)
                    rospy.loginfo("Rotate Service COMPLETED: %d deg", req.degrees)
                    return RotateResponse("OK: rotation complete")
                
                w_cmd = self.kp * error
                if abs(error) < slow_r:
                    w_cmd *= 0.5
                w_cmd = max(-self.max_w, min(self.max_w, w_cmd))
                if abs(w_cmd) < self.min_w:
                    w_cmd = math.copysign(self.min_w, w_cmd)
                
                cmd = Twist()
                cmd.angular.z = w_cmd
                self.cmd_pub.publish(cmd)

                if (rospy.Time.now() - start).to_sec() > timeout:
                    self.stop()
                    rospy.logwarn("Timeout reached before completion")
                    return RotateResponse("ERROR: timeout while rotating")
                self.rate.sleep()
        finally:
            self.stop()
            self.busy = False


def main():
    rospy.init_node("rotate_service_server")
    node = RotateServer()
    rospy.on_shutdown(node.stop)
    rospy.spin()

if __name__ == "__main__":
    main()