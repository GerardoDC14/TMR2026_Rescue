#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


#Checar el tipo de mensajes
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, PoseStamped
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32MultiArray

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

import numpy as np
import math
import tf_transformations

from collections import deque


class EkfOdomNode(Node):
    def __init__(self):
        super().__init__('ekf_odom_node')

        # --- Subscriptions ---
        self.create_subscription(Imu, '/sensors/imu', self.imu_callback, 10)
        self.create_subscription(Vector3, '/encoders/tracks', self.velocity_callback, 10)
        #self.create_subscription(Float32MultiArray, '/enconders/flipper')

        # --- Publishers & TF ---
        self.odom_pub     = self.create_publisher(Odometry, '/odom',      10)
        self.path_pub     = self.create_publisher(Path,     '/odom_path', 10)
        self.marker_pub   = self.create_publisher(Marker,   '/pose_marker', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- Path init ---
        self.path = Path()
        self.path.header.frame_id = 'odom'
        
        # --- EMA Parameters ---
        self.yaw_buffer = deque(maxlen=5)
        self.omega_buffer = deque(maxlen=5)
        self.ema_var_omega = None
        self.ema_var_yaw = None
        self.alpha = 0.1

        # --- EKF state ---
        self.x_est = np.zeros((3, 1))  # [x, y, yaw]
        self.P     = np.eye(3) * 1e-3

        # --- Noise matrices ---
        self.Q     = np.diag([0.05, 0.05, 0.05])
        self.R_yaw = np.array([[0.02]]) #TUNE THISSS
        self.R_omega = np.array([[0.01]]) #TUNE THISSS

        # --- Inputs / measurements ---
        self.vx       = 0.0
        self.vy       = 0.0
        self.omega    = 0.0
        self.yaw_meas = None
        self.v = 0.0
        self.omega_enc = 0.0  #computed value
        self.w_x = 0.0
        self.w_y = 0.0
        self.omega_imu = 0.0  

        #Physical properties (In meters)
        self.Trackwidth = 0.47 #Jaguar
        self.Radius     = 0.087 
        #self.Trackwidth = ## Dicerox MEDIR
        #self.Radius  = ###



        # --- Timing ---
        self.last_time = self.get_clock().now()
        self.create_timer(1.0 / 50.0, self.timer_callback)

    def velocity_callback(self, msg: Vector3):
        rpm_x = msg.x #left motor
        rpm_y = msg.y #right motor rpms

        self.w_x = (rpm_x * 2 * np.pi) / 60 #rpms to rad/s left
        self.w_y = (rpm_y * 2 * np.pi) / 60 #rpms to rad/s right

        self.v_x = (self.w_x* self.Radius) #Band radius
        self.v_y = (self.w_y* self.Radius)

        self.v = (self.v_x + self.v_y) / 2
        self.omega_enc = (self.v_y - self.v_x) / self.Trackwidth



    def imu_callback(self, msg: Imu):
        q = [msg.orientation.x,
             msg.orientation.y,
             msg.orientation.z,
             msg.orientation.w]
        _, _, yaw = tf_transformations.euler_from_quaternion(q)
        self.yaw_meas = yaw #Obtain orientation
        self.omega_imu    = msg.angular_velocity.z #OBtain rotation speed
        
        #Obtain the yaw variance (direction)
        imu_orientation_var = msg.orientation_covariance[8]
        if imu_orientation_var > 0:
            self.R_yaw = np.array([[imu_orientation_var]])
        else: 
            self.yaw_buffer.append(yaw)
            if len(self.yaw_buffer) == self.yaw_buffer.maxlen:
                var_yaw = np.var(self.yaw_buffer)
            
                if self.ema_var_yaw is None: 
                    self.ema_var_yaw = var_yaw
                else: 
                    self.ema_var_yaw = self.alpha * var_yaw + (1 - self.alpha) * self.ema_var_yaw
                self.R_yaw = np.array([[max(self.ema_var_yaw, 1e-4)]])
            else: 
                self.R_yaw = np.array([[0.02]])
        
        #obtain the angular velocity variance
        imu_omega_var = msg.angular_velocity_covariance[8]
        if imu_omega_var > 0:
            self.R_omega = np.array([[imu_omega_var]])
        else: 
            self.omega_buffer.append(self.omega_imu)
            if len(self.omega_buffer) == self.omega_buffer.maxlen:
                var_omega = np.var(self.omega_buffer)
            
                if self.ema_var_omega is None: 
                    self.ema_var_omega = var_omega
                else: 
                    self.ema_var_omega = self.alpha * var_omega + (1 - self.alpha) * self.ema_var_omega
                self.R_omega = np.array([[max(self.ema_var_omega, 1e-4)]])
            else: 
                self.R_omega = np.array([[0.01]])
        
             
    
    def timer_callback(self):
        now_time = self.get_clock().now()
        dt = (now_time - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return
        self.last_time = now_time

        #Fuse imu and encoder angular velocity
        var_enc = 0.05  #TUNE THIS AAAA

        if self.ema_var_omega is not None:
            var_imu = self.ema_var_omega
        else:
            var_imu = 0.01  # fallback

        # Avoid dividing with 0
        var_imu = max(var_imu, 1e-6)
        var_enc = max(var_enc, 1e-6)
            
        w_imu = 1.0 / var_imu
        w_enc = 1.0 / var_enc
        self.omega = (w_imu * self.omega_imu + w_enc * self.omega_enc) / (w_imu + w_enc) #Fused angular velocity

        #EKF 
        x, y, yaw = self.x_est.flatten()
        v, w = self.v, self.omega

        if abs(w) > 1e-6:
            dx = v / w * (math.sin(yaw + w*dt) - math.sin(yaw))
            dy = v / w * (-math.cos(yaw + w*dt) + math.cos(yaw))
            F = np.eye(3)
            F[0,2] = v/w * (math.cos(yaw + w*dt) - math.cos(yaw))
            F[1,2] = v/w * (math.sin(yaw + w*dt) - math.sin(yaw))
            
        else:
            dx = v * dt * math.cos(yaw)
            dy = v * dt * math.sin(yaw)
            
            F = np.eye(3)
            F[0,2] = -v * dt * math.sin(yaw)
            F[1,2] = v * dt * math.cos(yaw)
        dyaw = w * dt

        yaw_pred = math.atan2(math.sin(yaw + dyaw), math.cos(yaw + dyaw))
        self.x_est = np.array([[x + dx],
                               [y + dy],
                               [yaw_pred]])

        #F = np.eye(3)
        #F[0, 2] = -v * dt * math.sin(yaw)
        #F[1, 2] =  v * dt * math.cos(yaw)
        B = np.array([[dt * math.cos(yaw), 0],
                      [dt * math.sin(yaw), 0],
                      [0, dt]])
        #self.P = F @ self.P @ F.T + B @ np.diag([0.1, 0]) @ B.T + self.Q
        var_v = 0.1**2             # tune this Linear velocity variance (tune for enocder accuracy)
        var_w = var_imu         # from IMU (dynamic)
        Q_control = np.diag([var_v, var_w])
        self.P = F @ self.P @ F.T + B @ Q_control @ B.T + self.Q

        if self.yaw_meas is not None:
            H = np.array([[0, 0, 1]])
            z = np.array([[self.yaw_meas]])
            y_res = z - np.array([[yaw_pred]])
            y_res[0] = math.atan2(math.sin(y_res[0]), math.cos(y_res[0]))
            S = H @ self.P @ H.T + self.R_yaw
            K = self.P @ H.T @ np.linalg.inv(S)
            self.x_est += K @ y_res
            self.x_est[2] = math.atan2(math.sin(self.x_est[2]), math.cos(self.x_est[2])) #NOrmalize yaw 
            self.P = (np.eye(3) - K @ H) @ self.P

        now_msg = now_time.to_msg()
        self.publish_all(now_msg)

    def publish_all(self, now):
        q = tf_transformations.quaternion_from_euler(0, 0, float(self.x_est[2]))

        # ----- 1. TF -----
        tf_msg = TransformStamped()
        tf_msg.header.stamp = now
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_footprint'
        tf_msg.transform.translation.x = float(self.x_est[0])
        tf_msg.transform.translation.y = float(self.x_est[1])
        tf_msg.transform.rotation.x = q[0]
        tf_msg.transform.rotation.y = q[1]
        tf_msg.transform.rotation.z = q[2]
        tf_msg.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(tf_msg)

        # ----- 2. Odometry -----
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = tf_msg.transform.translation.x
        odom.pose.pose.position.y = tf_msg.transform.translation.y
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x  = self.v
        odom.twist.twist.angular.z = self.omega
        odom.pose.covariance[0]  = float(self.P[0, 0])   # x
        odom.pose.covariance[7]  = float(self.P[1, 1])   # y
        odom.pose.covariance[35] = float(self.P[2, 2])   # yaw
        self.odom_pub.publish(odom)

        # ----- 3. Path -----
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = now
        pose_stamped.header.frame_id = 'odom'
        pose_stamped.pose = odom.pose.pose
        self.path.header.stamp = now
        self.path.poses.append(pose_stamped)
        self.path_pub.publish(self.path)

        # ----- 4. Marker -----
        marker = Marker()
        marker.header.stamp = now
        marker.header.frame_id = 'odom'
        marker.ns = 'robot_heading'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = odom.pose.pose
        marker.scale.x = 0.3
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.r = 1.0
        marker.color.a = 1.0
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = EkfOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()