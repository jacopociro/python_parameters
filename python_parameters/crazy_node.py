#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
import math
import tf_transformations as tf
import numpy as np
import time 
from crazy_interfaces.msg import Resourcemsg
from crazy_interfaces.srv import Resource

class CrazyNode(Node):
    def __init__(self):
        super().__init__('crazy_node')

        self.wp = {}


        # --------------------
        # Dichiarazione dei parametri
        # --------------------

        # Lista di droni (default vuota)
        self.declare_parameter('drone_name', 'default_drone')
        self.declare_parameter('drone_id', 0)
        self.declare_parameter('pos_init', [0.0, 0.0, 0.0])
        self.declare_parameter('yaw_init', 0.0)

        # Parametri generali di controllo
        self.declare_parameter('control_rate', 10.0)
        self.declare_parameter('max_velocity', 1.0)
        self.declare_parameter('min_velocity', 0.1)
        self.declare_parameter('max_z', 2.0)
        self.declare_parameter('min_z', 0.1)
        self.declare_parameter('linearzbase', 0.01)
        self.declare_parameter('obstacle_topic', '/obstacle/pose')
        self.declare_parameter('drone_names', ['default_drone'])
        self.declare_parameter('wp_gvalue', 1.0)
        self.declare_parameter('cs_gvalue', 1.0)
        self.declare_parameter('obs_gvalue', 1.0)
        self.declare_parameter('drones_gvalue', 1.0)
        # Waypoints e stazioni di ricarica
        self.declare_parameter('wp.a', [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('wp.b', [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('wp.c', [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('wp.d', [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('wp.e', [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('wp.f', [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('charging_stations', [0.0, 0.0, 0.0, 0.0])

        
        # --------------------
        # Lettura dei parametri
        # --------------------
        self.drone_name = self.get_parameter('drone_name').value
        self.drone_id = self.get_parameter('drone_id').value
        pos_init = self.get_parameter('pos_init').value
        if len(pos_init) == 3:
            self.x_init, self.y_init, self.z_init = pos_init
            self.x, self.y, self.z = pos_init
        else:
            self.get_logger().warn("Parameter 'pos_init' should be a list of 3 values (x, y, z). Using default (0.0, 0.0, 0.0).")
            self.x_init, self.y_init, self.z_init = 0.0, 0.0, 0.0
        self.yaw_init = self.yaw = self.get_parameter('yaw_init').value


        self.control_rate = self.get_parameter('control_rate').value
        self.max_z = self.get_parameter('max_z').value
        self.min_z = self.get_parameter('min_z').value
        self.linearzbase = self.get_parameter('linearzbase').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.min_velocity = self.get_parameter('min_velocity').value
        self.obstacle_topic = self.get_parameter('obstacle_topic').value
        self.drone_names = self.get_parameter('drone_names').value

        self.wp['a'] = self.get_parameter('wp.a').value
        self.wp['b'] = self.get_parameter('wp.b').value
        self.wp['c'] = self.get_parameter('wp.c').value
        self.wp['d'] = self.get_parameter('wp.d').value
        self.wp['e'] = self.get_parameter('wp.e').value
        self.wp['f'] = self.get_parameter('wp.f').value
        self.charging_stations = self.get_parameter('charging_stations').value

        # Stampa per debug
        self.get_logger().info("=== Parameters ===")

        self.get_logger().info(f"Control rate: {self.control_rate}")
        self.get_logger().info(f"Waypoints: {self.wp}")
        self.get_logger().info(f"Charging stations: {self.charging_stations}")

        # Logger droni
        self.get_logger().info("=== Drones Initial Positions ===")

        self.get_logger().info(
            f"Drone {self.drone_name} (ID: {self.drone_id}) - Initial Position: x={self.x_init}, y={self.y_init}, z={self.z_init}, yaw={self.yaw_init}"
        )

        # Logger waypoints
        self.get_logger().info("=== Waypoints ===")
        for wp in self.wp:
            self.get_logger().info(
                #f"x={wp[0]}, y={wp[1]}, z={wp[2]}, yaw={wp[3]}"
                f"{wp}: x={self.wp[wp][0]}, y={self.wp[wp][1]}, z={self.wp[wp][2]}, yaw={self.wp[wp][3]}"
            )

        # Logger stazioni di ricarica
        self.get_logger().info("=== Charging Stations ===")

        self.get_logger().info(
            f"x={self.charging_stations[0]}, y={self.charging_stations[1]}, z={self.charging_stations[2]}, yaw={self.charging_stations[3]}"
        )
        # Publisher su /cmd_vel
        self.cmd_vel_publisher = self.create_publisher(TwistStamped, f'{self.drone_name}/cmd_vel', 10)
        # # Publisher su /leader dovrà diventare custom message con memoria 
        # self.leader_publisher = self.create_publisher(PoseStamped, f'{self.drone_name}/leader', 10)
        # Publisher su /goal
        self.goal_publisher = self.create_publisher(PoseStamped, f'{self.drone_name}/goal', 10)
        # Publisher su /odom
        self.pose_publisher = self.create_publisher(PoseStamped, f'{self.drone_name}/odom', 10)
        sub_pose_list = []
        # Subscriber 1 su /pose
        sub_self = self.create_subscription(
            PoseStamped,
            f'{self.drone_name}/pose',
            self.pose_callback,
            10,
        )
        for name in self.drone_names:
            if self.drone_name == name:
                continue
            else:
                nombre = name
        self.get_logger().info(f'{nombre}')
        sub_other = self.create_subscription(
            PoseStamped,
            f'{nombre}/odom',
            self.pose_callback_1,
            10,
        )
        # for name in self.drone_names:
        #     sub = self.create_subscription(
        #         PoseStamped,
        #         f'{name}/pose',
        #         lambda msg: self.pose_callback_1(msg, name),
        #         10,
        #     )
        #     self.name = name
        #     sub_pose_list.append(sub)
            
            
        # Subscriber 2 su /pose
        self.pose_subscriber_obs = self.create_subscription(
            PoseStamped,
            '/obstacle/pose',
            self.pose_callback_2,
            10
        )

        # Service server su /resource
        self.resource_service = self.create_service(
            Resource,
            f'{self.drone_name}/resource',
            self.handle_service
        )
        
        self.sensing_ = self.sphere()
        self.Memory = np.zeros(len(self.wp) + 1)
        self.Memory_temp = np.zeros(len(self.wp) + 1)
        self.Mem_leader = np.zeros(len(self.wp) + 1)
        self.split = 0.95
        self.Priority = np.zeros(len(self.wp) + 1)
        self.Perc = np.zeros(len(self.wp))
        self.wp_gvalue = self.get_parameter('wp_gvalue').value
        self.drones_gvalue =  self.get_parameter('drones_gvalue').value
        self.cs_gvalue = self.get_parameter('cs_gvalue').value
        self.obs_gvalue = self.get_parameter('obs_gvalue').value
        self.uptake = [False] * len(self.wp)
        self.vec_pos = []
        self.obstacle = []
        self.roll = 0.0
        self.pitch = 0.0
        self.odom = False
        for name in self.drone_names:
            self.vec_pos.append((0.0, 0.0, 0.0, 0.0, 0.0, 0.0))

        time.sleep(2.0)
        # hz 20, 50 parametrizzo
        self.timer = self.create_timer(1.0/self.control_rate, self.timer_callback)

    def timer_callback(self):
        #self.get_logger().info(f"x: {self.x}, x_init: {self.x_init}")
        T = self.compute_transformation_matrix()
        #self.get_logger().info(f"Transformation matrix T: {T}")
        self.sensing = np.zeros((len(self.sensing_), 4))
        for i in range(len(self.sensing)):
            self.sensing[i] = self.sensing_[i] + T[:, 3]
            self.sensing[i][3] = 1.0
            #self.get_logger().info(f"sensing: {self.sensing[i]} {i}")

        self.Perc1 = np.zeros(len(self.sensing))
        waypoint_pos = []
        cs_pos = []
        vec = np.zeros((len(self.sensing), 4))
        i = 0
        if self.Priority[len(self.Priority) - 1] < 0.7:
            for waypoint in self.wp:
                if self.wp[waypoint][0] == 0.0 and self.wp[waypoint][1] == 0.0 and self.wp[waypoint][2] == 0.0:
                    continue
                else:
                    waypoint_pos.append(np.array([self.wp[waypoint][0], self.wp[waypoint][1], self.wp[waypoint][2], 1]))
                    if np.linalg.norm(waypoint_pos[i][:3] - np.array([self.x, self.y, self.z])) < 1.0:
                        self.uptake[i] = True
                    else:
                        self.uptake[i] = False

                    #self.get_logger().info(f"Waypoint {waypoint} position: {waypoint_pos[i]} - distance: {np.linalg.norm(waypoint_pos[i][:3] - np.array([self.x, self.y, self.z]))} - waypoint data: {self.wp[waypoint]}")
                    self.Perc[i] = self.gaussian(np.linalg.norm(waypoint_pos[i][:3] - np.array([self.x, self.y, self.z])), self.wp_gvalue, 1.0)
                    self.Priority[i] = self.biological_calculation(i, self.Perc[i], self.uptake[i])
                    for j in range(len(self.sensing)):
                        self.Perc1[j] += self.Priority[i]*self.gaussian(np.linalg.norm(waypoint_pos[i][:3] - np.array([self.sensing[j][:3]])), self.wp_gvalue, 1.0)
                        #pass
                i = i + 1
            
        cs_pos.append(np.array([self.charging_stations[0],self.charging_stations[1], self.charging_stations[2], 1]))
        for j in range(len(self.sensing)):
            self.Perc1[j] += self.Priority[len(self.Priority) - 1]*self.gaussian(np.linalg.norm(cs_pos[0][:3] - np.array([self.sensing[j][:3]])), self.cs_gvalue, 1.0)
            #pass

        for i in range(len(self.drone_names)):
                for j in range(len(self.sensing)):
                    self.Perc1[j] += self.gaussian(np.linalg.norm(self.vec_pos[i][:3] - np.array([self.sensing[j][:3]])), self.drones_gvalue, -1.0)
                    #pass
                self.get_logger().info(f'self pos: {self.x, self.y, self.z} other drone pos: {self.vec_pos}')
            
        #self.get_logger().info(f"obstacles list {self.obstacle}")
        for obstacle in self.obstacle:
            #self.get_logger().info(f"Obstacle: {obstacle}")
            for j in range(len(self.sensing)):
                self.Perc1[j] += self.gaussian(np.linalg.norm(obstacle - np.array([self.sensing[j][:3]])), self.obs_gvalue, -1.0)
                pass
        for i in range(len(self.sensing)):
            #self.get_logger().info(f"Perc1 for sensing point {i}: {self.Perc1[i]}")
            vec[i] = (self.sensing[i] - np.array([self.x, self.y, self.z,1]))*self.Perc1[i]
        #self.get_logger().info(f"vettori {vec}")
        sum_vec = np.sum(vec, axis=0)
        sum_vec[3] = 0.0

        # Logica per il movimento del drone
        # speed 0.001 m/s min 0.1 m/s max
        norm = np.linalg.norm(sum_vec)
        
        scale = 1.0
        self.get_logger().info(f"Norm of sum_vec: {norm}")
        if norm != 0.0 and norm > self.max_velocity:
            scale = self.max_velocity / norm
        # elif norm != 0.0 and norm < self.min_velocity:
        #     scale = self.min_velocity / norm
        sum_vec = sum_vec*scale*self.control_rate
        norm = np.linalg.norm(sum_vec)
        
        if self.z >= self.max_z:
            sum_vec[2] = 0.0
        if self.z <= self.min_z and sum_vec[2] < 0.0:
            sum_vec[2] = 0.0 
            
        cmd_vel_msg = TwistStamped()
        cmd_vel_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_vel_msg.header.frame_id = 'map'
        cmd_vel_msg.twist.linear.x = sum_vec[0]  # Velocità lineare in x
        cmd_vel_msg.twist.linear.y = sum_vec[1]  # Velocità lineare in y
        cmd_vel_msg.twist.linear.z = sum_vec[2] + self.linearzbase  # Velocità lineare in z
        cmd_vel_msg.twist.angular.x = 0.0  # Velocità angolare in x
        cmd_vel_msg.twist.angular.y = 0.0  # Velocità angolare in y
        cmd_vel_msg.twist.angular.z = 0.0  # Velocità angolare in z

        self.cmd_vel_publisher.publish(cmd_vel_msg)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = sum_vec[0] + self.x
        pose_msg.pose.position.y = sum_vec[1] + self.y
        pose_msg.pose.position.z = sum_vec[2] + self.z
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 0.0
        self.goal_publisher.publish(pose_msg)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = self.x
        pose_msg.pose.position.y = self.y
        pose_msg.pose.position.z = self.z
        pose_msg.pose.orientation.x = tf.quaternion_from_euler(self.roll, self.pitch, self.yaw)[0]
        pose_msg.pose.orientation.y = tf.quaternion_from_euler(self.roll, self.pitch, self.yaw)[1]
        pose_msg.pose.orientation.z = tf.quaternion_from_euler(self.roll, self.pitch, self.yaw)[2]
        pose_msg.pose.orientation.w = tf.quaternion_from_euler(self.roll, self.pitch, self.yaw)[3]
        self.pose_publisher.publish(pose_msg)
        if self.odom == False:
            self.x += sum_vec[0]
            self.y += sum_vec[1]
            self.z += sum_vec[2]

        self.Memory += self.Memory_temp
        for i in range(len(self.Memory - 1)):
            if self.Memory[i] > 0.0005253:
                self.Memory[i] = 0.0005253
        self.obstacle = []
        #Logger per i valori di Priority
        self.get_logger().info("=== Priority Values ===")
        for i, priority_value in enumerate(self.Priority):
            self.get_logger().info(f"Priority[{i}]: {priority_value:.4f}")

        # Logger per i valori di Memory
        # self.get_logger().info("=== Memory Values ===")
        # for i, memory_value in enumerate(self.Memory):
        #     self.get_logger().info(f"Memory[{i}]: {memory_value:.4f}")

    def handle_service(self, request, response):
        self.Memory[0] += request.memory.mem1
        self.Memory[1] += request.memory.mem2
        self.Memory[2] += request.memory.mem3
        self.Memory[3] += request.memory.mem4
        self.Memory[4] += request.memory.mem5
        self.Memory[5] += request.memory.mem6

        msg = Resourcemsg() 
        msg.mem1 = self.Mem_leader[0]
        msg.mem2 = self.Mem_leader[1]
        msg.mem3 = self.Mem_leader[2]
        msg.mem4 = self.Mem_leader[3]
        msg.mem5 = self.Mem_leader[4]
        msg.mem6 = self.Mem_leader[5]

        response.memory = msg
        if any( i == 0.0 for i in self.Mem_leader):
            response.help = True
        else:
            response.help = False

        
        return response
    
    def pose_callback(self, msg):
            
            x = msg.pose.position.x
            y = msg.pose.position.y
            z = msg.pose.position.z
            roll, pitch, yaw = tf.euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])

            self.vec_pos[0] = (x, y, z, roll, pitch, yaw)
            if not msg.pose.position:
                self.odom = False
            else:
                self.odom = True
                self.x = x
                self.y = y
                self.z = z
                self.roll = roll
                self.pitch = pitch
                self.yaw = yaw



    def pose_callback_1(self, msg):
            
            x = msg.pose.position.x
            y = msg.pose.position.y
            z = msg.pose.position.z
            roll, pitch, yaw = tf.euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])

            self.vec_pos[1] = (x, y, z, roll, pitch, yaw)
            self.get_logger().info(f'{self.vec_pos}')

            
            
                
                


    def pose_callback_2(self, msg):
        
        # if np.any(np.all(self.obstacle == np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]))):
        #     pass
        # else:
        self.obstacle.append([msg.pose.position.x , msg.pose.position.y, msg.pose.position.z])


    # UTILS
    def sphere(self):
        # Angoli
        polar_angle = [
            0.0,
            math.pi/2.0,
            -math.pi/2.0,
            math.pi,
            0.0,
            math.pi,
            math.pi/4,
            -math.pi/4,
            3*math.pi/4,
            5*math.pi/4,
            math.pi/4,
            -math.pi/4,
            3*math.pi/4,
            5*math.pi/4
        ]

        azimuth_angle = [
            0.0,
            0.0,
            0.0,
            0.0,
            math.pi/2.0,
            -math.pi/2.0,
            math.pi/4,
            math.pi/4,
            math.pi/4,
            math.pi/4,
            -math.pi/4,
            -math.pi/4,
            -math.pi/4,
            -math.pi/4
        ]
        radius = 0.1
        sensing_list = np.zeros((len(polar_angle), 4))
        for i in range(len(polar_angle)):

            x_ = radius * math.sin(polar_angle[i]) * math.cos(azimuth_angle[i])
            y_ = radius * math.sin(polar_angle[i]) * math.sin(azimuth_angle[i])
            z_ = radius * math.cos(polar_angle[i])

            vec_out = np.array([x_, y_, z_])

            sensing_list[i, 0] = vec_out[0] + self.x_init
            sensing_list[i, 1] = vec_out[1] + self.y_init
            sensing_list[i, 2] = vec_out[2] + self.z_init
            sensing_list[i, 3] = 1.0

        
        return sensing_list

    def biological_calculation(self, counter: int, perc_value: float, uptake: bool) -> float:
        TF_Imax = -6.992e-3 * self.Memory[counter] + 3.672e-6
        TF_Kmax = -8.4e-4 * self.Memory[counter] + 61.0

        P = 1.1 * TF_Imax**2 / (36.72e-7)**2 - 0.1
        I = TF_Imax * perc_value / (TF_Kmax + perc_value) * 50

        if uptake:
            self.Memory_temp[counter] = I * self.split
            self.Mem_leader[counter] = I - I * self.split
        # if counter == 1:
        #     P = 1
        # else:
        #     P = 0
        # Logger per il valore di I
        # self.get_logger().info(f"Biological Calculation - Counter: {counter}, Perc Value: {perc_value}, Uptake: {uptake}, I: {I:.4f}")

        return P

    def compute_transformation_matrix(self):
        """
        Calcola la matrice di trasformazione omogenea 4x4
        che porta dal frame globale iniziale al frame corrente.
        """

        # Differenze
        dx = self.x - self.x_init
        dy = self.y - self.y_init
        dz = self.z - self.z_init
        dyaw = self.yaw - self.yaw_init

        # Matrice di rotazione attorno a Z
        R = np.array([
            [math.cos(dyaw), -math.sin(dyaw), 0.0],
            [math.sin(dyaw),  math.cos(dyaw), 0.0],
            [0.0,             0.0,            1.0]
        ])

        # Matrice omogenea 4x4
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = np.array([dx, dy, dz])

        return T

    def gaussian(self, x, sigma, A):
        mu = 0.0
        return A * math.exp(-((x - mu)**2) / (2 * sigma**2))
def main(args=None):
    rclpy.init(args=args)

    node = CrazyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()