#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from crazyflie_interfaces.msg import Status  # sostituisci con il tipo corretto del tuo Status

import matplotlib.pyplot as plt
import csv


class MultiDronePlotter(Node):

    def __init__(self):
        super().__init__('multi_drone_plotter')

        # -----------------------------
        # PARAMETERS
        # -----------------------------

        self.declare_parameter("control_rate", 10.0)
        self.declare_parameter("drone_names", ["cf_0"])

        self.declare_parameter('wp.a', [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('wp.b', [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('wp.c', [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('wp.d', [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('wp.e', [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('wp.f', [0.0, 0.0, 0.0, 0.0])

        self.control_rate = self.get_parameter("control_rate").value
        self.drone_names = self.get_parameter("drone_names").value

        # -----------------------------
        # WAYPOINT PARAMETERS
        # -----------------------------

        self.wp = {}
        self.wp['a'] = self.get_parameter('wp.a').value
        self.wp['b'] = self.get_parameter('wp.b').value
        self.wp['c'] = self.get_parameter('wp.c').value
        self.wp['d'] = self.get_parameter('wp.d').value
        self.wp['e'] = self.get_parameter('wp.e').value
        self.wp['f'] = self.get_parameter('wp.f').value

        # -----------------------------
        # DRONE DATA STRUCTURE
        # -----------------------------

        self.positions = {}
        self.battery = {}
        self.subscribers = {}
        self.sub_status = {}

        for name in self.drone_names:
            self.positions[name] = {"x": [], "y": [], "z": []}
            self.battery[name] = []

        # -----------------------------
        # DRONE POSE SUBSCRIBERS
        # -----------------------------

        for name in self.drone_names:
            topic = f"/{name}/odom"
            self.subscribers[name] = self.create_subscription(
                PoseStamped,
                topic,
                self.create_pose_callback(name),
                10
            )

        # -----------------------------
        # DRONE STATUS SUBSCRIBERS (BATTERY)
        # -----------------------------

        for name in self.drone_names:
            topic = f"/{name}/status"
            self.sub_status[name] = self.create_subscription(
                Status,
                topic,
                self.create_status_callback(name),
                10
            )

        # -----------------------------
        # WAYPOINTS
        # -----------------------------

        self.waypoints = []
        for key, values in self.wp.items():
            x = values[0]
            y = values[1]
            z = values[2]
            self.waypoints.append((x, y, z))

        # -----------------------------
        # OBSTACLE SUBSCRIBER
        # -----------------------------

        self.obstacles = []
        self.obstacle_sub = self.create_subscription(
            PoseStamped,
            "/obstacle/pose",
            self.obstacle_callback,
            10
        )

        # -----------------------------
        # CSV LOGGING
        # -----------------------------

        self.csv_file = open("drones_positions.csv", "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)

        header = ["time"]
        for name in self.drone_names:
            header += [f"{name}_x", f"{name}_y", f"{name}_z"]
        self.csv_writer.writerow(header)

        # CSV per battery
        self.csv_bat_file = open("drones_battery.csv", "w", newline="")
        self.csv_bat_writer = csv.writer(self.csv_bat_file)
        header_bat = ["time"]
        for name in self.drone_names:
            header_bat.append(f"{name}_battery")
        self.csv_bat_writer.writerow(header_bat)

        # -----------------------------
        # PLOT SETUP
        # -----------------------------

        self.fig = plt.figure(figsize=(15, 5))
        self.ax3d = self.fig.add_subplot(131, projection='3d')
        self.ax2d = self.fig.add_subplot(132)
        self.ax_bat = self.fig.add_subplot(133)

        plt.show(block=False)

        # -----------------------------
        # TIMER
        # -----------------------------

        self.timer = self.create_timer(
            1/self.control_rate,
            self.update_plot
        )

        self.get_logger().info("Multi drone plotter started")

    # ---------------------------------
    # CALLBACK FACTORY POSE
    # ---------------------------------

    def create_pose_callback(self, drone_name):
        def callback(msg):
            x = msg.pose.position.x
            y = msg.pose.position.y
            z = msg.pose.position.z
            self.positions[drone_name]["x"].append(x)
            self.positions[drone_name]["y"].append(y)
            self.positions[drone_name]["z"].append(z)
            self.write_csv()
        return callback

    # ---------------------------------
    # CALLBACK FACTORY STATUS
    # ---------------------------------

    def create_status_callback(self, drone_name):
        def callback(msg):
            # assumiamo msg.battery contiene lo stato della batteria in percentuale
            self.battery[drone_name].append(msg.battery)
            self.write_battery_csv()
        return callback

    # ---------------------------------
    # OBSTACLE CALLBACK
    # ---------------------------------

    def obstacle_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        obstacle = (x, y, z)
        if obstacle not in self.obstacles:
            self.obstacles.append(obstacle)

    # ---------------------------------
    # CSV LOGGING POSIZIONI
    # ---------------------------------

    def write_csv(self):
        t = self.get_clock().now().nanoseconds * 1e-9
        row = [t]
        for name in self.drone_names:
            if len(self.positions[name]["x"]) > 0:
                row += [self.positions[name]["x"][-1],
                        self.positions[name]["y"][-1],
                        self.positions[name]["z"][-1]]
            else:
                row += [None, None, None]
        self.csv_writer.writerow(row)

    # ---------------------------------
    # CSV LOGGING BATTERIA
    # ---------------------------------

    def write_battery_csv(self):
        t = self.get_clock().now().nanoseconds * 1e-9
        row = [t]
        for name in self.drone_names:
            if len(self.battery[name]) > 0:
                row.append(self.battery[name][-1])
            else:
                row.append(None)
        self.csv_bat_writer.writerow(row)

    # ---------------------------------
    # PLOT UPDATE
    # ---------------------------------

    def update_plot(self):
        # ---- posizione droni ----
        self.ax3d.cla()
        self.ax2d.cla()
        for name in self.drone_names:
            x = self.positions[name]["x"]
            y = self.positions[name]["y"]
            z = self.positions[name]["z"]
            if len(x) == 0:
                continue
            self.ax3d.plot(x, y, z, label=name)
            self.ax2d.plot(x, y, label=name)

        # ---- waypoints ----
        if self.waypoints:
            wx = [w[0] for w in self.waypoints]
            wy = [w[1] for w in self.waypoints]
            wz = [w[2] for w in self.waypoints]
            self.ax3d.scatter(wx, wy, wz, marker='o', label="waypoints")
            self.ax2d.scatter(wx, wy, marker='o')

        # ---- ostacoli ----
        if self.obstacles:
            ox = [o[0] for o in self.obstacles]
            oy = [o[1] for o in self.obstacles]
            oz = [o[2] for o in self.obstacles]
            self.ax3d.scatter(ox, oy, oz, marker='x', label="obstacles")
            self.ax2d.scatter(ox, oy, marker='x')

        # ---- assi ----
        self.ax3d.set_xlabel("X")
        self.ax3d.set_ylabel("Y")
        self.ax3d.set_zlabel("Z")
        self.ax2d.set_xlabel("X")
        self.ax2d.set_ylabel("Y")

        handles, labels = self.ax3d.get_legend_handles_labels()
        if handles:
            self.ax3d.legend()
        handles, labels = self.ax2d.get_legend_handles_labels()
        if handles:
            self.ax2d.legend()

        # ---- plot battery ----
        self.ax_bat.cla()
        for name in self.drone_names:
            y = self.battery[name]
            x = list(range(len(y)))  # tempo discreto in frame
            if len(y) > 0:
                self.ax_bat.plot(x, y, label=name)
        self.ax_bat.set_xlabel("Frame")
        self.ax_bat.set_ylabel("Battery %")
        handles, labels = self.ax_bat.get_legend_handles_labels()
        if handles:
            self.ax_bat.legend()

        plt.pause(0.1/self.control_rate)


def main(args=None):
    rclpy.init(args=args)
    node = MultiDronePlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.csv_file.close()
    node.csv_bat_file.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()