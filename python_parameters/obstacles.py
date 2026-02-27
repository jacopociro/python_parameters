import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class ObstaclePublisher(Node):
    def __init__(self):
        super().__init__('obstacle_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, '/obstacle/pose', 10)
        self.timer = self.create_timer(0.025, self.publish_positions)
        self.positions = self.generate_positions()
        self.index = 0

    def generate_positions(self):
        obstacles = []
        obstacle_positions = [
            (3.0, 0.0, 0.0),
            # (1.0, 1.0, 0.0),
            # (2.0, -1.0, 0.0),
            # (-1.0, -0.5, 0.0)
        ]
        for i in range(range(len(obstacle_positions))):
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'map'
            pose.pose.position.x = obstacle_positions[i][0]
            pose.pose.position.y = obstacle_positions[i][1]
            pose.pose.position.z = obstacle_positions[i][2]
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 0.0

            obstacles.append(pose)
        return obstacles

    def publish_positions(self):
        if self.index >= len(self.positions):
            self.index = 0
        msg = self.positions[self.index]
        self.publisher_.publish(msg)
        #self.get_logger().info(f'Published position: {msg}')


        self.index = (self.index + 1)

def main(args=None):
    rclpy.init(args=args)
    node = ObstaclePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()