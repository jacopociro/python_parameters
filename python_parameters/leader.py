import rclpy
from rclpy.node import Node
import numpy as np
from crazy_interfaces.msg import Resourcemsg
from crazy_interfaces.srv import Resource  # Sostituisci con il nome effettivo del servizio

class ServiceClientNode(Node):
    def __init__(self):
        super().__init__('service_client_node')
        # Parametri generali di controllo
        self.declare_parameter('control_rate', 10.0)
        self.declare_parameter('max_velocity', 1.0)
        self.declare_parameter('max_acceleration', 0.5)
        self.declare_parameter('obstacle_topic', '/obstacle/pose')
        self.declare_parameter('drone_names', ['default_drone'])
        # Waypoints e stazioni di ricarica
        self.declare_parameter('wp.a', [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('wp.b', [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('wp.c', [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('wp.d', [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('wp.e', [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('wp.f', [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('charging_stations', [0.0, 0.0, 0.0, 0.0])

        self.control_rate = self.get_parameter('control_rate').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.max_acceleration = self.get_parameter('max_acceleration').value
        self.obstacle_topic = self.get_parameter('obstacle_topic').value
        self.drone_names = self.get_parameter('drone_names').value

        self.wp = {}
        self.wp['a'] = self.get_parameter('wp.a').value
        self.wp['b'] = self.get_parameter('wp.b').value
        self.wp['c'] = self.get_parameter('wp.c').value
        self.wp['d'] = self.get_parameter('wp.d').value
        self.wp['e'] = self.get_parameter('wp.e').value
        self.wp['f'] = self.get_parameter('wp.f').value

        # Logger waypoints
        self.get_logger().info("=== Waypoints ===")
        for wp in self.wp:
            self.get_logger().info(
                #f"x={wp[0]}, y={wp[1]}, z={wp[2]}, yaw={wp[3]}"
                f"{wp}: x={self.wp[wp][0]}, y={self.wp[wp][1]}, z={self.wp[wp][2]}, yaw={self.wp[wp][3]}"
            )
        self.Memory = np.zeros(len(self.wp))
        self.client0 = self.create_client(Resource, 'cf_0/resource')  # Sostituisci con il nome effettivo del servizio
        while not self.client0.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Servizio non disponibile, in attesa...')
        self.timer = self.create_timer(2.0, self.send_request)
        self.client1 = self.create_client(Resource, 'cf_1/resource')  # Sostituisci con il nome effettivo del servizio
        while not self.client1.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Servizio non disponibile, in attesa...')
        self.check = np.zeros(len(self.wp), dtype=bool)
        self.a = 0.0
        self.help = False
        self.split = 0.95
        self.timer = self.create_timer(2.0, self.send_request)

    def send_request(self):
        request = Resource.Request()
        # Imposta i campi della richiesta qui, ad esempio:
        # request.campo = valore
        if self.help == True:
            request.memory.mem1 = self.Memory[0]/3
            request.memory.mem2 = self.Memory[1]/3
            request.memory.mem3 = self.Memory[2]/3
            request.memory.mem4 = self.Memory[3]/3
            request.memory.mem5 = self.Memory[4]/3
            request.memory.mem6 = self.Memory[5]/3
            self.Memory[0] -= self.Memory[0]/3
            self.Memory[1] -= self.Memory[1]/3
            self.Memory[2] -= self.Memory[2]/3
            self.Memory[3] -= self.Memory[3]/3
            self.Memory[4] -= self.Memory[4]/3
            self.Memory[5] -= self.Memory[5]/3
        else:
            request.memory.mem1 = self.Memory[0]*self.split/len(self.drone_names)
            request.memory.mem2 = self.Memory[1]*self.split/len(self.drone_names)
            request.memory.mem3 = self.Memory[2]*self.split/len(self.drone_names)
            request.memory.mem4 = self.Memory[3]*self.split/len(self.drone_names)
            request.memory.mem5 = self.Memory[4]*self.split/len(self.drone_names)
            request.memory.mem6 = self.Memory[5]*self.split/len(self.drone_names)
            self.Memory[0] -= self.Memory[0]*self.split/len(self.drone_names)
            self.Memory[1] -= self.Memory[1]*self.split/len(self.drone_names)
            self.Memory[2] -= self.Memory[2]*self.split/len(self.drone_names)
            self.Memory[3] -= self.Memory[3]*self.split/len(self.drone_names)
            self.Memory[4] -= self.Memory[4]*self.split/len(self.drone_names)
            self.Memory[5] -= self.Memory[5]*self.split/len(self.drone_names)
        
        # Log the current state of Memory and self.a
        self.get_logger().info(f'Memory: {self.Memory}')
        self.get_logger().info(f'self.a: {self.a}')

        self.future = self.client0.call_async(request)
        self.future = self.client1.call_async(request)
        self.future.add_done_callback(self.handle_response)

    def handle_response(self, future):
    
        response = future.result()
        self.Memory[0] += response.memory.mem1
        self.Memory[1] += response.memory.mem2
        self.Memory[2] += response.memory.mem3
        self.Memory[3] += response.memory.mem4
        self.Memory[4] += response.memory.mem5
        self.Memory[5] += response.memory.mem6

        self.help = response.help

        ph_prod = self.photosynthesis_prod(self.check, self.a)
        self.a += ph_prod
        # Gestisci la risposta qui
        self.get_logger().info(f'Risposta ricevuta: {response}')
        
    def photosynthesis_prod(self, check, a):
        lambda_ = 0.82
        a_max = 6 * (24 - 18)  # 18 hours of light
        ph_max = 12.7
        C = lambda_ + (1 - lambda_) * (a_max - a) / a_max

        min_mem = 100.0  # Initialize to a large value

        for i in range(len(self.Memory)):
            if self.Memory[i] != 0.0:
                check[i] = True
            if check[i] and self.Memory[i] < min_mem:
                min_mem = self.Memory[i]

        if min_mem == 100.0:
            min_mem = 0.0  # If no memory is used, set to 0

        for i in range(len(self.Memory)):
            self.Memory[i] = self.Memory[i] - min_mem

        ph_prod = ph_max * min_mem * C
        return ph_prod

def main(args=None):
    rclpy.init(args=args)
    node = ServiceClientNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()