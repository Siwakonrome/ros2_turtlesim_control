import rclpy
import rclpy.node

class OperatingClass(rclpy.node.Node):
    def __init__(self):
        super().__init__('tcp_client_service_server_node')
        self.init_parameter()
        self.timer = self.create_timer(1, self.timer_callback)
    
    def init_parameter(self):
        self.declare_parameter('server_ip', '127.0.0.1')
        self.declare_parameter('server_port', 65432)
        self.declare_parameter('flush_repeat', 10)
        self.declare_parameter('receive_timeout', 2.0)
        self.declare_parameter('receive_buffer_timeout', 0.1)

    def timer_callback(self):
        my_param = self.get_parameter('server_ip').get_parameter_value().string_value
        self.get_logger().info('Hello %s!' % my_param)


def main():
    rclpy.init()
    node = OperatingClass()
    rclpy.spin(node)

if __name__ == '__main__':
    main()