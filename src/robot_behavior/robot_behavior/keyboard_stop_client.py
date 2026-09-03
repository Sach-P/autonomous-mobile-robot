import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from pynput import keyboard

class KeyboardStopClient(Node):
    def __init__(self):
        super().__init__('keyboard_stop_client')
        self._cli = self.create_client(Trigger, 'stop_robot')

        while not self._cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for /stop_robot service to become available.')

        
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()

    def on_press(self, key):

        # SPACE or ESC to stop robot
        try:
            if key == keyboard.Key.space or key == keyboard.Key.esc:
                self.send_stop_request()

        except Exception as e:
            self.get_logger().error(f'Error reading key: {e}')


    def send_stop_request(self):
        self.get_logger().warn('Triggering /stop_robot service call')
        req = Trigger.Request()
        future = self._cli.call_async(req)
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            res = future.result()
            self.get_logger().info(f'Serivce Response: success={res.success}, msg={res.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: success={res.success}, msg={res.message}')

def main():
    rclpy.init()
    node = KeyboardStopClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()