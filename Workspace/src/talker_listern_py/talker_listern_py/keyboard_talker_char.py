import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import termios
import tty

class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.orig_termios = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin)

    def __del__(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.orig_termios)

    def capture_and_publish(self):
        try:
            while rclpy.ok():
                c = sys.stdin.read(1)
                if c == '\x03':  # Ctrl-C to exit
                    break
                msg = String()
                msg.data = c
                self.get_logger().info(f'Publishing: "{msg.data}"')
                self.publisher_.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Exception: {e}')
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.orig_termios)

def main(args=None):
    rclpy.init(args=args)
    node = Talker()
    try:
        node.capture_and_publish()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()