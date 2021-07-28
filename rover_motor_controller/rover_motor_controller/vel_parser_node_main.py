
import rclpy
from rover_motor_controller.motor_controller import VelParserNode


def main(args=None):
    rclpy.init(args=args)

    node = VelParserNode()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
