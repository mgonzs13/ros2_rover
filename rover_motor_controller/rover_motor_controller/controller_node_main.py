
import rclpy
from rover_motor_controller.motor_controller import ControllerNode


def main(args=None):
    rclpy.init(args=args)

    node = ControllerNode()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
