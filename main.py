import rclpy

from collision_avoidance import CollisionAvoidance


def main(args=None):
    rclpy.init(args=args)
    avoider = CollisionAvoidance()
    rclpy.spin(avoider)

    avoider.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
