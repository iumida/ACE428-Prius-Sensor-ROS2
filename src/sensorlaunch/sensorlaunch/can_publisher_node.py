import rclpy
from rclpy.node import Node
from autoware_vehicle_msgs.msg import VelocityReport, SteeringReport
import socket
import struct


class CanPublisherNode(Node):

    def __init__(self):
        super().__init__('can_publisher_node')

        self.velocity_pub = self.create_publisher(
            VelocityReport, '/velocity_can', 10)
        self.steering_pub = self.create_publisher(
            SteeringReport, '/steering_can', 10)

        # CAN interface setup
        self.can_socket = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.can_socket.bind(('can0',))

        # Set CAN filter for specific messages (0x0B4 and 0x025)
        can_filter = [
            struct.pack("=II", 0x0B4, 0x7FF),
            struct.pack("=II", 0x025, 0x7FF)
        ]
        self.can_socket.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, b''.join(can_filter))

        self.timer = self.create_timer(0.001, self.publish_can_data)

    def publish_can_data(self):
        frame_fmt = "<IB3x8s"
        frame_size = struct.calcsize(frame_fmt)
        frame_data = self.can_socket.recv(frame_size)

        can_id, can_dlc, data = struct.unpack(frame_fmt, frame_data)

        if can_id == 0x0B4:
            velocity_kmh = ((data[5] << 8) | data[6]) * 0.01

            velocity_msg = VelocityReport()
            velocity_msg.header.stamp = self.get_clock().now().to_msg()
            velocity_msg.longitudinal_velocity = velocity_kmh
            velocity_msg.lateral_velocity = 0.0
            velocity_msg.heading_rate = 0.0

            self.velocity_pub.publish(velocity_msg)

        elif can_id == 0x025:
            hex_value = (data[0] << 8) | data[1]
            angle_raw = hex_value & 0x0FFF

            if angle_raw & 0x0800:
                angle_raw |= 0xF000

            steering_angle = struct.unpack('h', struct.pack('H', angle_raw))[0] * 1.5

            steering_msg = SteeringReport()
            steering_msg.stamp = self.get_clock().now().to_msg()
            steering_msg.steering_tire_angle = steering_angle

            self.steering_pub.publish(steering_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CanPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
