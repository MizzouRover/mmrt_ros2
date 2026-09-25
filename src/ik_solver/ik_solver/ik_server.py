# Service:  /solve_ik   custom_interfaces/srv/SolveIK
#   request:  geometry_msgs/Point position, geometry_msgs/Quaternion orientatio 
#   response: bool solved
#
# colcon build --packages-select custom_interfaces
# source install/setup.bash
# python3 src/ik_solver/ik_solver/ik_server.py

import math

import rclpy
from rclpy.node import Node

from custom_interfaces.srv import SolveIK

try:
    from ik_solver.kinematics import ArmGeometry, UnreachableError, solve  # when installed as a package
except ImportError:
    from kinematics import ArmGeometry, UnreachableError, solve  # when run directly from this folder


class IKServer(Node):
    def __init__(self):
        super().__init__('ik_server')

        # Arm geometry as ROS parameters (defaults match the example in kinematics.py main())
        self.declare_parameter('l1', 2.0)
        self.declare_parameter('l2', 2.0)
        self.declare_parameter('l3', 0.5)
        self.declare_parameter('shoulder_x', 0.0)
        self.declare_parameter('shoulder_z', 0.0)
        self.declare_parameter('slide_min', -0.1)
        self.declare_parameter('slide_max', 0.1)
        self.declare_parameter('elbow_up', True)
        self.declare_parameter('yaw_tolerance', 0.01)  # how far out of the x/z plane the claw axis may point

        self.geo = ArmGeometry(
            l1=self.get_parameter('l1').value,
            l2=self.get_parameter('l2').value,
            l3=self.get_parameter('l3').value,
            shoulder_x=self.get_parameter('shoulder_x').value,
            shoulder_z=self.get_parameter('shoulder_z').value,
            slide_min=self.get_parameter('slide_min').value,
            slide_max=self.get_parameter('slide_max').value,
        )
        self.elbow_up = self.get_parameter('elbow_up').value
        self.yaw_tolerance = self.get_parameter('yaw_tolerance').value

        self.srv = self.create_service(SolveIK, 'solve_ik', self.solve_callback)
        self.get_logger().info(f'IK server ready on /solve_ik with {self.geo}')

    def solve_callback(self, request, response):
        p = request.position
        q = request.orientation

        # normalize the quaternion
        n = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if n < 1e-9:
            self.get_logger().warn('No solution: orientation quaternion is all zeros')
            response.solved = False
            return response
        qx, qy, qz, qw = q.x / n, q.y / n, q.z / n, q.w / n

        # claw pointing axis = first column of the rotation matrix
        dx = 1 - 2 * (qy * qy + qz * qz)
        dy = 2 * (qx * qy + qw * qz)
        dz = 2 * (qx * qz - qw * qy)
        # claw's local y axis = second column of the rotation matrix
        r01 = 2 * (qx * qy - qw * qz)
        r11 = 1 - 2 * (qx * qx + qz * qz)
        r21 = 2 * (qy * qz + qw * qx)

        if abs(dy) > self.yaw_tolerance:
            self.get_logger().warn(f'No solution: claw would need to yaw (pointing axis y component {dy:.3f})')
            response.solved = False
            return response

        # pitch: angle of the pointing axis above horizontal, in the x/z plane (+ = up)
        pitch = math.atan2(dz, dx)
        # roll: rotation of the claw's y axis about the pointing axis, measured from world y
        roll = math.atan2(dx * r21 - dz * r01, r11)

        try:
            solution = solve(p.x, p.y, p.z, pitch, roll, self.geo, self.elbow_up)
        except UnreachableError as e:
            self.get_logger().warn(f'No solution: {e}')
            response.solved = False
            return response

        self.get_logger().info(f'Solved: {solution}')
        response.solved = True
        return response


def main(args=None):
    rclpy.init(args=args)
    node = IKServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
