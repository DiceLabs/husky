import unittest
from robot_types import Position, Quaternion, Euler, PoseM, LinearVelocity, AngularVelocity

class TestGeometryClasses(unittest.TestCase):

    def test_position_initialization(self):
        pos = Position(1.0, 2.0, 3.0)
        self.assertEqual(pos.x, 1.0)
        self.assertEqual(pos.y, 2.0)
        self.assertEqual(pos.z, 3.0)

    def test_quaternion_initialization(self):
        quat = Quaternion(1.0, 0.0, 0.5, 0.5)
        self.assertEqual(quat.w, 1.0)
        self.assertEqual(quat.x, 0.0)
        self.assertEqual(quat.y, 0.5)
        self.assertEqual(quat.z, 0.5)

    def test_euler_initialization(self):
        euler = Euler(45.0, 30.0, 90.0)
        self.assertEqual(euler.yaw, 45.0)
        self.assertEqual(euler.pitch, 30.0)
        self.assertEqual(euler.roll, 90.0)

    def test_posem_initialization(self):
        pose = PoseM(Euler(45.0, 30.0, 15.0), Position(1.0, 2.0, 3.0))
        self.assertEqual(pose.orientation.yaw, 45.0)
        self.assertEqual(pose.orientation.pitch, 30.0)
        self.assertEqual(pose.orientation.roll, 15.0)
        self.assertEqual(pose.position.x, 1.0)
        self.assertEqual(pose.position.y, 2.0)
        self.assertEqual(pose.position.z, 3.0)

    def test_linear_velocity_initialization(self):
        lin_vel = LinearVelocity(1.0, -1.0, 0.5)
        self.assertEqual(lin_vel.x, 1.0)
        self.assertEqual(lin_vel.y, -1.0)
        self.assertEqual(lin_vel.z, 0.5)

    def test_angular_velocity_initialization(self):
        ang_vel = AngularVelocity(0.5, 1.5, -0.5)
        self.assertEqual(ang_vel.x, 0.5)
        self.assertEqual(ang_vel.y, 1.5)
        self.assertEqual(ang_vel.z, -0.5)

if __name__ == '__main__':
    unittest.main()
