import unittest
import math
from conversions import quaternion_to_euler, euler_to_quaternion, degrees_to_radians

class TestConversions(unittest.TestCase):

    def test_quaternion_to_euler_to_quaternion(self):
        w, x, y, z = 0.707, 0.707, 0, 0
        roll, pitch, yaw = quaternion_to_euler(w, x, y, z)
        w1, x1, y1, z1 = euler_to_quaternion(yaw, pitch, roll)

        self.assertAlmostEqual(w, w1)
        self.assertAlmostEqual(x, x1)
        self.assertAlmostEqual(y, y1)
        self.assertAlmostEqual(z, z1)

    def test_degrees_to_radians_to_degrees(self):
        yaw_deg, pitch_deg, roll_deg = 90, 45, 30
        yaw_rad, pitch_rad, roll_rad = degrees_to_radians(yaw_deg, pitch_deg, roll_deg)
        yaw_deg1, pitch_deg1, roll_deg1 = math.degrees(yaw_rad), math.degrees(pitch_rad), math.degrees(roll_rad)

        self.assertAlmostEqual(yaw_deg, yaw_deg1)
        self.assertAlmostEqual(pitch_deg, pitch_deg1)
        self.assertAlmostEqual(roll_deg, roll_deg1)

if __name__ == '__main__':
    unittest.main()
