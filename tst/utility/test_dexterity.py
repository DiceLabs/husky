import unittest
from dexterity import Dexterity

class TestDexterityEnum(unittest.TestCase):
    def test_enum_members(self):
        self.assertEqual(Dexterity.LEFT.value, 0)
        self.assertEqual(Dexterity.RIGHT.value, 1)

    def test_enum_str(self):
        self.assertEqual(str(Dexterity.LEFT), "left")
        self.assertEqual(str(Dexterity.RIGHT), "right")

if __name__ == '__main__':
    unittest.main()
