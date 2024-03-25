#!/usr/bin/env python3

import unittest
from dexterity import Dexterity
from protocol import GenericComponentMetaData
from factory import convert_int_to_id, ComponentFactory, ComponentId

class TestComponentSystem(unittest.TestCase):
    
    def test_convert_int_to_id(self):
        self.assertEqual(convert_int_to_id(0), ComponentId.BASE)
        self.assertEqual(convert_int_to_id(1), ComponentId.LEFT_ARM)
        self.assertEqual(convert_int_to_id(3), ComponentId.LEFT_GRIPPER)
        
        self.assertIsNone(convert_int_to_id(99))
    
    def test_component_factory(self):
        factory = ComponentFactory()
        
        self.assertTrue(ComponentId.BASE in factory)
        self.assertTrue(ComponentId.LEFT_GRIPPER in factory)
        self.assertTrue(ComponentId.RIGHT_GRIPPER in factory)
        
        base_metadata = factory[ComponentId.BASE]
        self.assertIsInstance(base_metadata, GenericComponentMetaData)
        
        self.assertEqual(base_metadata.name, "Base")
        self.assertEqual(factory[ComponentId.LEFT_GRIPPER].name, "Left_Gripper")
        
        self.assertEqual(factory[ComponentId.LEFT_GRIPPER].args, {"dexterity": Dexterity.LEFT})
        self.assertEqual(factory[ComponentId.RIGHT_GRIPPER].args, {"dexterity": Dexterity.RIGHT})
        
        self.assertFalse(ComponentId.RIGHT_ARM in factory)
        self.assertFalse(ComponentId.LEFT_ARM in factory)
    
if __name__ == '__main__':
    unittest.main()
