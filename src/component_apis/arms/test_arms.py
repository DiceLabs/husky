import pytest
import argparse
import key_arms
from unittest.mock import patch, MagicMock

def test_parse_args():
    assert key_arms.parse_args("l") == key_arms.Dexterity.LEFT
    assert key_arms.parse_args("r") == key_arms.Dexterity.RIGHT
    with pytest.raises(argparse.ArgumentTypeError):
        key_arms.parse_args("invalid")

@patch('sys.argv', ['key_arms.py', '-d', 'l'])
def test_get_args():
    args = key_arms.get_args()
    assert args.dexterity == 'l'

@patch('key_arms.rospy.ROSInterruptException', new_callable=MagicMock)
@patch('key_arms.UR5e_Arm')
def test_on_key_press(mock_arm, mock_exception):
    actions = {
        key_arms.Dexterity.LEFT: {'q': mock_arm.move_up}
    }
    mock_key = MagicMock()
    mock_key.char = 'q'

    key_arms.on_key_press(mock_key, mock_arm, actions)

    mock_arm.move_up.assert_called_once()

    mock_exception.side_effect = Exception("Interrupt")
    with pytest.raises(Exception):
        key_arms.on_key_press(mock_key, mock_arm, actions)
