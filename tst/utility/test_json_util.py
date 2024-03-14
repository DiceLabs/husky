import unittest
from unittest.mock import mock_open, patch, MagicMock
from json_util import load_json_file, get_key_from_data, get_key_from_file
import json

class TestJsonUtilities(unittest.TestCase):
    def setUp(self):
        self.sample_json_data = {"name": "Test", "value": 42}
        self.json_str = json.dumps(self.sample_json_data)
        self.json_file_path = "test.json"

    @patch("builtins.open", new_callable=mock_open, read_data=json.dumps({"name": "Test", "value": 42}))
    def test_load_json_file(self, mock_file):
        data = load_json_file(self.json_file_path)
        self.assertEqual(data, self.sample_json_data)

    def test_get_key_from_data(self):
        value = get_key_from_data(self.sample_json_data, "name")
        self.assertEqual(value, "Test")

    @patch("json_util.load_json_file", return_value={"name": "Test", "value": 42})
    def test_get_key_from_file(self, mock_load):
        value = get_key_from_file(self.json_file_path, "value")
        self.assertEqual(value, 42)

if __name__ == '__main__':
    unittest.main()

