import unittest
from timer import Timer 
import time

class TestTimer(unittest.TestCase):
    def setUp(self):
        self.callback_calls = 0
        self.timer = Timer(1, self.callback)  

    def callback(self):
        self.callback_calls += 1

    def test_initialization(self):
        self.assertEqual(self.timer.frequency, 1)
        self.assertFalse(self.timer.stop_event.is_set())

    def test_start_and_stop(self):
        self.timer.start()
        time.sleep(2) 
        self.timer.stop()
        self.assertTrue(self.timer.stop_event.is_set())

if __name__ == '__main__':
    unittest.main()
