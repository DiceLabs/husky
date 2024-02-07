#!/usr/bin/env python3

from speed_gen import generate_speed_profile

def test_speed_gen():
    vals = generate_speed_profile(.3)
    print(vals)
    total = sum(val * .05 for val in vals)
    assert total == .3
    
if __name__ == "__main__":
    test_speed_gen()