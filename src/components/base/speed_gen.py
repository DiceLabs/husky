#!/usr/bin/env python3

TIME_STEP = 0.05
MAX_SPEED = 0.1

def generate_speed_profile(distance, max_speed=MAX_SPEED, time_step=TIME_STEP):
    import numpy as np
    # Total time for half the sine wave (π/2)
    half_sine_time = np.pi / (2 * max_speed)

    # Total time to cover the distance at max speed
    total_time_at_max = distance / max_speed - half_sine_time

    if total_time_at_max < 0:
        # Distance is too short to reach max speed
        half_sine_time = np.sqrt(distance / max_speed)
        total_time_at_max = 0

    total_time = 2 * half_sine_time + total_time_at_max
    time_array = np.arange(0, total_time, time_step)

    # Speed profile
    speed_profile = []

    for t in time_array:
        if t < half_sine_time:
            # Acceleration phase
            speed = max_speed * np.sin((t / half_sine_time) * (np.pi / 2))
        elif t < half_sine_time + total_time_at_max:
            # Constant max speed
            speed = max_speed
        else:
            # Deceleration phase
            decel_time = t - half_sine_time - total_time_at_max
            speed = max_speed * np.cos((decel_time / half_sine_time) * (np.pi / 2))

        speed_profile.append(speed)

    return np.array(speed_profile)