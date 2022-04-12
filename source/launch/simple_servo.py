# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

"""Simple test for a standard servo on channel 0 and a continuous rotation servo on channel 1."""
import time
from adafruit_servokit import ServoKit

# Set channels to the number of servo channels on your kit.
# 8 for FeatherWing, 16 for Shield/HAT/Bonnet.
kit = ServoKit(channels=16)

while True:
  kit.servo[13].angle = 180
#  kit.continuous_servo[1].throttle = 1
  time.sleep(1)
  # kit.continuous_servo[1].throttle = -1
  # time.sleep(1)
  kit.servo[13].angle = 88.5
  # kit.continuous_servo[1].throttle = 0
  time.sleep(2)


# servo mapping
# 0 - rm_steer
# 1 - rf_steer
# 2 - rm_drive
# 3 - rf_drive
# 4 - rr_steer
# 5 - lr_drive
# 6 - lr_steer
# 7 - rr_drive
# 8 - lf_drive
# 9 - lm_drive
# 10 - lm_steer
# 11 - lf_steer
# 12 - camera_x
# 13 - camera_y
# 14 - 
# 15 - 




