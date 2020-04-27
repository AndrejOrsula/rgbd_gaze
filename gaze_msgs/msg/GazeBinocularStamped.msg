# Message that contains gaze information for both eyes with a standard header

# Index of the left eye
uint8 EYE_LEFT=0
# Index of the right eye
uint8 EYE_RIGHT=1

# Standard header
std_msgs/Header header
# Gaze information for the left eye
Gaze[2] gaze
