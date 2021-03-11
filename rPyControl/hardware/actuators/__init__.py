
from time import perf_counter
from math import pi

# TODO:
# Run motor in several modes in processes:
# i.e: speed, pos, current
# Implement motor modes
# motor.set_mode('s', 'p', 'c')
# via motor.run(mode = 's')
# provide interface to can_bus through tuple of functions
#   tranciver
#   reciver


class Actuator:
    """ """

    def __init__(self, can_bus=None, device_id=0x01, units="rad"):
        pass

    def __del__(self):
        pass
