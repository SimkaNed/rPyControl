from ...actuators.gyems import GyemsDRC
from ...sensors.can_sensors.encoders import IncrementalEncoders
from time import perf_counter
from math import pi
from multiprocessing import Process, Value
from os import nice

# TODO:
#   implement error handler - sudden jumps in control effort
#   global ticker inside class, thread/process?
#   make helper class with sensor related functions
#   make a two processes, one for motor another for sensors


class Antagonist:
    """ This class provide interface to the Gyems BLDC motor driver over CAN socket"""

    def __init__(
        self,
        motors_bus=None,
        sensors_bus=None,
        motor_id={'left': 0x141, 'right': 0x142},
        sensor_id=0x01,
    ):

        self.encoders = {'sensors': IncrementalEncoders(can_bus = sensors_bus)}
        self.cpi = 360 
        
        self.actuators = {}
        # self.actuators = 
        self.state = {}
        
        for actuator in ['left', 'right']:
            self.actuators[actuator] = {
                'motor': GyemsDRC(can_bus=motors_bus, device_id=motor_id[actuator]),
                'limit': 500,
                'angle_offset': 0,
                'pos_offset': 0
            }
            self.actuators[actuator]['motor'].set_rad()
            self.actuators[actuator]['motor'].current_limit = self.actuators[actuator]['limit']
            self.encoders[actuator]['counts'] = Value('d', 0)
            self.state[actuator] = self.actuators[actuator]['motor'].state
            self.state[actuator].update({'lin_pos':0, 'lin_vel':0})

        # TODO:
        #   define and begin sensing process with shared variables for pos states
        

    def parse_state(self):
        '''Parse the state'''
        for actuator in {'left', 'right'}:
            self.state[actuator] = self.actuators[actuator]['motor'].state
            self.state[actuator].update({'lin_pos':Value('d',0), 'lin_vel':0})
            

        # self.state = 0
        # self.
        # pass

    def set_control(self):
        pass
    
    def init_encoders(self):
        pass

    def init_actuator(self):
        '''Initialization for one of the actuators'''
        pass

    # def go_to():