
from time import perf_counter
from math import pi


# TODO:
# add parsing to the state dictionary
# add rotation counter
# add low pass filtering of the cuirrent and velocity
# add sensor scales
# add setting encoder labels and bytes mapping as external functions
# decide which implementation is better, in atributes or in dict
# think how to store the state

class IncrementalEncoders:
    """This class provide interface to incremental encoders through CAN bus"""

    def __init__(self, can_bus=None, device_id=0x01, labels = {1,2}):

        # TODO: pass dict with reciver/transmitter functions from the specific bus
        if not can_bus:
            print("Provide can_bus as argument")
            self.__del__()
        
        self.bus =can_bus

        self.transmiter = can_bus.send_bytes
        self.reciver = can_bus.recive_frame

        self.device_id = device_id

        # TODO:
        #
        self.protocol = dict()
        self.protocol = {
            "get_counters": b"\x9C", 
            "reset_counters": b"\x05",
        }

        self.command = 8 * b"\x00"

        # \\\\\\ Offsets \\\\\
        self.counter_buffer = 2 ** 16 - 1
        
        self.encoders = {}

        self.encoders_labels = labels


        self.init_variables()
        self.bytes_mapping = {1:{'first':2, 'bytes':2},
                              2:{'first':0, 'bytes':2}}
        
        self.time = 0
        self.init_time = perf_counter()
        self.state = {}
        self.state['time'] = self.time 

        self.raw_state_data = dict(zip(self.encoders_labels, 2 * [0]))

    def __del__(self):
        self.bus.can_reset()
        print('IncrementalEncoder was deleted from memory')
        # pass


    def init_variables(self):
        self.counts, self.prev_counts, self.turns, self.scale, self.offset, self.pos, self.prev_pos, self.vel = {},{},{},{},{},{},{},{}
        for encoder in self.encoders_labels:
            self.counts[encoder] = 0
            self.prev_counts[encoder] = 0
            self.turns[encoder] = 0
            self.scale[encoder] = 1
            self.offset[encoder] = 0
            self.pos[encoder] = 0
            self.prev_pos[encoder] = 0
            self.vel[encoder] = 0
            # self.state[encoder] = {}

    # def update_state(self):
    #     for encoder in self.encoders_labels:
    #         self.state[encoder]['pos'] = {encoder:{}}

    # def init_variables(self):
    #     for encoder in self.encoders_labels:
    #         self.encoders[encoder]['counts'] = 0
    #         self.encoders[encoder]['prev_counts'] = 0
    #         self.encoders[encoder]['turns'] = 0
    #         self.encoders[encoder]['scale'] = 1
    #         self.encoders[encoder]['offset'] = 0
    #         self.encoders[encoder]['pos'] = 0
    #         self.encoders[encoder]['prev_pos'] = 0
    #         self.encoders[encoder]['vel'] = 0


    # def 

    def set_labels(self, labels):
        self.encoders_labels = labels
        

    # ///////////////////////////
    # /// Conversion routines ///
    # ///////////////////////////

    def to_bytes(self, n, integer, signed=True):
        """convert int to n bytes"""
        return int(integer).to_bytes(n, byteorder="little", signed=signed)

    def from_bytes(self, byte_string, signed=True):
        """convert bytes to int"""
        return int.from_bytes(byte_string, byteorder="little", signed=signed)

    def send_command(self, command):
        """send command via CAN bus"""
        self.transmiter(self.device_id, command)

    def recive_reply(self):
        """recive reply via CAN bus"""
        _, _, self.reply = self.reciver()
        return self.reply

    def set_scale(self, scales):
        """set the scales for encoders"""
        pass

    
    def reset_device(self):
        """Reset encoder counters and disable the motor driver"""
        command = self.protocol["reset_counters"] + 7 * b"\x00"
        self.send_command(command)
        self.recive_reply()
    
    # ////////// Parsing raw data //////////

    def parse_sensor_data(self, reply):
        """parse the raw sensor data from the CAN frame"""
        for encoder in self.encoders_labels:
            first  = self.bytes_mapping[encoder]['first']
            last = first + self.bytes_mapping[encoder]['bytes']
            self.raw_state_data[encoder] = self.from_bytes(reply[first:last], signed=False)
    
        return self.raw_state_data


    def get_counters(self):
        """Get the timers counters from the device"""
        
        self.command = (
            self.protocol["get_counters"]
            + 7 * b"\x00"
        )

        self.send_command(self.command)
        self.recive_reply()
        self.parse_sensor_data(self.reply)

    # ////////// Parsing Scaled Sensor Data //////////

    def counter_overflow(self, counts, prev_counts, scale, threshold=None):
        """handle overflow of counters with given scale and threshold"""

        turns = 0

        if not threshold:
            threshold = scale / 2

        if prev_counts - counts >= threshold:
            turns += 1
        elif prev_counts - counts <= -1 * threshold:
            turns -= 1

        return scale * turns

    def set_scale(self):
        pass

    def set_scale(self, scales):
        # do mapping from encoder labels dict to  
        # for scale in scales:
        #     self.encoders[encoder]['turns']
        pass 
        
    def get_state(self):
        """parse the position data from the CAN frame"""
        self.get_counters()
        raw_data = self.parse_sensor_data(
            self.reply
        )  # parse the raw data to self.raw_state_data

        t = perf_counter() - self.init_time
        dt = t - self.time
        self.time = t

        self.state["time"] = self.time

        for encoder in self.encoders_labels:
            # self.encoders[encoder] = {}
            self.counts[encoder] = raw_data[encoder]
            self.turns[encoder] += self.counter_overflow(
                self.counts[encoder], self.prev_counts[encoder], self.counter_buffer
            )
            self.prev_counts[encoder] = self.counts[encoder]
            self.pos[encoder] = (self.counts[encoder] + self.turns[encoder]) * self.scale[encoder]
            self.vel[encoder] = (self.pos[encoder] - self.prev_pos[encoder]) / dt
            self.prev_pos[encoder] = self.pos[encoder]
        return self.state


    def set_zero(self, encoders = None):
        if None:
            encoders_to_zero =  self.encoders_labels
        else:
            encoders_to_zero = encoders

        for encoder in encoders_to_zero:
            self.counts[encoder] = 0
            self.turns[encoder] = 0
    

    # def get_state(self):
    #     """parse the position data from the CAN frame"""
    #     self.get_counters()
    #     raw_data = self.parse_sensor_data(
    #         self.reply
    #     )  # parse the raw data to self.raw_state_data

    #     t = perf_counter() - self.init_time
    #     dt = t - self.time
    #     self.time = t

    #     self.state["time"] = self.time

    #     for encoder in self.encoders_labels:
    #         # self.encoders[encoder] = {}
    #         self.encoders[encoder]['counts'] = raw_data[encoder]
    #         self.encoders[encoder]['turns'] += self.counter_overflow(
    #             self.encoders[encoder]['counts'], self.encoders[encoder]['prev_counts'], self.counter_buffer
    #         )
    #         self.encoders[encoder]['prev_counts'] = self.encoders[encoder]['counts']
    #         self.encoders[encoder]['pos'] = (self.encoders[encoder]['counts'] + self.encoders[encoder]['turns']) * self.encoders[encoder]['scale']
    #         self.encoders[encoder]['vel'] = (self.encoders[encoder]['pos'] - self.encoders[encoder]['prev_pos']) / dt
    #         self.encoders[encoder]['prev_pos'] = self.encoders[encoder]['pos']
    #     return self.state



