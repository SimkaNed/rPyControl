# TODO:
# add rotation counter
# add low pass filtering of the cuirrent and velocity
# put state in dictionary
# add conversion from raw data to state
# add sensor scales

from time import perf_counter
from math import pi


# TODO:


class IncrementalEncoders:
    """This class provide interface to incremental encoders through CAN bus"""

    def __init__(self, can_bus=None, device_id=0x01):

        # TODO: pass dict with reciver/transmitter functions from the specific bus
        if not can_bus:
            print("Provide can_bus as argument")
            self.__del__()

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
        self.encoders_labels = {1,2}
        self.bytes_mapping = {1:{'first':0, 'bytes':2},
                              2:{'first':2, 'bytes':2}}

        for encoder in self.encoders_labels:
            self.encoders[encoder]['counts'] = 0
            self.encoders[encoder]['pos'] = 0
            self.encoders[encoder]['vel'] = 0
            self.encoders[encoder]['scale'] = 1  

        self.time = 0

        self.raw_state_data = dict(zip(self.encoders_labels, 2 * [0]))

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
        command = self.protocol["reset_device"] + 7 * b"\x00"
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

    # def get_state(self, reply):
    #     """parse the motor state from CAN frame"""
    #     raw_data = self.parse_sensor_data(
    #         reply
    #     )  # parse the raw data to self.raw_state_data

    #     t = perf_counter() - self.init_time
    #     dt = t - self.time
    #     # print(dt)
    #     self.time = t

    #     self.state["time"] = self.time

    #     # Parse angular position and speed
    #     motor_counts = raw_data["mot_counts"]
    #     self.motor_turns += self.counter_overflow(
    #         motor_counts, self.motor_counts, self.counter_buffer
    #     )
    #     self.motor_counts = motor_counts
    #     self.motor_pos = (motor_counts + self.motor_turns) * self.angle_scale
    #     self.state["motor_speed"] = (self.motor_pos - self.state["motor_pos"]) / dt
    #     self.state["motor_pos"] = self.motor_pos

    #     # Parse linear position and speed
    #     lin_counts = raw_data["lin_counts"]
    #     self.lin_turns += self.counter_overflow(
    #         abs(lin_counts), abs(self.lin_counts), self.counter_buffer
    #     )
    #     self.lin_counts = lin_counts
    #     self.lin_pos = (lin_counts + self.lin_turns) * self.linear_scale
    #     self.state["lin_speed"] = (self.lin_pos - self.state["lin_pos"]) / dt
    #     self.state["lin_pos"] = self.lin_pos

    #     # Parse current, torque and tenstion
    #     self.state["current"] = self.get_sensor(
    #         raw_data["current_counts"], self.current_scale, self.current_offset
    #     )
    #     self.state["torque"] = self.torque_constant * self.state["current"]
    #     self.state["tension"] = self.get_sensor(
    #         raw_data["force_counts"], self.force_scale, self.force_offset
    #     )

    #     return self.state
