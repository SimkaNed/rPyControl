from time import perf_counter
from math import pi


# CANSensors is class that provide interface between

# TODO:
# CANSensors created through the can_bus object
#   argument to initalize:
#       labels = dictionary or set with labels/numbers for sensors
#
#       sensor_dict{
#                   types : dictionarry with types
#                   offsets : dictionary with ofsets
#                   scales : dictionarry with scales for sensors
#                   data_map : mapping between sensors and recived package
#                   }
#
#       protocol = represent the protocol with several commands, may be changed via .map_protocol()
#
# parsing of sensors should be done in separate process
#
# Implement filtering
# Implement


class CANSensors:
    """This class map CAN messages coming from specific bus encoders through CAN bus"""
    # pass

    def __init__(self, can_bus=None, device_id=0x01, labels=None, sensors_params={}):

        # TODO: pass dict with reciver/transmitter functions from the specific bus
        if not can_bus:
            print("Provide can_bus as argument")
            self.__del__()

        # TODO: move this to class CANDevice, CANSensors should inherit from CANDevice
        self.bus = can_bus
        self.transmiter = can_bus.send_bytes
        self.reciver = can_bus.recive_frame
        self.device_id = device_id

        if not labels:
            print("Labels are not provided!!!\n Do you want to initialize sensors object with one sensor named 'sens'?\n Y - for yes")
            ans = input()
            if ans == 'Y' or 'y':
                self.labels = {'sens'}


        self.count, self.scales, self.offsets, self.overflow, self.data_map, self.differences, self.filters = {}, {}, {}, {}, {}, {}, {}

        for sensor_labels in self.labels:
            self.counts[sensor_labels] = 0
            self.scales[sensor_labels] = 1
            self.offsets[sensor_labels] = 0
            self.data_map[sensor_labels] = [0, 2]
            self.differences[sensor_labels] = 0
            self.overflow[sensor_labels] = False
            self.filters = None


        self.protocol = {
            "init_sensors":b"\x9B"
            "get_sensors": b"\x9C",
            "reset_counters": b"\x05",
        }

        self.raw = {}

    # def init_paremey

    def request_counters(self):
        self.command = {}
        pass

    def execute(self):
        self.time = 0
        # self.request_counters()
        self.
        # self.
        pass

    def parse_data(self):
        pass


    def overflow_routine(self):
        pass

    def reset_counters(self, sensors=None):
        if None:
            sensors_to_reset = self.sensors_labels
        else:
            sensors_to_reset = sensors

        for sensors in counters_to_reset:
            self.sensors[counter] = 0
            self.turns[sensor] = 0


    # def update_data(self):
    # def

    # TODO: run send/recive in separate process
    #  self.get_raw()
    #  self.parse_data()
    #  self.

    # def parse_encoders(self):
    #     pass

    # def parse_sensor(self):
    #     pass

    # def differentiate(self):
    #     pass

    # self.

    #     # TODO:
    #     #
    #     self.protocol = dict()
    #     self.protocol = {
    #         "get_sensors": b"\x9C",
    #         "reset_counters": b"\x05",
    #     }

    #     self.command = 8 * b"\x00"

    #     # \\\\\\ Offsets \\\\\
    #     self.counter_buffer = 2 ** 16 - 1

    #     self.encoders = {}

    #     self.encoders_labels = labels

    #     self.init_variables()
    #     self.bytes_mapping = {1:{'first':2, 'bytes':2},
    #                           2:{'first':0, 'bytes':2}}

    #     self.time = 0
    #     self.init_time = perf_counter()
    #     self.state = {}
    #     self.state['time'] = self.time

    #     self.raw_state_data = dict(zip(self.encoders_labels, 2 * [0]))

    # def __del__(self):
    #     self.bus.can_reset()
    #     print('IncrementalEncoder was deleted from memory')
    #     # pass

    # def init_variables(self):
    #     self.counts, self.prev_counts, self.turns, self.scale, self.offset, self.pos, self.prev_pos, self.vel = {},{},{},{},{},{},{},{}
    #     for encoder in self.encoders_labels:
    #         self.counts[encoder] = 0
    #         self.prev_counts[encoder] = 0
    #         self.turns[encoder] = 0
    #         self.scale[encoder] = 1
    #         self.offset[encoder] = 0
    #         self.pos[encoder] = 0
    #         self.prev_pos[encoder] = 0
    #         self.vel[encoder] = 0
    #         # self.state[encoder] = {}

    # # def update_state(self):
    # #     for encoder in self.encoders_labels:
    # #         self.state[encoder]['pos'] = {encoder:{}}

    # # def init_variables(self):
    # #     for encoder in self.encoders_labels:
    # #         self.encoders[encoder]['counts'] = 0
    # #         self.encoders[encoder]['prev_counts'] = 0
    # #         self.encoders[encoder]['turns'] = 0
    # #         self.encoders[encoder]['scale'] = 1
    # #         self.encoders[encoder]['offset'] = 0
    # #         self.encoders[encoder]['pos'] = 0
    # #         self.encoders[encoder]['prev_pos'] = 0
    # #         self.encoders[encoder]['vel'] = 0

    # # def

    # def set_labels(self, labels):
    #     self.encoders_labels = labels

    # # ///////////////////////////
    # # /// Conversion routines ///
    # # ///////////////////////////

    # def to_bytes(self, n, integer, signed=True):
    #     """convert int to n bytes"""
    #     return int(integer).to_bytes(n, byteorder="little", signed=signed)

    # def from_bytes(self, byte_string, signed=True):
    #     """convert bytes to int"""
    #     return int.from_bytes(byte_string, byteorder="little", signed=signed)

    # def send_command(self, command):
    #     """send command via CAN bus"""
    #     self.transmiter(self.device_id, command)

    # def recive_reply(self):
    #     """recive reply via CAN bus"""
    #     _, _, self.reply = self.reciver()
    #     return self.reply

    # def set_scale(self, scales):
    #     """set the scales for encoders"""
    #     pass

    # def reset_device(self):
    #     """Reset encoder counters and disable the motor driver"""
    #     command = self.protocol["reset_counters"] + 7 * b"\x00"
    #     self.send_command(command)
    #     self.recive_reply()

    # # ////////// Parsing raw data //////////

    # def parse_sensor_data(self, reply):
    #     """parse the raw sensor data from the CAN frame"""
    #     for encoder in self.encoders_labels:
    #         first  = self.bytes_mapping[encoder]['first']
    #         last = first + self.bytes_mapping[encoder]['bytes']
    #         self.raw_state_data[encoder] = self.from_bytes(reply[first:last], signed=False)

    #     return self.raw_state_data

    # def get_counters(self):
    #     """Get the timers counters from the device"""

    #     self.command = (
    #         self.protocol["get_counters"]
    #         + 7 * b"\x00"
    #     )

    #     self.send_command(self.command)
    #     self.recive_reply()
    #     self.parse_sensor_data(self.reply)

    # # ////////// Parsing Scaled Sensor Data //////////

    # def counter_overflow(self, counts, prev_counts, scale, threshold=None):
    #     """handle overflow of counters with given scale and threshold"""

    #     turns = 0

    #     if not threshold:
    #         threshold = scale / 2

    #     if prev_counts - counts >= threshold:
    #         turns += 1
    #     elif prev_counts - counts <= -1 * threshold:
    #         turns -= 1

    #     return scale * turns

    # def set_scale(self):
    #     pass

    # def set_scale(self, scales):
    #     # do mapping from encoder labels dict to
    #     # for scale in scales:
    #     #     self.encoders[encoder]['turns']
    #     pass

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
    #         self.counts[encoder] = raw_data[encoder]
    #         self.turns[encoder] += self.counter_overflow(
    #             self.counts[encoder], self.prev_counts[encoder], self.counter_buffer
    #         )
    #         self.prev_counts[encoder] = self.counts[encoder]
    #         self.pos[encoder] = (self.counts[encoder] + self.turns[encoder]) * self.scale[encoder]
    #         self.vel[encoder] = (self.pos[encoder] - self.prev_pos[encoder]) / dt
    #         self.prev_pos[encoder] = self.pos[encoder]
    #     return self.state

    # def set_zero(self, encoders = None):
    #     if None:
    #         encoders_to_zero =  self.encoders_labels
    #     else:
    #         encoders_to_zero = encoders

    #     for encoder in encoders_to_zero:
    #         self.counts[encoder] = 0
    #         self.turns[encoder] = 0

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


class SensorsTSA:

    """This class provide interface to incremental encoders through CAN bus"""

    def __init__(self, can_bus=None, device_id=0x01, labels={1, 2, "frc"}):

        # TODO: pass dict with reciver/transmitter functions from the specific bus
        if not can_bus:
            print("Provide can_bus as argument")
            self.__del__()

        self.bus = can_bus

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

        self.sensors_labels = labels

        self.init_variables()
        self.bytes_mapping = {1: {'first': 2, 'bytes': 2},
                              2: {'first': 0, 'bytes': 2},
                              "frc": {'first': 4, 'bytes': 2}}

        self.time = 0
        self.init_time = perf_counter()
        self.state = {}
        self.state['time'] = self.time

        self.raw_state_data = dict(zip(self.sensors_labels, 3 * [0]))

    def __del__(self):
        self.bus.can_reset()
        print('IncrementalEncoder was deleted from memory')
        # pass

    def init_variables(self):
        self.counts, self.prev_counts, self.turns, self.scale, self.offset, self.pos, self.prev_pos, self.vel = {
        }, {}, {}, {}, {}, {}, {}, {}
        for sensor in self.sensors_labels:
            self.counts[sensor] = 0
            self.prev_counts[sensor] = 0
            self.turns[sensor] = 0
            self.scale[sensor] = 1
            self.offset[sensor] = 0
            self.pos[sensor] = 0
            self.prev_pos[sensor] = 0
            self.vel[sensor] = 0

    def set_labels(self, labels):
        self.sensors_labels = labels

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
        """set the scales for sensors"""
        pass

    def reset_device(self):
        """Reset sensor counters and disable the motor driver"""
        command = self.protocol["reset_counters"] + 7 * b"\x00"
        self.send_command(command)
        self.recive_reply()

    # ////////// Parsing raw data //////////

    def parse_sensor_data(self, reply):
        """parse the raw sensor data from the CAN frame"""
        for sensor in self.sensors_labels:
            first = self.bytes_mapping[sensor]['first']
            last = first + self.bytes_mapping[sensor]['bytes']
            self.raw_state_data[sensor] = self.from_bytes(
                reply[first:last], signed=False)

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
        # do mapping from sensor labels dict to
        # for scale in scales:
        #     self.sensors[sensor]['turns']
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

        for sensor in self.sensors_labels:
            # self.sensors[sensor] = {}
            self.counts[sensor] = raw_data[sensor]
            self.turns[sensor] += self.counter_overflow(
                self.counts[sensor], self.prev_counts[sensor], self.counter_buffer
            )
            self.prev_counts[sensor] = self.counts[sensor]
            self.pos[sensor] = (self.counts[sensor] +
                                self.turns[sensor]) * self.scale[sensor]
            self.vel[sensor] = (self.pos[sensor] - self.prev_pos[sensor]) / dt
            self.prev_pos[sensor] = self.pos[sensor]

        return self.state

    def set_zero(self, sensors=None):
        if None:
            sensors_to_zero = self.sensors_labels
        else:
            sensors_to_zero = sensors

        for sensor in sensors_to_zero:
            self.counts[sensor] = 0
            self.turns[sensor] = 0
