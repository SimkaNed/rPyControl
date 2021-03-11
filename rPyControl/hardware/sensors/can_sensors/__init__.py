from time import perf_counter
from math import pi
from ...can import CANDevice
from .._routines import counter_overflow
# CANSensors is class that provide interface between

# TODO:

# Implement filtering
# Implement differences 
# Implement integration
# Implement overflow


class CANSensors(CANDevice):
    """This class map CAN messages coming from specific bus encoders through CAN bus"""
    # pass

    def __init__(self, can_bus=None, device_id=0x01, labels=None, sensors_params={}):

        super().__init__(can_bus = can_bus)
        
        self.labels = labels

        if not self.labels:
            print("Labels are not provided!!!\n Do you want to initialize sensors object with one sensor named 'sens'?\n Y - for yes")
            ans = input()
            if ans == 'Y' or 'y':
                self.labels = ['sens']

        self.counts, self.scales, self.offsets, self.overflow_buffer, self.data_map, self.differences, self.filters = {}, {}, {}, {}, {}, {}, {}
        self.bytes, self.data, self.turns = {}, {}, {}
        self.timestamps, self.buffer_size = {}, {}
        self.measurements = {}
        for index, sensor in enumerate(self.labels):
            self.bytes[sensor] = 0
            self.measurements[sensor] = 0
            self.counts[sensor] = [0]
            self.timestamps[sensor] = [0]
            self.differences[sensor] = 0
            self.scales[sensor] = 1
            self.offsets[sensor] = 0
            self.data_map[sensor] = [index*2, (index + 1)*2]
            self.data[sensor] = [0]
            self.overflow_buffer[sensor] = 0
            self.turns[sensor] = 0
            self.filters[sensor] = None
            self.buffer_size[sensor] = 1

        self.protocol = {
            "init_sensors":b"\x9B",
            "get_sensors": b"\x9C",
            "reset_counters": b"\x05",
        }

        self.raw = {}

        self.init_time = perf_counter()

    # TODO: Implement the functions that will facilitate the sensor configuration

    def set_scales(self):
        pass
    
            # self.offsets[sensor] = self.counts[sensor]
            # self.turns[sensor] = 0

    def init_arrays(self, sensor):
        self.counts[sensor] = self.buffer_size[sensor]*[0]
        self.timestamps[sensor] = self.buffer_size[sensor]*[0]
        self.data[sensor] = self.buffer_size[sensor]*[0]

    def set_differences(self, sensors):
        # sensors_to_diff = sensors
        for sensor in sensors:
            if self.buffer_size[sensor] < 2:
                self.buffer_size[sensor] = 2
                self.init_arrays(sensor)
            self.differences[sensor] = 0
            self.init_arrays(sensor)


    def enable_overflow(self, buffers):
        sensors_to_overflow = buffers.keys()
        for sensor in sensors_to_overflow:
            if self.buffer_size[sensor] < 2:
                self.buffer_size[sensor] = 2
                self.init_arrays(sensor)
            self.overflow_buffer[sensor] = buffers[sensor]
            # print(self.overflow_buffer[sensor])


    def request_reply(self, command = None):
        self.command = self.protocol["get_sensors"] + 7*b"\x00"
        self.execute()                                                                                                                        
        pass
    

    def parse_data(self):

        for sensor in self.labels:

            for i in range(1,self.buffer_size[sensor]):
                self.timestamps[sensor][i] = self.timestamps[sensor][i-1]
                self.counts[sensor][i] = self.counts[sensor][i-1]
                self.data[sensor][i] = self.data[sensor][i-1]
                # self.differences[sensor][i] = self.differences[sensor][i-1]

            self.bytes[sensor] = self.reply[self.data_map[sensor][0]:self.data_map[sensor][1]]
            self.counts[sensor][0] = self.from_bytes(self.bytes[sensor])
            self.timestamps[sensor][0] = perf_counter() - self.init_time
            
                        
            if self.overflow_buffer[sensor]:
                # print(sensor)
                self.turns[sensor] += counter_overflow(self.counts[sensor][0], self.counts[sensor][1], self.overflow_buffer[sensor])
                # print(self.turns[sensor])
            self.data[sensor][0] = self.scales[sensor]*(self.turns[sensor]*self.overflow_buffer[sensor] + self.counts[sensor][0] - self.offsets[sensor])
      
            if self.buffer_size[sensor]>1: 
                dt = self.timestamps[sensor][0] - self.timestamps[sensor][1]
                self.differences[sensor] = (self.data[sensor][0] - self.data[sensor][1])/dt
      
            if self.filters[sensor]:
                # do the filtering
                pass    

            self.measurements[sensor] = self.data[sensor][0] 


    def reset_counters(self, sensors=None, output = False):
        """Reset counters for specific sensors provided by their labels"""

        if sensors is None:
            sensors_to_reset = self.labels
        else:
            sensors_to_reset = sensors
        
        print(sensors_to_reset)

        self.request_reply()
        self.parse_data()

        for sensor in sensors_to_reset:
            self.offsets[sensor] = self.counts[sensor][0]
            # print(sensor, self.counts[sensor][0])
            self.turns[sensor] = 0

        if output:
            print(f'New offsets are setted as:\n{self.offsets}')


