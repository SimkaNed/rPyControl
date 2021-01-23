# TODO:
# add rotation counter
# add low pass filtering of the cuirrent and velocity
# put state in dictionary
# add conversion from raw data to state
# add sensor scales

from time import perf_counter
from math import pi


# TODO:
#   implement error handler - sudden jumps in control effort
#   global ticker inside class, thread/process?
#   make helper class with sensor related functions
#


class TSA_Maxon:
    """ This class provide interface to the Gyems BLDC motor driver over CAN socket"""

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
            "motor_on": b"\x88",  # enable motor driver
            "motor_off": b"\x80",  # disable motor driver
            "set_current": b"\xA1",  # set motor current
            "reset_device": b"\x05",
        }

        self.command = self.protocol["motor_off"] + 7 * b"\x00"

        # TODO:
        # Define meanengfull params
        # Remove junk

        # \\\\\\ Limits \\\\\\

        self.current_limit = 3.3
        self.torque_limit = 60
        self.driver_current_lim = 5.0

        # \\\\\\ Offsets \\\\\
        self.current_offset = 2 ** 15 - 1
        self.force_offset = 0
        self.counter_buffer = 2 ** 16 - 1
        
        # \\\\\\ Scales \\\\\\
        self.current_scale = self.driver_current_lim / (2 ** 15 - 1)
        self.force_scale = 1
        self.torque_constant = 10.2
        self.lin_encoder_dpi = 360
        self.mot_encoder_cpt = 1024

        self.motor_status = ["on", "off", "error"]

        raw_state_labels = {
            "mot_turns",
            "mot_counts",
            "lin_counts",
            "current_counts",
            "force_counts",
        }

        self.raw_state_data = dict(zip(raw_state_labels, 5 * [0]))

        state_labels = [
            "time",
            "motor_pos",
            "motor_speed",
            "lin_pos",
            "lin_speed",
            "torque",
            "current",
            "tension",
        ]

        self.state = dict(zip(state_labels, 8 * [0]))
        
        # 
        self.init_time = perf_counter()
        
        # 
        self.current = 0
        self.torque = 0
        
        # 
        self.motor_counts = 0
        self.motor_turns = 0
        self.motor_pos = 0
        
        # 
        self.lin_counts = 0
        self.lin_turns = 0
        self.lin_pos = 0
        
        # 
        self.desired_angle = 0
        self.desired_torque = 0
        
        # 
        self.reply = 0
        self.time = 0
        self.dt = 0
        
        # 
        self.set_units()

        # self.lin_encoder_dpi

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

    # /////////////////////////////
    # ///// Commands Routines /////
    # /////////////////////////////

    def disable(self, clear_errors=True):
        """Set motor driver in disable mode"""
        command = self.protocol["motor_off"] + 7 * b"\x00"
        self.send_command(command)
        self.recive_reply()

    def enable(self, clear_errors=False):
        """Set motor driver in enable mode"""
        # if clear_errors:
        #     self.clear_errors()

        command = self.protocol["motor_on"] + 7 * b"\x00"
        self.send_command(command)
        self.recive_reply()

    def reset_device(self):
        """Reset encoder counters and disable the motor driver"""
        command = self.protocol["reset_device"] + 7 * b"\x00"
        self.send_command(command)
        self.recive_reply()

    # ///////////////////////////////////
    # ///// Sensor Related Routines /////
    # ///////////////////////////////////

    # ////////// Set Units //////////

    def set_degrees(self):
        """Set angle and speed scales for degrees"""
        self.angle_scale = 360 / (self.mot_encoder_cpt * 4)

    def set_radians(self):
        """Set radians for angle and speed scales"""
        self.angle_scale = 2 * pi / (self.mot_encoder_cpt * 4)

    def set_milimeters(self):
        """Set mm for linear position"""
        self.linear_scale = 25.4 / (self.lin_encoder_dpi * 4)

    def set_meters(self):
        """Set meters for linear position"""
        self.linear_scale = 25.4e-3 / (self.lin_encoder_dpi * 4)

    def set_units(self, motor_units="rad", linear_units="mm"):
        self.motor_units = motor_units
        self.linear_units = linear_units

        if motor_units == "rad":
            self.set_radians()
        else:
            self.motor_units = "deg"
            self.set_degrees()

        if linear_units == "mm":
            self.set_milimeters()
        else:
            self.linear_units = "meters"
            self.set_meters()

    # ////////// Parsing raw data //////////

    def parse_sensor_data(self, reply):
        """parse the raw sensor data from the CAN frame"""

        self.raw_state_data["mot_counts"] = self.from_bytes(reply[:2], signed=False)
        self.raw_state_data["lin_counts"] = self.from_bytes(reply[2:4], signed=False)
        self.raw_state_data["force_counts"] = self.from_bytes(reply[4:6])
        self.raw_state_data["current_counts"] = (reply[7] << 8) | reply[6]

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

    def get_sensor(self, data, scale=1, offset=0):
        """parse scaled and biased sensor"""
        return scale * (data - offset)

    def get_state(self, reply):
        """parse the motor state from CAN frame"""
        raw_data = self.parse_sensor_data(
            reply
        )  # parse the raw data to self.raw_state_data

        t = perf_counter() - self.init_time
        dt = t - self.time
        # print(dt)
        self.time = t

        self.state["time"] = self.time

        # Parse angular position and speed
        motor_counts = raw_data["mot_counts"]
        self.motor_turns += self.counter_overflow(
            motor_counts, self.motor_counts, self.counter_buffer
        )
        self.motor_counts = motor_counts
        self.motor_pos = (motor_counts + self.motor_turns) * self.angle_scale
        self.state["motor_speed"] = (self.motor_pos - self.state["motor_pos"]) / dt
        self.state["motor_pos"] = self.motor_pos

        # Parse linear position and speed
        lin_counts = raw_data["lin_counts"]
        self.lin_turns += self.counter_overflow(
            abs(lin_counts), abs(self.lin_counts), self.counter_buffer
        )
        self.lin_counts = lin_counts
        self.lin_pos = (lin_counts + self.lin_turns) * self.linear_scale
        self.state["lin_speed"] = (self.lin_pos - self.state["lin_pos"]) / dt
        self.state["lin_pos"] = self.lin_pos

        # Parse current, torque and tenstion
        self.state["current"] = self.get_sensor(
            raw_data["current_counts"], self.current_scale, self.current_offset
        )
        self.state["torque"] = self.torque_constant * self.state["current"]
        self.state["tension"] = self.get_sensor(
            raw_data["force_counts"], self.force_scale, self.force_offset
        )

        return self.state

    # ///////////////////////////
    # ///// Control Modes ///////
    # ///////////////////////////

    def float_to_int(self, float_to_convert, max_val, bits=16, signed=True):

        if signed:
            bits -= 1
        else:
            float_to_convert = abs(float_to_convert)

        if float_to_convert >= max_val:
            float_to_convert = max_val

        elif float_to_convert <= -max_val:
            float_to_convert = -max_val

        integer = int((2 ** bits - 1) * float_to_convert / max_val)
        return integer

    def limiter(self, value, limit):
        if value > limit:
            value = limit
        if value < -limit:
            value = -limit
        return value

    def set_current(self, current):
        """Send desired motor current to driver, and update the state"""
        # limit desired current
        self.desired_current = self.limiter(current, self.current_limit)
        #
        self.desired_current_int = self.float_to_int(
            self.desired_current, self.driver_current_lim
        )

        self.command = (
            self.protocol["set_current"]
            + self.to_bytes(2, self.desired_current_int)
            + 5 * b"\x00"
        )

        self.send_command(self.command)
        self.recive_reply()
        self.get_state(self.reply)

    def set_toruqe(self, torque):
        """Send desired motor torque to driver, and update the state"""
        if self.torque_limit:
            self.desired_current = self.limiter(
                torque / self.torque_constant, self.current_limit / self.torque_constant
            )

        self.set_current(self.desired_current)

    # \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
    # \\\\\\\\\\\ FUNCTIONS TO IMPLEMENT \\\\\\\\\\\\\\
    # \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

    def set_torque(self, torque, torque_limit=None):
        pass

    def set_zero(self):
        pass

    def set_speed(self):
        pass

    def set_angle(self):
        pass

    # Measurements
    def get_state(self):
        pass

    def set_current_limit(self):
        pass

    def set_torque_limit(self):
        pass

    def initialize_setup(self):
        pass

    def calibrate_force_sensor(self):
        pass

    def get_vel(self):
        pass

    def get_angle(self):
        pass

    def get_pos(self):
        pass

    # ///////////////
    # //// JUNK /////
    # ///////////////

    # TODO: write error handler functions
    # def clear_errors(self):
    #     command = self.protocol['clear_error_flags'] + 7*b'\x00'
    #     self.send_command(command)
    #     self.recive_reply()

    # Turn motor modes
    # def pause(self, clear_errors = False):
    #     if clear_errors:
    #         self.clear_errors()

    #     command = self.protocol['motor_stop'] + 7*b'\x00' # message = {0x141:  b'\x81\x00\x00\x00\x00\x00\x00\x00'}
    #     self.send_command(command)
    #     self.recive_reply()

    # def reset(self, go_to_zero = False):
    #     self.disable(clear_errors = True)
    #     self.enable()

    # def go_to_zero(self):
    #     """Go to the specific point and set new zero at this point"""
    #     pass

    # def set_as_zero(self):
    #     """Go to the specific point and set new zero at this point"""
    #     pass
