from time import perf_counter, sleep
from math import pi
from .....can import CANDevice
from multiprocessing import Value, Process

# TODO:
#   implement error handler - sudden jumps in control effort
#   global ticker inside class, thread/process?
#   make helper class with sensor related functions
#   use CANSensors class to work with sensors


class LinearMaxon(CANDevice):
    """ This class provide interface to the Gyems BLDC motor driver over CAN socket"""

    # def __init__(self, can_bus=None, device_id=0x01):

    def __init__(self, can_bus=None, device_id=0x01, units="rad", freq = 250):
        super().__init__(can_bus = can_bus, device_id = device_id)
    
        self.protocol = dict()
        self.protocol = {
            "motor_on": b"\x88",  # enable motor driver
            "motor_off": b"\x80",  # disable motor driver
            "set_current": b"\xA1",  # set motor current
            "reset_device": b"\x05",
        }

        self.command = self.protocol["motor_off"] + 7 * b"\x00"


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
        self.freq = 250

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
        self.angle_offset = 0
        self.linear_offset = 0
        # 
        self.set_units()
        self.processes = []

        self.state_labels = {"theta", "dtheta", "x", "dx", "current", "torque", "force"}
        self.state = {}
        self.sensors_data = {}
        self.freq = freq
        self.T = 1/self.freq

        self.control = Value("d", 0)
        for state in self.state_labels:
            self.sensors_data[state] = Value("d", 0)
            self.state[state] = 0

        # self.lin_encoder_dpi

    # ///////////////////////////
    # /// Conversion routines ///
    # ///////////////////////////

    # /////////////////////////////
    # ///// Commands Routines /////
    # /////////////////////////////



    def __del__(self):
        # self.stop()
        print("Linear setup was deleted from memory")


    # def run(self):
    #     """Run the sensing and motor processes"""
        
    #     self.processes.append(
    #         Process(target=self.setup_process)
    #     )
        
    #     print("Processes are about to start...")
    #     for process in self.processes:
    #         process.start()


    # def stop(self, delay = 0.0):
    #     sleep(delay)
    #     print("Processes are about to stop...")
    #     # self.exit_event.set()
    #     # self.disable()
    #     if self.processes:
    #         for process in self.processes:
    #             process.terminate()
    #     print("Processes are terminated...")


    def setup_process(self):
        try:
            self.enable()
            t0 = perf_counter()
            tc = 0
            while True:
                t = perf_counter() - t0
                
                if (t - tc)>self.T:
                    tc = t 
                    u = self.control.value
                    self.set_torque(u)

                    state = self.parse_state(self.reply)
                # print(state)

                    for state_label in self.state_labels:
                        self.sensors_data[state_label].value = state[state_label]
                    
        except KeyboardInterrupt:
        #     if self.to_home:
        #         while abs(self.actuator["motor"].state["angle"])>0.1:
        #             self.actuator["motor"].set_angle(0, speed_limit = 10000)
        #         sleep(0.5)
            self.disable()
        #     exit_event.set()
            print("Exit motor process")

    def run(self):
        """Run the sensing and motor processes"""
        self.processes.append(
            Process(target=self.setup_process)
        )
        print("Processes are about to start...")
        for process in self.processes:
            process.start()

        # if self.exit_event.is_set():
        #     print('test')
        #     for process in self.processes:
        #         process.join()


    def stop(self, delay = 0.0):
        sleep(delay)
        print("Processes are about to stop...")
        # self.exit_event.set()
        # self.disable()
        if self.processes:
            for process in self.processes:
                process.terminate()
        print("Processes are terminated...")


        # 

    def disable(self, clear_errors=True):
        """Set motor driver in disable mode"""
        self.command = self.protocol["motor_off"] + 7 * b"\x00"
        self.execute()


    def enable(self, clear_errors=False):
        """Set motor driver in enable mode"""
        self.command = self.protocol["motor_on"] + 7 * b"\x00"
        self.execute()

    def reset_device(self):
        """Reset encoder counters and disable the motor driver"""
        self.command = self.protocol["reset_device"] + 7 * b"\x00"
        self.execute()

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


    def get_sensor(self, data, scale=1, offset=0):
        """parse scaled and biased sensor"""
        return scale * (data - offset)

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

    def parse_state(self, reply):
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
        self.motor_pos = (motor_counts + self.motor_turns) * self.angle_scale - self.angle_offset
        self.state["dtheta"] = (self.motor_pos - self.state["theta"]) / dt
        self.state["theta"] = self.motor_pos

        # Parse linear position and speed
        lin_counts = raw_data["lin_counts"]
        self.lin_turns += self.counter_overflow(
            abs(lin_counts), abs(self.lin_counts), self.counter_buffer
        )
        self.lin_counts = lin_counts
        self.lin_pos = (lin_counts + self.lin_turns) * self.linear_scale - self.linear_offset
        self.state["dx"] = (self.lin_pos - self.state["x"]) / dt
        self.state["x"] = self.lin_pos

        # Parse current, torque and tenstion
        self.state["current"] = self.get_sensor(
            raw_data["current_counts"], self.current_scale, self.current_offset
        )
        self.state["torque"] = self.torque_constant * self.state["current"]
        self.state["force"] = self.get_sensor(
            raw_data["force_counts"], self.force_scale, self.force_offset
        )

        return self.state

    # def 

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

    def get_state(self):

        for state_label in self.state_labels:
                    self.state[state_label] = self.sensors_data[state_label].value

        return self.state


    def set_control(self, control):
        self.control.value = control

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
        self.execute()
        self.parse_state(self.reply)


    def set_torque(self, torque):
        """Send desired motor torque to driver, and update the state"""
        if self.torque_limit:
            self.desired_current = self.limiter(
                torque / self.torque_constant, self.current_limit
            )

        self.set_current(self.desired_current)