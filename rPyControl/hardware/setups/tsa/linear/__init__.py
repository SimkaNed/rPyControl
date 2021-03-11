from ....actuators.gyems import GyemsDRC
from ....sensors.can_sensors import CANSensors
from time import perf_counter, sleep
from math import pi
from multiprocessing import Process, Value, Event
from os import nice

# TODO:
#   implement error handler - sudden jumps in control effort
#   global ticker inside class, thread/process?
#   make helper class with sensor related functions
#   make a two processes, one for motor another for sensors

class LinearSetup:
    """ This class provide interface to the Gyems BLDC motor driver over CAN socket"""

    def __init__(
        self,
        motors_bus=None,
        sensors_bus=None,
        motor_id=0x141,
        sensor_id=0x01,
    ):

        self.sensor_labels = ['lin', 'frc']

        self.sensors = CANSensors(can_bus = sensors_bus, labels = self.sensor_labels)
        self.sensors.data_map = {'lin':[2, 4], 'frc':[4, 6]}
        self.sensors.scales = {'lin':25.4 / (360 * 4), 'frc':0.166}
        self.sensors.offsets = {'lin':0, 'frc':1764}
        self.sensors.reset_counters(['lin'])
        self.sensors.set_differences({'lin'})

        self.actuator = {"motor": GyemsDRC(can_bus=motors_bus, device_id=motor_id),
                         "limit": 2000,
                         "angle_offset": 0,
                         "pos_offset": 0,
                         }

        self.actuator["motor"].reset()
        self.actuator["motor"].set_radians()
        self.actuator["motor"].current_limit = self.actuator["limit"]
        self.actuator["control"] = Value("d", 0)
        self.actuator["torque_constant"] = 1

        self.sensors_data = {}
        self.state = {}

        self.state_labels = {"theta", "dtheta", "x", "dx", "current", "force"}
        self.state = {}

        for state in self.state_labels:
            self.sensors_data[state] = Value("d", 0)
            self.state[state] = 0

        # Dict to store system params
        # Maybe we need to store related kinematics and dynamics there
        self.parameters = {}
        self.to_home = True

        # Array to store the processes
        self.processes = []

        self.exit_event = Event()
        self.exit_event.clear()


        


    def __del__(self):
        
        self.stop()
        print("Linear setup was deleted from memory")

    def set_torque_constant(self, constant):
        # TODO: move the actuator class
        self.actuator["torque_constant"] = constant

    def run(self):
        """Run the sensing and motor processes"""
        self.processes.append(
            Process(target=self.sensing_process, args=(self.exit_event,))
        )
        self.processes.append(
            Process(target=self.motor_process, args=(self.exit_event,))
        )
        print("Processes are about to start...")
        for process in self.processes:
            process.start()

        if self.exit_event.is_set():
            print('test')
            for process in self.processes:
                process.join()

    def enable(self):
        # for actuator in self.actuators_labels:
        self.actuator["motor"].enable()

    def disable(self):
        # for actuator in self.actuators_labels:
        self.actuator["motor"].disable()

    def stop(self, delay = 0.0):
        sleep(delay)
        print("Processes are about to stop...")
        self.exit_event.set()
        # self.disable()
        if self.processes:
            for process in self.processes:
                process.terminate()
        print("Processes are terminated...")


    def sensing_process(self, exit_event):
        print("Sensing procces is launched")
        while True:
            try:
                self.sensors.request_reply()
                self.sensors.parse_data()
                self.sensors_data["x"].value = self.sensors.data['lin']
                self.sensors_data["dx"].value = self.sensors.differences['lin']
                self.sensors_data["force"].value = self.sensors.data['frc']

                # TODO: think how to add everything here 

                # self.sensors_data["theta"].value = (
                #     self.actuator["motor"].state["angle"]
                #     - self.actuator["angle_offset"]
                # )
                # self.sensors_data["dtheta"].value = self.actuator["motor"].state["speed"]
                # self.sensors_data["current"].value = self.actuator["motor"].state["torque"]
            except KeyboardInterrupt:
                exit_event.set()
                print("Exit sensing process")


    def motor_process(self, exit_event):
        print("Motor procces is launched")
        # for actuator in self.actuators_labels:
        self.actuator["motor"].enable()
        try:
            t0 = perf_counter()
            while True:
                t = perf_counter() - t0
                # for actuator in self.actuators_labels:
                u = self.actuator["control"].value
                self.actuator["motor"].set_current(u)

                self.sensors_data["theta"].value = (
                    self.actuator["motor"].state["angle"]
                    - self.actuator["angle_offset"]
                )
                self.sensors_data["dtheta"].value = self.actuator["motor"].state["speed"]
                self.sensors_data["current"].value = self.actuator["motor"].state["torque"]

        except KeyboardInterrupt:
            if self.to_home:
                while abs(self.actuator["motor"].state["angle"])>0.1:
                    self.actuator["motor"].set_angle(0, speed_limit = 10000)
                sleep(0.5)
            self.actuator["motor"].disable()
            exit_event.set()
            print("Exit motor process")

    def get_state(self):
        for state in self.state_labels:
            self.state[state] = self.sensors_data[state].value
        return self.state

    def set_control(self, control):
        """Update the value for controller with arguments"""
        # TODO: think on what is appropriate argument
        # for actuator in self.actuators_labels:
        self.actuator["control"].value = control

    # def init_setup(self):
    #     """Initialize setup"""
    #     print(f'\n {10*"*"} Initialization of encoders offsets... {10*"*"}\n')

    #     # Calibrate force sensor
    #     # Estimate kinematics
    #     # Find offsets

    #     print(self.init_offsets(speed=30, max_contraction=5, iters=5))

    #     print(f'\n {10*"*"} Initialization finished! {10*"*"}\n')

    # def init_offsets(self, speed=50, max_contraction=5, iters=2):
    #     """Initialize zero of motor and linear encoder by reference motion"""

    #     actuator = self.actuator["motor"]
    #     actuator.set_zero()

    #     max_reached = False
    #     pos, pos_incr, pos_offset, angle_offset = 0, 0, 0, 0
    #     n_iter = 0

    #     des_speed = speed

    #     # reset counter for specific linear encoder
    #     # self.encoders.set_zero(encoders={1})


    #     actuator.enable()

    #     while n_iter < iters:
    #         actuator.set_speed(des_speed)
    #         angle = actuator.state["angle"]

    #         self.sensors.request_reply()
    #         self.sensors.parse_data()
    #         pos_incr = self.sensors.data['lin']

    #         pos = pos_incr

    #         if not max_reached and pos > max_contraction:
    #             des_speed = -des_speed
    #             max_reached = True
    #             n_iter += 1

    #         if pos_offset >= pos_incr:
    #             # self.sensors.reset_counters(['lin'])
    #             pos_offset = pos_incr
    #             angle_offset = angle
    #             max_reached = False
           
    #     actuator.set_current(0)
    #     actuator.disable()

    #     # self.actuator["angle_offset"] = angle_offset
    #     # self.actuator["pos_offset"] = pos_offset

    #     return pos_offset, angle_offset





class LinearMaxon(CANSensors):
    """ This class provide interface to the Gyems BLDC motor driver over CAN socket"""

    def __init__(
        self,
        can_bus=None,
        device_id=0x01,
        freq = 250,
    ):

        self.sensor_labels = ['lin', 'ang', 'frc', 'cur']
        super().__init__(can_bus = can_bus, device_id = device_id, labels = self.sensor_labels)
        # sleep(0.1)
        self.protocol = dict()
        self.protocol = {
            "motor_on": b"\x88",  # enable motor driver
            "motor_off": b"\x80",  # disable motor driver
            "set_current": b"\xA1",  # set motor current
            "reset_device": b"\x05",
            "get_sensors": b"\x9C",
            # "reset_counters": b"\x05",
        }
        self.disable()
        self.reset_device()

        # self.sensors = CANSensors(can_bus = bus, labels = self.sensor_labels)
        self.data_map = {'ang':[0, 2], 'lin':[2, 4], 'frc':[4, 6], 'cur':[6,8]}
        # self.overflow_buffer = {'ang':2 ** 16 - 1, 'lin':2 ** 16 - 1, 'frc':2 ** 16 - 1, 'cur':2 ** 16 - 1}
        self.enable_overflow({'ang':2 ** 16 - 1, 'lin':2 ** 16 - 1, 'cur':2 ** 16 - 1})
        
        self.scales = {'ang':2*pi/(1024*4),'lin':25.4 / (360 * 4), 'frc':0.166,'cur':5/(2 ** 15 - 1)}
        self.offsets = {'ang':0, 'lin':0, 'frc':0,'cur':-(2 ** 15 - 1)}
        # self.reset_counters(sensors = {'ang', 'lin'})
        
        self.set_differences({'ang', 'lin'})



        self.process_state = {}
        self.state = {}

        self.state_labels = {"time","theta", "dtheta", "x", "dx", "current", "force", "torque"}
        self.state = {}

        self.control = Value("d", 0)
        for state in self.state_labels:
            self.process_state[state] = Value("d", 0)
            self.state[state] = 0

        self.parameters = {}
        self.to_home = True

        # Array to store the processes
        self.processes = []

        self.exit_event = Event()
        self.exit_event.clear()

        self.T = 1/freq


        self.current_limit = 3.3
        self.torque_limit = 60
        self.driver_current_lim = 5.0

        # \\\\\\ Offsets \\\\\
        self.current_offset = 2 ** 15 - 1
        self.force_offset = 0
        self.counter_buffer = 2 ** 16 - 1
        
        # \\\\\\ Scales \\\\\\
        self.current_scale = self.driver_current_lim / (2 ** 15 - 1)
        self.torque_constant = 10.2


    def __del__(self):
        # self.stop()
        print("Linear setup was deleted from memory")

    def set_torque_constant(self, constant):
        # TODO: move the actuator class
        self.actuator["torque_constant"] = constant

    def run(self):
        """Run the sensing and motor processes"""
        # self.is_enabled = enable
        self.processes.append(
            Process(target=self.setup_process, args=(self.exit_event,))
        )
        print("Processes are about to start...")
        for process in self.processes:
            process.start()

        if self.exit_event.is_set():
            print('test')
            for process in self.processes:
                process.join()

    def stop(self, delay = 0.0, to_zero = True):

        if to_zero:
            self.to_zero()
            

        self.disable()
        # self.set_control(0)
        sleep(delay)
        print("Processes are about to stop...")
        self.exit_event.set()
        if self.processes:
            for process in self.processes:
                process.terminate()
        print("Processes are terminated...")


    def to_zero(self, T = 2, kp = 0.6, kd = 0.08, max_torque = 10, theta_0 = 0.05):
        t0 = perf_counter()
        while True:
            t = perf_counter() - t0
            state = self.get_state()
            theta, dtheta = state['theta'], state['dtheta']

            if abs(theta)< 6:
                kp = 6
            u = -kp*theta - kd*dtheta
            
            u = self.limiter(u, max_torque)
            self.set_control(u)

            if T:
                if t>= T:
                    break

            if abs(theta) <= theta_0:
                break
        # print('Returned to zero')
        

    def setup_process(self, exit_event):
        print("Motor procces is launched")
        # for actuator in self.actuators_labels:
        try:
            # if self.is_enabled:
            #     self.enable()
            
            t0 = perf_counter()
            tc = 0
            while True:
                t = perf_counter() - t0
                # for actuator in self.actuators_labels:

                if (t - tc)>self.T:
                    tc = t 
                    u = self.control.value
                    self.set_torque(u, torque_limit = self.torque_limit)

                    # self.request_reply()
                    self.parse_data()
                    self.process_state["time"].value = t
                    self.process_state["theta"].value = self.measurements['ang']
                    self.process_state["dtheta"].value = self.differences['ang']
                    self.process_state["x"].value = self.measurements['lin']
                    self.process_state["dx"].value = self.differences['lin']
                    self.process_state["force"].value = self.measurements['frc']
                    self.process_state["current"].value = self.measurements['cur']
                    self.process_state["torque"].value = self.torque_constant*self.measurements['cur']

                # for state_label in self.state_labels:
                #     self.sensors_data[state_label].value = state[state_label]

    # def parse_sensors()

        except KeyboardInterrupt:
            # if self.to_home:
            #     while abs(self.actuator["motor"].state["angle"])>0.1:
            #         self.actuator["motor"].set_angle(0, speed_limit = 10000)
            #     sleep(0.5)
            self.disable()
            exit_event.set()
            print("Exit motor process")


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


    def get_state(self):
        for state in self.state_labels:
            self.state[state] = self.sensors_data[state].value
        return self.state


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
                    self.state[state_label] = self.process_state[state_label].value

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
        # self.parse_state(self.reply)


    def set_torque(self, torque, torque_limit = None):
        """Send desired motor torque to driver, and update the state"""
        if torque_limit:
            self.desired_current = self.limiter(
                    torque / self.torque_constant, torque_limit / self.torque_constant
                )
        else:
            self.desired_current = torque / self.torque_constant


        self.set_current(self.desired_current)


    def set_control(self, control):
        """Update the value for controller with arguments"""
        # TODO: think on what is appropriate argument
        # for actuator in self.actuators_labels:
        self.control.value = control