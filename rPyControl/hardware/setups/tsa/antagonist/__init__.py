from ....actuators.gyems import GyemsDRC
from ....sensors.can_sensors.encoders import IncrementalEncoders
from time import perf_counter, sleep
from math import pi
from multiprocessing import Process, Value, Event
from os import nice

# TODO:
#   implement error handler - sudden jumps in control effort
#   global ticker inside class, thread/process?
#   make helper class with sensor related functions
#   implement different mode in motor processes:
#           for position, speed, torque controls 
#  initialization trough the motor porcess

class LinearAntagonist:
    """ This class provide interface to the Gyems BLDC motor driver over CAN socket"""

    def __init__(
        self,
        motors_bus=None,
        sensors_bus=None,
        motor_id={1: 0x141, 2: 0x142},
        sensor_id=0x01,
    ):
        # create object to store incremental encoders
        self.encoders = IncrementalEncoders(can_bus=sensors_bus)
        # reset the counter
        # self.encoders.reset_device()
        # Set the encoder scales (counts -> mm)
        self.encoder_scales = {1: -25.4 / (360 * 4), 2: 25.4 / (360 * 4)}

        self.sensors = {}

        #
        self.actuators = {}
        self.actuators_labels = {1, 2}
        #
        self.state = {}
        self.state_labels = {"theta", "dtheta", "x", "dx", "current"}

        # Fill the dictionary with actuators
        for actuator in self.actuators_labels:
            self.actuators[actuator] = {
                "motor": GyemsDRC(can_bus=motors_bus, device_id=motor_id[actuator]),
                # 'linear_sensor':
                "limit": 2000,
                "angle_offset": 0,
                "pos_offset": 0,
            }
            self.actuators[actuator]["motor"].reset()
            self.actuators[actuator]["motor"].set_radians()
            self.actuators[actuator]["motor"].current_limit = self.actuators[actuator][
                "limit"
            ]
            self.actuators[actuator]["control"] = Value("d", 0)

            self.sensors[actuator] = {}
            self.state[actuator] = {}
            
            for state in self.state_labels:
                self.sensors[actuator][state] = Value("d", 0)
                self.state[actuator][state] = 0
        
        # Array to store the processes
        self.processes = []

        self.exit_event = Event()
        self.exit_event.clear()

    def __del__(self):

        self.stop()
        print("Antagonist was deleted from memory")

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

    def disable(self):
        for actuator in self.actuators_labels:
            self.actuators[actuator]["motor"].disable()

    def stop(self):
        print("Processes are about to stop...")
        self.exit_event.set()
        self.disable()
        if self.processes:
            for process in self.processes:
                process.terminate()
        print("Processes are terminated...")

    def sensing_process(self, exit_event):
        print("Sensing procces is launched")
        while True:
            try:
                self.encoders.get_state()
                for actuator in self.actuators_labels:
                    self.sensors[actuator]["x"].value = (
                        self.encoder_scales[actuator] * self.encoders.pos[actuator]
                        - self.actuators[actuator]["pos_offset"]
                    )
                    self.sensors[actuator]["dx"].value = self.encoders.vel[actuator]
            except KeyboardInterrupt:
                exit_event.set()
                print("Exit sensing process")

    def motor_process(self, exit_event):
        print("Motor procces is launched")
        for actuator in self.actuators_labels:
            self.actuators[actuator]["motor"].enable()
        try:
            t0 = perf_counter()
            while True:
                t = perf_counter() - t0
                for actuator in self.actuators_labels:
                    u = self.actuators[actuator]["control"].value
                    self.actuators[actuator]["motor"].set_current(u)

                    self.sensors[actuator]["theta"].value = (
                        self.actuators[actuator]["motor"].state["angle"]
                        - self.actuators[actuator]["angle_offset"]
                    )
                    self.sensors[actuator]["dtheta"].value = self.actuators[actuator][
                        "motor"
                    ].state["speed"]
                    self.sensors[actuator]["current"].value = self.actuators[actuator][
                        "motor"
                    ].state["torque"]

        except KeyboardInterrupt:
            for actuator in self.actuators_labels:
                self.actuators[actuator]["motor"].disable()
                exit_event.set()
                print("Exit motor process")

    def get_state(self):
        for actuator in self.actuators_labels:
            for state in self.state_labels:
                self.state[actuator][state] = self.sensors[actuator][state].value
        return self.state

    def set_control(self, control):
        """Update the value for controller with arguments"""
        # TODO: think on what is appropriate argument
        for actuator in self.actuators_labels:
            self.actuators[actuator]["control"].value = control[actuator]

    def init_setup(self):
        """Initialize encoders offsets"""

        print(f'\n {10*"*"} Initialization of encoders offsets... {10*"*"}\n')
        
        self.encoders.reset_device()

        for init_actuator in self.actuators_labels:
            # sleep(0.5)
            other_actuators = self.actuators_labels.difference({init_actuator})

            self.init_actuator(init_actuator, other_actuators)
        
        # sleep(0.5)        

        print(f'\n {10*"*"} Initialization finished! {10*"*"}\n')


    def init_actuator(self, actuator_label, other_actuators, speed=100, max_contraction=5, iters=2):
        # TODO:
        # DEBUG, MOVE TO THE MP
        """Initialize zero of motor and linear encoder by reference motion"""

        print(f"Initialization of actuator {actuator_label}...")

        actuator = self.actuators[actuator_label]["motor"]
        actuator.set_zero()

        max_reached = False
        pos, pos_incr, pos_offset, angle_offset = 0, 0, 0, 0
        n_iter = 0

        des_speed = speed

        # reset counter for specific linear encoder
        self.encoders.set_zero(encoders = {actuator_label})
        for tension_actuator in other_actuators:
            self.actuators[tension_actuator]["motor"].enable()
        actuator.enable()
        
        while n_iter <= iters:
            actuator.set_speed(des_speed)
            
            for tension_actuator in other_actuators:
                self.actuators[tension_actuator]["motor"].set_current(40)

            angle = actuator.state["angle"]

            self.encoders.get_state()

            pos_incr = (
                self.encoders.pos[actuator_label] * self.encoder_scales[actuator_label]
            )

            pos = pos_incr - pos_offset

            if not max_reached and pos > max_contraction:
                des_speed = -des_speed
                max_reached = True
                n_iter += 1

            # check either linear sensor data decreasing or not
            if pos_offset > pos_incr:
                pos_offset = pos_incr
                angle_offset = angle

                # actuator.set_zero()

                max_reached = False

        actuator.disable()

        for tension_actuator in other_actuators:
            # self.actuators[tension_actuator]["motor"].set_current(0)
            self.actuators[tension_actuator]["motor"].disable()

        self.actuators[actuator_label]["angle_offset"] = angle_offset
        self.actuators[actuator_label]["pos_offset"] = pos_offset

        print(f"Actuator {actuator_label} initialized...")

        return pos_offset, angle_offset

