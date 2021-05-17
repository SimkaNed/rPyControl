from ....actuators.gyems import GyemsDRC
from ....sensors.can_sensors import CANSensors
from time import perf_counter, sleep
from math import pi
from multiprocessing import Process, Value, Event
from os import nice



class Pendulum:
    """ This class provide interface to the Gyems BLDC motor driver over CAN socket"""

    def __init__(
        self,
        can_bus=None,
        device_id=0x141,
        freq = 250,
    ):

        self.actuator = {"motor": GyemsDRC(can_bus=can_bus, device_id=device_id),
                         "limit": 2000,
                         "angle_offset": 0,
                         "pos_offset": 0,
                         }
        self.actuator["motor"].reset()
        # print('test')
        self.actuator["motor"].set_radians()
        self.actuator["motor"].current_limit = self.actuator["limit"]
        self.actuator["control"] = Value("d", 0)
        self.actuator["torque_constant"] = 1

        self.sensors_data = {}
        self.state = {}

        self.state_labels = {"time", "theta", "dtheta", "current", "torque", "temp"}
        self.state = {}

        for state in self.state_labels:
            self.sensors_data[state] = Value("d", 0)
            self.state[state] = 0

        self.parameters = {}
        self.to_home = True
        self.T = 1/freq

        # Array to store the processes
        self.processes = []

        self.exit_event = Event()
        self.exit_event.clear()        


    def __del__(self):
        
        self.stop()
        print("Pendulum setup was deleted from memory")

    def set_torque_constant(self, constant):
        # TODO: move the actuator class
        self.actuator["torque_constant"] = constant

    def run(self):
        """Run the sensing and motor processes"""
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
        self.actuator["motor"].enable()

    def disable(self):
        self.actuator["motor"].disable()

    def stop(self, delay = 0.0):
        sleep(delay)
        print("Processes are about to stop...")
        self.exit_event.set()
        if self.processes:
            for process in self.processes:
                process.terminate()
        print("Processes are terminated...")


    def motor_process(self, exit_event):
        print("Motor procces is launched")
        self.actuator["motor"].enable()
        
        try:
            
            t0 = perf_counter()
            tc = 0

            while True:
                t = perf_counter() - t0
                if (t - tc)>=self.T:
                    tc = t 
                    u = self.actuator["control"].value
                    self.actuator["motor"].set_current(u)
                    
                    self.sensors_data["time"].value = t
                    self.sensors_data["theta"].value = (self.actuator["motor"].state["angle"] - self.actuator["angle_offset"])
                    self.sensors_data["dtheta"].value = self.actuator["motor"].state["speed"]
                    self.sensors_data["current"].value = self.actuator["motor"].state["torque"]
                    self.sensors_data["temp"].value = self.actuator["motor"].state["temp"]

                # self.sensors_data["current"].value = self.actuator["motor"].state["torque"]

        except KeyboardInterrupt:
            if self.to_home:
                self.to_zero()
            self.actuator["motor"].disable()
            exit_event.set()
            print("Exit motor process")

    def to_zero(self):
        while abs(self.actuator["motor"].state["angle"])>0.05:
            self.actuator["motor"].set_angle(0, speed_limit = 200)
        # sleep(0.5)
        pass

    def get_state(self):
        for state in self.state_labels:
            self.state[state] = self.sensors_data[state].value
        return self.state

    def set_control(self, control):
        """Update the value for controller with arguments"""
        # TODO: think on what is appropriate argument
        # for actuator in self.actuators_labels:
        self.actuator["control"].value = control
