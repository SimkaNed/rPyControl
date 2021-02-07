from multiprocessing import Process, Value, Lock 

# TODO:
# Create a setup from sensors and actuators instances
# One should pass the get_sensor() and set_controller() methods, as well as init_function()
# 
#  
# checking sensors and setting controller should be done in separate processes
# 

class Setup:
    def __init__(self, sensors = {}, actuators = {},  name = ''):
        self.name = name

        self.sensors = sensors
        self.actuators = actuators 
        self.processes = []
        self.processes.append(Process(target = self.sensing_process))
        self.processes.append(Process(target = self.actuation_process))
        self.state = {}

        for process in self.processes:
            process.start()
        print(f'{self.name} setup was constructed')

    def build(self):
        """Build the setup from components"""
        pass

    def get_sensors(self):
        """Get sensors """
        pass
        
    def sensors_to_state(self, sensors):
        """Convert the sensors from the state"""
        pass

    def actuation_process(self):
        """ """
        pass

    def sensing_process(self):
        # do some shit
        # copy sensor data to shared memory
        #  
        pass

    def get_state(self):
        # copy sensors data to the sensors objects
        # self.state = self.sensors_to_state()
        pass

    def set_control(self):
        pass

    def run(self):
        """Run the setup"""
        pass

    def stop(self):
        """Stop the setup"""
        pass