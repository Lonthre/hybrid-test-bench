
# Configure python path to load hybrid test bench modules
import sys
import os
import logging
import logging.config
import time
import numpy as np
from math import *

from scipy.integrate import solve_ivp

# Get the current working directory. Should be hybrid-test-bench
current_dir = os.getcwd()

assert os.path.basename(current_dir) == 'hybrid-test-bench', 'Current directory is not hybrid-test-bench'

# Get the parent directory. Should be the root of the repository
parent_dir = current_dir

from communication.server.rabbitmq import Rabbitmq
from communication.shared.protocol import ROUTING_KEY_STATE, ROUTING_KEY_FORCES, ROUTING_KEY_DISPLACEMENT
import pt_model as pt_model
# import dt_model as dt_model
import calibration_service as cal_service
import actuator_controller as ac_ctrl
import RainFlowCycleAlgorithm as rfca

# Define the global variables for the model
fx, fy, fz, mx, my, mz = 1, 2, 3, 4, 5, 6 # force and moment indices

class PTEmulatorService:
    
    def __init__(self, uh_initial, uv_initial, lh_initial, lv_initial, max_vertical_displacement, execution_interval, rabbitmq_config):
        # Initialize the PTEmulatorService with initial values and configuration
        self._l = logging.getLogger("PTEmulatorService")
        self._l.info("Initializing PTEmulatorService.")

        self._rabbitmq = Rabbitmq(**rabbitmq_config)
        
        self.uh = uh_initial
        self.uv = uv_initial
        self.lh = lh_initial
        self.lv = lv_initial

        self.vertical_period = 2.0
        self.horizontal_period = 2.0

        self.lh_wanted = 100
        self.uv_wanted = 20

        self.max_vertical_displacement = max_vertical_displacement
        self._execution_interval = execution_interval # seconds
        self._force_on = 0.0
        self.E_modulus = 70e3 # MPa (example value for aluminum)
        self.Damage = 0.0

        # Initialize the PT model instance
        try:
            self.PT_Model = pt_model.PtModel()
        except Exception as e:
            self._l.error("Failed to initialize PTModel: %s", e, exc_info=True)
            raise

        # Initialize the actuator controller instance
        try:
            self.H_ac = ac_ctrl.ActuatorController(self.lh_wanted, self.horizontal_period, self._execution_interval)
            self.V_ac = ac_ctrl.ActuatorController(self.uv_wanted, self.vertical_period, self._execution_interval)
        except Exception as e:
            self._l.error("Failed to initialize ActuatorController: %s", e, exc_info=True)
            raise

        # Initialize the RFCA (RainFlow Cycle Algorithm) instance (Only in PT)
        try:
            self.RFCA = rfca.RFCA([])
        except Exception as e:
            self._l.error("Failed to initialize RFCA: %s", e, exc_info=True)
            raise

        self.PT_Model.set_beampars(16, 'E', self.E_modulus) # Set the beam parameters for the PT model  

    def setup(self):
        self._rabbitmq.connect_to_server()

        # Declare local queues for the force messages
        self.forces_queue_name = self._rabbitmq.declare_local_queue(routing_key=ROUTING_KEY_FORCES)

        self._l.info(f"PTEmulatorService setup complete.")

    def _read_forces(self):
        # Read the forces from the RabbitMQ queue
        
        msg = self._rabbitmq.get_message(self.forces_queue_name)
        #self._l.debug(f"Message received: {msg}")
        if msg is not None:
            return msg
        else:
            return None
    
    def check_control_commands(self):
        # Check if there are control commands
        force_cmd = self._read_forces()
        # self._l.debug(f"Control command: {force_cmd}")
        if force_cmd is not None:
            if 'forces' in force_cmd and force_cmd['forces'] is not None:
                self._l.info("Force command: %s", force_cmd["forces"])
                self._force_on = 1.0 if force_cmd["forces"] else 0.0

            if "horizontal_force" in force_cmd and force_cmd["horizontal_force"] is not None:
                self._l.info(f"Horizontal force command: {force_cmd['horizontal_force']}")
                self.lh_wanted = force_cmd["horizontal_force"]
                self.H_ac.set_amplitude(self.lh_wanted)

            if "vertical_displacement" in force_cmd and force_cmd["vertical_displacement"] is not None:
                self._l.info(f"Vertical force command: {force_cmd['vertical_displacement']}")
                self.uv_wanted = force_cmd["vertical_displacement"]
                self.V_ac.set_amplitude(self.uv_wanted)
                
            if "horizontal_period" in force_cmd and force_cmd["horizontal_period"] is not None:
                self._l.info(f"Horizontal period command: {force_cmd['horizontal_period']}")
                self.horizontal_period = force_cmd["horizontal_period"]
                self.H_ac.set_period(self.horizontal_period)
                
            if "vertical_period" in force_cmd and force_cmd["vertical_period"] is not None:
                self._l.info(f"Vertical period command: {force_cmd['vertical_period']}")
                self.vertical_period = force_cmd["vertical_period"]
                self.V_ac.set_period(self.vertical_period)

    def emulate_pt(self):
        # Emulate the PT behavior based on the control commands

        # Additional logic for the emulator can go here
        # Additional logic for the PT emulator can go here
        if self._force_on == 1.0:
            try:
                Load = self.H_ac.step_simulation()
                Displacement = self.V_ac.step_simulation()
                self._l.info(f"Load: {Load}, Displacement: {Displacement}")
                self.PT_Model.set_loads_between_nodes(Load, [9,10])
                self.PT_Model.set_displacements_between_nodes(Displacement,[5,10])
            except Exception as e:
                self._l.error("Failed to emulate PT behavior: %s", e, exc_info=True)
                raise

            try:
                self.PT_Model.run_simulation()
            except Exception as e:
                self._l.error("Simulation failed: %s", e, exc_info=True)
                raise
            
            self._uh, self._uv, self._lh, self._lv = self.get_data(10) # Get the data from the PT model (10 is the node number)
            self._l.info(f"Horizontal displacement: {self._uh}, Vertical displacement: {self._uv}, Horizontal force: {self._lh}, Vertical force: {self._lv}")
            
            # Fatigue - PT only
            if self.RFCA.update_if_peak(self._lv):
                self._l.info(f"Running Fatigue test")
                [self.Damage, self.E_modulus] = self.PT_Model.calculate_fatigue(self.RFCA.get_cycles())
                self._l.info(f"Fatigue test result: {round(self.E_modulus)} MPa, Damage: {round(self.Damage,3)}")

        else:
            # Horizontal displacement
            self._uh = 0.0
            # Vertical displacement
            self._uv = 0.0
            # Horizontal force
            self._lh = 0.0
            # Vertical force
            self._lv = 0.0
            # Restoring force
            # self._r = r[something] # in case we need this for the emulator, we can put it here

        self.E_modulus = self.PT_Model.get_beampars(16).E # Get the E modulus from the PT model
        #self._l.info("PT script executed successfully.")
        
    def send_state(self, time_start):
        #self._l.info("Sending state to hybrid test bench physical twin.")
        timestamp = time.time_ns()
        # Publishes the new state
        message = {
            "measurement": "emulator",
            "time": timestamp,
            "tags": {
                "source": "emulator"
            },
            "fields": {
                "horizontal_displacement": self._uh,
                "vertical_displacement": self._uv,
                "horizontal_force": self._lh,
                "vertical_force": self._lv,
                "horizontal_displacement_between": self.PT_Model.get_displacement_between_nodes(9, 10),
                "vertical_displacement_between": self.PT_Model.get_displacement_between_nodes(5, 10),
                "E_modulus": self.E_modulus,
                "force_on": self._force_on,
                "max_vertical_displacement": self.max_vertical_displacement,
                "execution_interval": self._execution_interval,
                "elapsed": time.time() - time_start,
            }
        }
        
        self._rabbitmq.send_message(ROUTING_KEY_STATE, message)

    def update_state(self, time_start):
        #self._l.info("Sending state to hybrid test bench physical twin.")
        timestamp = time.time_ns()
        # Publishes the new state
        state_message = {
            # "pt_displacements": self.PT_Model.get_displacement([10, 10, 10], [1, 2, 3])
            "horizontal_displacement": round(self.PT_Model.get_displacement_between_nodes(9, 10), 3),
            "vertical_displacement": round(self.PT_Model.get_displacement_between_nodes(5, 10), 3),
            "horizontal_force": round(self.PT_Model.get_load(10, fx), 3),
            "vertical_force": round(self.PT_Model.get_load(10, fz), 3)
            }

        self._rabbitmq.send_message(ROUTING_KEY_DISPLACEMENT, state_message)
        #self._l.debug(f"Message sent to {ROUTING_KEY_STATE}.")
        #self._l.debug(message)
    
    def start_emulation(self):
        # Start the emulation loop
        self._l.info("Starting PTEmulator emulation loop.")
        send_state_interval = 1
        try:
            while True:
                #self._l.debug("Emulation loop iteration.")
                time_start = time.time()
                #Check if there are control commands
                self.check_control_commands()
                # Emulate the PT behavior
                self.emulate_pt() 
                # Send the new state to the hybrid test bench physical twin
                self.send_state(time_start)
                if send_state_interval == 3:
                    self.update_state(time_start)
                    send_state_interval = 1
                send_state_interval += 1                 # Sleep until the next sample
                time_end = time.time()
                time_diff = time_end - time_start
                if time_diff < self._execution_interval:
                    time.sleep(self._execution_interval - time_diff)
                else:
                    self._l.warning(f"Emulation loop took too long: {time_diff} seconds.")
        except KeyboardInterrupt:
            self._l.info("Emulation loop interrupted by user.")
        except Exception as e:
            self._l.error("Emulation loop failed: %s", e, exc_info=True)

    def get_data(self, node):
        # Get the data from the PT model
        try:
            uh = float(self.PT_Model.get_displacement(node, fx))
            uv = float(self.PT_Model.get_displacement(node, fz))
            lh = float(self.PT_Model.get_load(node, fx))
            lv = float(self.PT_Model.get_load(node, fz))
            return uh, uv, lh, lv
        except Exception as e:
            self._l.error("Failed to get data from PT model: %s", e, exc_info=True)
            raise
        return self._uh, self._uv, self._lh, self._lv
    
if __name__ == "__main__":
    # Get utility functions to config logging and load configuration
    from pyhocon import ConfigFactory
    
    logging_conf = os.path.join(os.path.dirname(os.getcwd()), 'hybrid-test-bench', 'logging.conf')
    logging.config.fileConfig(logging_conf)

    # Get path to the startup.conf file used in the hybrid test bench PT & DT:
    startup_conf = os.path.join(os.path.dirname(os.getcwd()), 'hybrid-test-bench', 'software','startup.conf')
    assert os.path.exists(startup_conf), 'startup.conf file not found'

    # The startup.conf comes from the hybrid test bench repository.
    config = ConfigFactory.parse_file(startup_conf)
    
    service = PTEmulatorService(
        uh_initial = 0.0,
        uv_initial = 0.0,
        lh_initial = 0.0,
        lv_initial = 0.0,
        max_vertical_displacement = 70.0,
        execution_interval = 3.0,
        rabbitmq_config=config["rabbitmq"])

    service.setup()
    
    # Start the PTEmulatorService
    service.start_emulation()
