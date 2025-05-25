
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
from communication.shared.protocol import ROUTING_KEY_STATE, ROUTING_KEY_DT_FORCES, ROUTING_KEY_DISPLACEMENT
import dt_model as dt_model
import pt_model as pt_model
import calibration_service as cal_service
import actuator_controller as ac_ctrl

# Define the global variables for the model
fx, fy, fz, mx, my, mz = 1, 2, 3, 4, 5, 6 # force and moment indices

class DTService:
    
    def __init__(self, uh_initial, uv_initial, lh_initial, lv_initial, max_vertical_displacement, execution_interval, rabbitmq_config):
        # Initialize the DTService with initial values and configuration
        self._l = logging.getLogger("DTService")
        self._l.info("Initializing DTService.")

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
        self.E_modulus = 100e3 # MPa (wrong example value for aluminum)
        # self.Damage = 0.0

        self.PT_Model_h_d = 0.0
        self.PT_Model_v_d = 0.0
        self.PT_Model_h_f = 0.0
        self.PT_Model_v_f = 0.0

        # Initialize the DT model instance
        try:
            self.DT_Model = dt_model.DtModel()
        except Exception as e:
            self._l.error("Failed to initialize DTModel: %s", e, exc_info=True)
            raise

        # Initialize the actuator controller instance
        try:
            self.H_ac = ac_ctrl.ActuatorController(self.lh_wanted, self.horizontal_period, self._execution_interval)
            self.V_ac = ac_ctrl.ActuatorController(self.uv_wanted, self.vertical_period, self._execution_interval)
        except Exception as e:
            self._l.error("Failed to initialize ActuatorController: %s", e, exc_info=True)
            raise

        # Initialize the CalibrationService instance (Only in DT)
        try:
            self.calibration_service = cal_service.CalibrationService(self.DT_Model) # DT Model
        except Exception as e:
            self._l.error("Failed to initialize CalibrationService: %s", e, exc_info=True)
            raise

        self.DT_Model.set_beampars(16, 'E', self.E_modulus) # Set the beam parameters for the DT model 

    def setup(self):
        self._rabbitmq.connect_to_server()

        # Declare local queues for the force messages
        self.forces_queue_name = self._rabbitmq.declare_local_queue(routing_key=ROUTING_KEY_DT_FORCES)
        self.displacements_queue_name = self._rabbitmq.declare_local_queue(routing_key=ROUTING_KEY_DISPLACEMENT)

        self._l.info(f"DTService setup complete.")

    def _read_forces(self):
        # Read the forces from the RabbitMQ queue
        msg = self._rabbitmq.get_message(self.forces_queue_name)

        if msg is not None:
            return msg
        else:
            return None

    def _read_state(self):
        # Read the PT model from the RabbitMQ queue
        msg = self._rabbitmq.get_message(self.displacements_queue_name)

        if msg is not None:
            return msg
        else:
            return None
    
    def _check_pt_model(self):
        # Check if there are control commands
        state = self._read_state()
        #self._l.debug(f"Control command: {pt_model}")
        if state is not None:
            self.state_received = True
            if 'horizontal_displacement' in state and state['horizontal_displacement'] is not None:
                # self._l.info("Horizontal displacement: %s", state["horizontal_displacement"])
                self.PT_Model_h_d = state["horizontal_displacement"][2]
            if 'vertical_displacement' in state and state['vertical_displacement'] is not None:
                # self._l.info("Vertical displacement: %s", state["vertical_displacement"])
                self.PT_Model_v_d = state["vertical_displacement"][2]
            if 'horizontal_force' in state and state['horizontal_force'] is not None:
                # self._l.info("Horizontal force: %s", state["horizontal_force"])
                self.PT_Model_h_f = state["horizontal_force"][0]
            if 'vertical_force' in state and state['vertical_force'] is not None:
                # self._l.info("Vertical force: %s", state["vertical_force"])
                self.PT_Model_v_f = state["vertical_force"][0]
        else:
            self.state_received = False
            # self._l.debug("No control command received.")
    
    def check_control_commands(self):
        # Check if there are control commands
        force_cmd = self._read_forces()
        #self._l.debug(f"Control command: {force_cmd}")
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

    def emulate_dt(self):
        # Emulate the DT behavior based on the control commands

        # Additional logic for the DT can go here
        if self._force_on == 1.0:
            self._l.info(f"State received: {self.state_received}.")
            try:
                if self.state_received:
                    old_load = self.H_ac.step_simulation()
                    old_displacement = self.V_ac.step_simulation()
                    rload = self.PT_Model_h_f
                    rdisplacement = self.PT_Model_v_d
                    self.H_ac.calibrate(rload)
                    self.V_ac.calibrate(rdisplacement)
                    
                    if abs((old_load - rload)/old_load) > 0.1:
                        self._l.warning(f"Load difference: {abs((old_load - rload)/old_load)*100}%")
                    
                    if abs((old_displacement - rdisplacement)/old_displacement) > 0.1:
                        self._l.warning(f"Displacement difference: {abs((old_displacement - rdisplacement)/old_displacement)*100}%")

            except Exception as e:
                self._l.error("Failed to emulate PT behavior: %s", e, exc_info=True)
                raise

            # Calibration service - DT only
            if self.state_received:
                rload = self.PT_Model_h_f
                rdisplacement = self.PT_Model_v_d
                try:
                    self.DT_Model.set_loads_between_nodes(rload, [9,10])
                    self.DT_Model.set_displacements_between_nodes(rdisplacement,[5,10])
                    
                    state = np.array([self.PT_Model_h_d, self.PT_Model_v_d,self.PT_Model_h_f, self.PT_Model_v_f]) # Get the displacements from the PT model
                    self._l.info(f"State: {state}")
                    #self.calibration_service.set_DT_Model(self.DT_Model) # Set the DT model in the calibration service
                    self.calibration_service.set_calibration_state(state) # Set the displacements in the calibration service
                    self.DT_Model = self.calibration_service.calibrate_model(self.DT_Model) # Call the calibration service to calibrate the model
                    #self.DT_Model = self.calibration_service.get_DT_Model() # Get the calibrated model
                except Exception as e:
                    self._l.error("Calibration service failed: %s", e, exc_info=True)
                    raise

            try:
                load = self.H_ac.step_simulation()
                displacement = self.V_ac.step_simulation()

                self.DT_Model.set_loads_between_nodes(load, [9,10])
                self.DT_Model.set_displacements_between_nodes(displacement,[5,10])

                self.DT_Model.run_simulation()
            except Exception as e:
                self._l.error("Simulation failed: %s", e, exc_info=True)
                raise
            
            self._uh, self._uv, self._lh, self._lv = self.get_data(10) # Get the data from the PT model (10 is the node number)
        
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

        self.E_modulus = self.DT_Model.get_beampars(16).E # Get the E modulus from the DT model
        # self.DT_Model.set_beampars(16, 'E', self.E_modulus) # Set the E modulus in the DT model
        #self._l.info("PT script executed successfully.")
        
    def send_state(self, time_start):
        #self._l.info("Sending state to hybrid test bench physical twin.")
        timestamp = time.time_ns()
        # Publishes the new state
        message = {
            "measurement": "dt",
            "time": timestamp,
            "tags": {
                "source": "dt"
            },
            "fields": {
                "horizontal_displacement": self._uh,
                "vertical_displacement": self._uv,
                "horizontal_force": self._lh,
                "vertical_force": self._lv,
                "horizontal_displacement_between": self.DT_Model.get_displacement_between_nodes(9, 10)[2],
                "vertical_displacement_between": self.DT_Model.get_displacement_between_nodes(5, 10)[2],
                "E_modulus": self.E_modulus,
                "force_on": self._force_on,
                "max_vertical_displacement": self.max_vertical_displacement,
                "execution_interval": self._execution_interval,
                "elapsed": time.time() - time_start,
            }
        }

        self._rabbitmq.send_message(ROUTING_KEY_STATE, message)
        #self._l.debug(f"Message sent to {ROUTING_KEY_STATE}.")
        #self._l.debug(message)
    
    def start_emulation(self):
        # Start the emulation loop
        self._l.info("Starting PTEmulator emulation loop.")
        try:
            while True:
                #self._l.debug("Emulation loop iteration.")
                time_start = time.time()
                #Check if there are control commands
                self.check_control_commands()
                # Check if there are PT model displacements
                self._check_pt_model()
                # Emulate the DT behavior
                self.emulate_dt() 
                # Send the new state to the hybrid test bench digital twin
                self.send_state(time_start)
                # Sleep until the next sample
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
            uh = float(self.DT_Model.get_displacement(node, fx)[0])
            uv = float(self.DT_Model.get_displacement(node, fz)[0])
            lh = float(self.DT_Model.get_load(node, fx)[0])
            lv = float(self.DT_Model.get_load(node, fz)[0])
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
    
    service = DTService(
        uh_initial = 0.0,
        uv_initial = 0.0,
        lh_initial = 0.0,
        lv_initial = 0.0,
        max_vertical_displacement = 70.0,
        execution_interval = 3.0,
        rabbitmq_config=config["rabbitmq"])

    service.setup()
    
    # Start the DTService
    service.start_emulation()
