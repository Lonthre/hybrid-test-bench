
# Configure python path to load hybrid test bench modules
import sys
import os
import logging
import logging.config
import time
import numpy as np

# Get the current working directory. Should be hybrid-test-bench
current_dir = os.getcwd()

assert os.path.basename(current_dir) == 'hybrid-test-bench', 'Current directory is not hybrid-test-bench'

# Get the parent directory. Should be the root of the repository
parent_dir = current_dir

from communication.server.rabbitmq import Rabbitmq
from communication.shared.protocol import ROUTING_KEY_STATE, ROUTING_KEY_FORCES
import pt_model as pt_model


class PTEmulatorService:
    
    def __init__(self, uh_initial, uv_initial, lh_initial, lv_initial, execution_interval, rabbitmq_config):

        self._rabbitmq = Rabbitmq(**rabbitmq_config)
        self._l = logging.getLogger("PTEmulatorService")

        self.uh = uh_initial
        self.uv = uv_initial
        self.lh = lh_initial
        self.lv = lv_initial
        # self.r = r_initial # do we need this for the emulator?
        self._execution_interval = execution_interval # seconds
        self._force_on = 0.0  

    def setup(self):
        self._rabbitmq.connect_to_server()

        # Declare local queues for the force messages
        self.forces_queue_name = self._rabbitmq.declare_local_queue(routing_key=ROUTING_KEY_FORCES)

        self._l.info(f"PTEmulatorService setup complete.")

    def _read_forces(self):
        msg = self._rabbitmq.get_message(self.forces_queue_name)
        if msg is not None:
            return msg["forces"]
        else:
            return None
    
    def check_control_commands(self):
        # Check if there are control commands
        force_cmd = self._read_forces()
        if force_cmd is not None:
            self._l.debug(f"Force command: on={force_cmd}")
            self._force_on = 1.0 if force_cmd else 0.0

    def emulate_pt(self):

        # Call the main function or logic from the script
        model = pt_model.PtModel()
        u, lf, r = model.run_simulation()


        # Additional logic for the emulator can go here
        # _uh, _uv, _lh, _lv, and _r need to be extracted from the simulation results (u, lf, r)
        if self._force_on == 1.0:
            # Horizontal displacement
            self._uh = u[model.get_dof1_horizontal(), :] # not in use currently
            # Vertical displacement
            self._uv = u[model.get_dof1_vertical(), :]
            # Horizontal force
            self._lh = lf[0, :] # not in use currently
            # Vertical force
            self._lv = lf[0, :]
            # Restoring force
            #self._r = r[?????] # in case we need this for the emulator, we can put it here
        else: 
            # If the force is off, set displacements and forces to zero
            self._uh = 0.0
            self._uv = 0.0
            self._lh = 0.0
            self._lv = 0.0
            #self._r = 0.0 # in case we need this for the emulator, we can put it here

        self._l.info("PT script executed successfully.")
        
    def send_state(self, time_start):
        timestamp = time.time_ns()
        # Publishes the new state
        message = {
            "measurement": "emulator",
            "time": timestamp,
            "tags": {
                "source": "emulator"
            },
            "fields": {
                "horizontal_displacement": float(np.mean(self._uh)) if isinstance(self._uh, (np.ndarray, list)) else float(self._uh),
                "vertical_displacement": float(np.mean(self._uv)) if isinstance(self._uv, (np.ndarray, list)) else float(self._uv),
                "horizontal_force": float(np.mean(self._lh)) if isinstance(self._lh, (np.ndarray, list)) else float(self._lh),
                "vertical_force": float(np.mean(self._lv)) if isinstance(self._lv, (np.ndarray, list)) else float(self._lv),
                # "restoring_force": self._r.tolist() if isinstance(self._r, np.ndarray) else self._r,
                "force_on": self._force_on,
                "execution_interval": self._execution_interval,
                "elapsed": time.time() - time_start,
            }
        }

        self._rabbitmq.send_message(ROUTING_KEY_STATE, message)
        self._l.debug(f"Message sent to {ROUTING_KEY_STATE}.")
        self._l.debug(message)
    
    def start_emulation(self):
        # Start the emulation loop
        self._l.info("Starting PTEmulator emulation loop.")
        while True:
            time_start = time.time()
            #Check if there are control commands
            self.check_control_commands()
            # Emulate the PT behavior
            self.emulate_pt() 
            # Send the new state to the hybrid test bench physical twin
            self.send_state(time_start)
            # Sleep until the next sample
            time_end = time.time()
            time_diff = time_end - time_start
            if time_diff < self._execution_interval:
                time.sleep(self._execution_interval - time_diff)
            else:
                self._l.warning(f"Emulation loop took too long: {time_diff} seconds.")
    
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
        # r_initial = 0.0,
        execution_interval = 3.0,
        rabbitmq_config=config["rabbitmq"])

    service.setup()
    
    # Start the PTEmulatorService
    service.start_emulation()
