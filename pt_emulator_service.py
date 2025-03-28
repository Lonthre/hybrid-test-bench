
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
from communication.shared.protocol import ROUTING_KEY_STATE,ROUTING_KEY_FORCES
import pt_model as pt_model


class PTEmulatorService:
    
    def __init__(self, execution_interval, rabbitmq_config):

        self._rabbitmq = Rabbitmq(**rabbitmq_config)
        self._l = logging.getLogger("PTEmulatorService")

        self._execution_interval = execution_interval

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

    def emulate_pt(self):

        # Call the main function or logic from the script
        model = pt_model.PtModel()
        u, l, r = model.run_simulation()

        # Additional logic for the emulator can go here
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
                "id": 29 # Just hardcoded for now, but should be the id of the PT in the system.
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
            # Emulate the PT behavior
            self.emulate_pt() 
            # Sleep until the next sample
            time_end = time.time()
            time_diff = time_end - time_start
            if time_diff < self._execution_interval:
                time.sleep(self._execution_interval - time_diff)
            else:
                self._l.warning(f"Emulation loop took too long: {time_diff} seconds.")
    
if __name__ == "__main__":
    # Get utility functions to config logging and load configuration
    # from software.config import load_config
    from pyhocon import ConfigFactory
    
    # Get path to the startup.conf file used in the hybrid test bench PT & DT:
    startup_conf = os.path.join(os.path.dirname(os.getcwd()), 'hybrid-test-bench', 'software','startup.conf')
    assert os.path.exists(startup_conf), 'startup.conf file not found'

    # The startup.conf comes from the hybrid test bench repository.
    config = ConfigFactory.parse_file(startup_conf)
    
    service = PTEmulatorService(
        execution_interval = 3.0,
        rabbitmq_config=config["rabbitmq"])

    service.setup()
    
    # Start the PTEmulatorService
    service.start_emulation()
