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
from communication.shared.protocol import ROUTING_KEY_STATE, ROUTING_KEY_FORCES
import pt_model as pt_model

class ActuatorController:
    def __init__(self, rabbitmq_config):
        self._l = logging.getLogger("ActuatorController")
        self._l.info("Initializing ActuatorController.")

        self._rabbitmq = Rabbitmq(**rabbitmq_config)

    def do_something(self, ch, method, properties, body_json):
        # Log the values received.
        self._l.info(f"Received state sample: {body_json}")

        # Here you can implement the logic to control the actuator based on the received message.
        # For example, you can parse the message and send commands to the actuator.

    def setup(self):
        self._rabbitmq.connect_to_server()

        # Subscribe to any message coming from the Hybrid Test Bench physical twin.
        self._rabbitmq.subscribe(routing_key=ROUTING_KEY_STATE,
                                on_message_callback=self.do_something)

        self._l.info(f"ActuatorController setup complete.")
    
    def start_controller(self):
        self._rabbitmq.start_consuming()

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

    controller = ActuatorController(
        rabbitmq_config=config["rabbitmq"])

    controller.setup()

    # Start the actuator controller
    controller.start_controller()