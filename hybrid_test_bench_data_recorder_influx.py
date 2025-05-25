
# Configure python path to load incubator modules
import sys
import os
import logging
import logging.config
import time

from influxdb_client import InfluxDBClient
from influxdb_client.client.write_api import SYNCHRONOUS

# Get the current working directory. Should be hybrid-test-bench
current_dir = os.getcwd()

assert os.path.basename(current_dir) == 'hybrid-test-bench', 'Current directory is not hybrid-test-bench'

# Get the parent directory. Should be the root of the repository
parent_dir = current_dir

# The root of the repo should contain the startup folder. Otherwise something went wrong during the inital setup.
assert os.path.exists(os.path.join(parent_dir, 'startup')), 'startup folder not found in the repository root'

# The root of the repo should contain the installation folder. Otherwise something went wrong during the inital setup.
assert os.path.exists(os.path.join(parent_dir, 'installation')), 'installation folder not found in the repository root'

bench_startup_dir = os.path.join(parent_dir, 'startup')

assert os.path.exists(bench_startup_dir), 'hybrid-test-bench startup directory not found'

# Add the parent directory to sys.path
sys.path.append(bench_startup_dir)

from communication.shared.protocol import ROUTING_KEY_RECORDER
from communication.server.rabbitmq import Rabbitmq
import numpy as np

class HybridTestBenchDataRecorderInflux:
    def __init__(self, rabbitmq_config, influxdb_config):
        self._l = logging.getLogger("HybridTestBenchDataRecorderInflux")
        self._l.info("Initializing HybridTestBenchDataRecorderInflux.")
        self._l.info("Connecting to InfluxDB...")    
        client = InfluxDBClient(**influxdb_config)
        write_api = client.write_api(write_options=SYNCHRONOUS)
        self.write_api = write_api
        self.influx_db_org = influxdb_config["org"]
        self.influxdb_bucket = influxdb_config["bucket"]

        self.rabbitmq = Rabbitmq(**rabbitmq_config)

    def read_record_request(self, ch, method, properties, body_json):
        self._l.debug("New record msg:")
        self._l.debug(body_json)
        try:
            self.write_api.write(self.influxdb_bucket, self.influx_db_org, body_json)
        except Exception as e:
            self._l.error("Failed to write to InfluxDB: %s", e, exc_info=True)
            raise

    def setup(self):
        self._l.info("Setting up HybridTestBenchDataRecorderInflux.")
        self.rabbitmq.connect_to_server()

        self.rabbitmq.subscribe(routing_key=ROUTING_KEY_RECORDER,
                           on_message_callback=self.read_record_request)

    def start_recording(self):
        self._l.info("Starting HybridTestBenchDataRecorderInflux.")
        try:
            self.rabbitmq.start_consuming()
        except KeyboardInterrupt:
            self.rabbitmq.close()
    
    def get_data(self, node):
        # Placeholder for data retrieval logic
        wanted_node = node
        
        # Horizontal displacement
        try:
            self._uh = float(PT_Model.get_displacement(wanted_node, fx))
        except IndexError as e:
            self._l.error(f"Error retrieving horizontal displacement from u({wanted_node},1): %s", e, exc_info=True)

        # Vertical displacement
        try:
            self._uv = float(PT_Model.get_displacement(wanted_node, fz))
        except IndexError as e:
            self._l.error(f"Error retrieving vertical displacement from u({wanted_node},1): %s", e, exc_info=True)
        
        #Forces
        try:
            # Vertical force
            self._lh = float(PT_Model.get_load(wanted_node, fx))
            # Horizontal force
            self._lv = float(PT_Model.get_load(wanted_node, fz))
        except Exception as e:
            self._l.error(f"Error retrieving forces from PT_Model.get_loads(): %s", e, exc_info=True)
            self._l.error(f"Forces not set: lh = {self._lh}, lv = {self._lv}")

        return self._uh, self._uv, self._lh, self._lv

    
if __name__ == "__main__":
    # Get utility functions to config logging and load configuration
    from pyhocon import ConfigFactory

    # Get logging configuration
    logging.config.fileConfig("logging.conf")

    # Get path to the startup.conf file used in the hybrid test bench PT & DT:
    startup_conf = os.path.join(os.path.dirname(os.getcwd()), 'hybrid-test-bench', 'software','startup.conf')
    assert os.path.exists(startup_conf), 'startup.conf file not found'

    # The startup.conf comes from the hybrid test bench repository.
    config = ConfigFactory.parse_file(startup_conf)
    
    service = HybridTestBenchDataRecorderInflux(rabbitmq_config=config["rabbitmq"], influxdb_config=config["influxdb"])

    service.setup()
    
    # Start the HybridTestBenchDataRecorderInflux
    service.start_recording()
