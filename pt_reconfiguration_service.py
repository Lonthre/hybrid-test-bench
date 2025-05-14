
# Configure python path to load the hybrid test bench modules
import sys
import os
import logging
import logging.config
import time
from influxdb_client import InfluxDBClient
from influxdb_client.client.write_api import SYNCHRONOUS
import rtamt

# Get the current working directory. Should be hybrid-test-bench.
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

from communication.server.rabbitmq import Rabbitmq
from communication.shared.protocol import ROUTING_KEY_STATE, ROUTING_KEY_FORCES

class PT_ReconfigurationService:

    def __init__(self, rabbitmq_config, influxdb_config):
        self._rabbitmq = Rabbitmq(**rabbitmq_config)
        
        url = influxdb_config['url']
        token = influxdb_config['token']
        self._org = influxdb_config['org']
        self._bucket = influxdb_config['bucket']

        self._client = InfluxDBClient(url=url, token=token, org=self._org)

        self._l = logging.getLogger("PT_ReconfigurationService")

    def turn_off_forces(self):
        # Send a message to the PT to turn off the forces.
        with Rabbitmq(**config["rabbitmq"]) as rabbitmq:
            rabbitmq.send_message(ROUTING_KEY_FORCES, {"forces": False})

        # Wait a bit for the message to be processed
        time.sleep(5)

    def check_robustness(self, robustness):
        # Check if the robustness is below a certain threshold.
        # If it is, send a message to the PT to reconfigure the system.
        threshold = 0.0
        for r in robustness:
            # Check if the robustness is below the threshold.
            if r[1] < threshold:
                # Here we will send a message that will turn off the forces
                self.turn_off_forces()

    def query_influxdb(self):
            # Define your Flux query: Query the relevant forces and displacements.
            # We set a stop time of -3s to ensure that the data is aligned from the different measurements.

            flux_query = f'''
                from(bucket: "{self._bucket}")
                |> range(start: -1h, stop: -3s)
                |> filter(fn: (r) => r["_measurement"] == "robustness")
                |> filter(fn: (r) => r["_field"] == "robustness")
                |> filter(fn: (r) => r["source"] == "pt_stl_monitor")
                |> aggregateWindow(every: 3s, fn: last, createEmpty: true)
                |> yield(name: "last")
            '''
            # Execute the query
            result = self._query_api.query(org=self._org, query=flux_query)

            robustness = []

            for table in result:
                for record in table.records:
                    ts = record.get_time().timestamp()
                    if record.get_field() == 'robustness':
                        robustness.append([ts, record.get_value()])

            return robustness
    
    def get_robustness(self, ch, method, properties, body_json):
    # Log the values received.
        self._l.info(f"Received state sample: {body_json}")
        
        # Get the displacement history from the influxdb, and process the displacement data into signals that ramt can understand.
        robustness = self.query_influxdb()

        self._l.debug(f"Robustness: {robustness}")

        self.check_robustness(robustness)

    def setup(self):
            self._rabbitmq.connect_to_server()

            # Initialize the Query API
            self._query_api = self._client.query_api()
            self._write_api = self._client.write_api(write_options=SYNCHRONOUS)

            # Subscribe to any message coming from the Hybrid Test Bench physical twin.
            self._rabbitmq.subscribe(routing_key=ROUTING_KEY_STATE,
                                    on_message_callback=self.get_robustness)

            self._l.info(f"PT_ReconfigurationService setup complete.")

    def start_serving(self):
            self._rabbitmq.start_consuming()
        
if __name__ == "__main__":
    # Get utility functions to config logging and load configuration
    from pyhocon import ConfigFactory

    # Get logging configuration
    log_conf = os.path.join(os.path.dirname(os.getcwd()), 'hybrid-test-bench', 'log.conf')
    logging.config.fileConfig(log_conf)

    # Get path to the startup.conf file used in the hybrid test bench PT & DT:
    startup_conf = os.path.join(os.path.dirname(os.getcwd()), 'hybrid-test-bench', 'software','startup.conf')
    assert os.path.exists(startup_conf), 'startup.conf file not found'

    # The startup.conf comes from the hybrid test bench repository.
    config = ConfigFactory.parse_file(startup_conf)

    service = PT_ReconfigurationService(rabbitmq_config=config["rabbitmq"], influxdb_config=config["influxdb"])

    service.setup()
    
    # Start the PT_ReconfigurationService
    service.start_serving()
