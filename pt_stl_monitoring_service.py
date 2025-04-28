
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
import pt_model as pt_model

class PT_STLMonitoringService:

    def __init__(self, rabbitmq_config, influxdb_config):

        self._rabbitmq = Rabbitmq(**rabbitmq_config)
        
        url = influxdb_config['url']
        token = influxdb_config['token']
        self._org = influxdb_config['org']
        self._bucket = influxdb_config['bucket']

        self._client = InfluxDBClient(url=url, token=token, org=self._org)

        self._l = logging.getLogger("PT_STLMonitoringService")

        # Specification
        self._spec = rtamt.StlDenseTimeSpecification()
        # Declare the variables that will correspond to the above signals.
        self._spec.declare_var('vertical_displacement', 'float')
        self._spec.declare_const('max_vertical_displacement', 'float', 5.0)
        self._spec.spec = 'always((vertical_displacement >= max_vertical_displacement) implies (eventually[0:60](vertical_displacement <= max_vertical_displacement)))'
        self._spec.parse()

    def setup(self):
        self._rabbitmq.connect_to_server()

        # Initialize the Query API
        self._query_api = self._client.query_api()
        self._write_api = self._client.write_api(write_options=SYNCHRONOUS)

        # Subscribe to any message coming from the Hybrid Test Bench physical twin.
        self._rabbitmq.subscribe(routing_key=ROUTING_KEY_STATE,
                                on_message_callback=self.process_state_sample)

        self._l.info(f"PT_STLMonitoringService setup complete.")

    def query_influxdb(self):
        # Define your Flux query: Query the relevant forces and displacements.
        # We set a stop time of -3s to ensure that the data is aligned from the different measurements.

        flux_query = f'''
            from(bucket: "{self._bucket}")
            |> range(start: -1h, stop: -3s)
            |> filter(fn: (r) => r["_measurement"] == "emulator")
            |> filter(fn: (r) => r["_field"] == "vertical_displacement")
            |> filter(fn: (r) => r["source"] == "emulator")
            |> aggregateWindow(every: 3s, fn: last, createEmpty: true)
            |> yield(name: "last")
        '''
        # Execute the query
        result = self._query_api.query(org=self._org, query=flux_query)

        vertical_displacement = []

        for table in result:
            for record in table.records:
                ts = record.get_time().timestamp()
                vertical_displacement.append([ts, record.get_value()])

        # Generate a time-series signal for max_vertical_displacement
        max_vertical_displacement = [[ts, 5.0] for ts, _ in vertical_displacement]

        assert len(vertical_displacement) == len(max_vertical_displacement), 'Vertical displacement and maximum vertical displacement data not aligned.'

        return vertical_displacement, max_vertical_displacement

    def compute_robustness(self, vertical_displacement, max_vertical_displacement):
        # Evaluate rtamt on the signals and get the robustness.
        print("Evaluating rtamt on the signals.")
        robustness = self._spec.evaluate(
            ['vertical_displacement', vertical_displacement],
            ['max_vertical_displacement', max_vertical_displacement]
        )
        return robustness
    
    def store_robustness(self, robustness):
        # Store the robustness in the InfluxDB. Duplicate records on the same timestamp will just be updated.
        records = []
        for r in robustness:
            ts = int(r[0] * 1e9)

            records.append({
                "measurement": "robustness",
                "tags": {
                    "source": "pt_stl_monitor"
                },
                "time": ts,
                "fields": {
                    "robustness": r[1]
                }
                })

        self._write_api.write(bucket=self._bucket, record=records)

    def process_state_sample(self, ch, method, properties, body_json):
        # Log the values received.
        self._l.info(f"Received state sample: {body_json}")
        
        # Get the displacement history from the influxdb, and process the displacement data into signals that ramt can understand.
        vertical_displacement, max_vertical_displacement = self.query_influxdb()

        # Evaluate ramt on the signals and get the robustness.
        robustness = self.compute_robustness(vertical_displacement, max_vertical_displacement)

        self._l.debug(f"Robustness: {robustness}")

        # Store the robustness in the InfluxDB.
        self.store_robustness(robustness)

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

    service = PT_STLMonitoringService(rabbitmq_config=config["rabbitmq"], influxdb_config=config["influxdb"])

    service.setup()
    
    # Start the PT_STLMonitoringService
    service.start_serving()
