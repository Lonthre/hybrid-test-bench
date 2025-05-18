
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

# Define the global variables for the model
fx, fy, fz, mx, my, mz = 1, 2, 3, 4, 5, 6 # force and moment indices

# Define the system of ODEs for the bench
def bench_ODE(t, y, s0, omega, v_max, a_max):
    """
    Feedforward-based ODE to follow S(t) = S0 * sin(omega * t),
    matching velocity by adjusting amplitude and clipping to v_max / a_max.
    
    Parameters:
        t (float): Time
        y (array): [x, v] = position and velocity
        S0 (float): Amplitude of target motion
        omega (float): Frequency of motion
        v_max (float): Max allowed velocity
        a_max (float): Max allowed acceleration
        
    Returns:
        dydt: [dx/dt, dv/dt]
    """
    s, v = y

    ts = asin(s/(s0))/omega if abs(s / s0) <= 1 else 0 # time scale for the target motion
    ts = ts if v >= 0 else pi/omega - ts
    
    v0 = s0 * omega
    a0 = v0 * omega

    # Compute target velocity and acceleration
    s_target = s0 * sin(omega * ts)
    v_target = v0 * cos(omega * ts)
    a_target =-a0 * sin(omega * ts)
    
    # Scale amplitude if target velocity exceeds limit
    scale = v_target / v if v != 0 else 1.0
    if a_target == 0 and v_target != v:
        a_target = v_target
        
    a_target = a_target * scale + (v_target - v)

    if abs(s) >= s0:   
        a_target = -(abs(s) - s0) if s > 0 else (abs(s) - s0)

    if s > 0:
        a_max_pos = +a_max
        a_max_neg = -a_max * 2.0
    else:
        a_max_pos = +a_max * 2.0
        a_max_neg = -a_max

    # Clip acceleration to a_max
    v = np.clip(v, -v_max, v_max)
    a = np.clip(a_target, a_max_neg, a_max_pos)

    return [v, a]


class PTEmulatorService:
    
    def __init__(self, uh_initial, uv_initial, lh_initial, lv_initial, max_vertical_displacement, execution_interval, rabbitmq_config):
        # Initialize the PTEmulatorService with initial values and configuration
        self._l = logging.getLogger("PTEmulatorService")
        self._l.info("Initializing PTEmulatorService.")

        try:
            self.PT_Model = pt_model.PtModel()
        except Exception as e:
            self._l.error("Failed to initialize PTModel: %s", e, exc_info=True)
            raise

        self._rabbitmq = Rabbitmq(**rabbitmq_config)
        
        self.uh = uh_initial
        self.uv = uv_initial
        self.lh = lh_initial
        self.lv = lv_initial
        self.max_vertical_displacement = max_vertical_displacement
        self.lh_wanted = 100
        self.uv_wanted = 100
        self.VERTICAL_FREQ = (2*pi/60) / 4
        self.HORIZONTAL_FREQ = (2*pi/60) / 2
        self.VERTICAL_V_Max = self.uv_wanted * self.VERTICAL_FREQ
        self.HORIZONTAL_V_Max = self.lh_wanted * self.HORIZONTAL_FREQ
        self.VERTICAL_A_Max = self.VERTICAL_V_Max * self.VERTICAL_FREQ
        self.HORIZONTAL_A_Max = self.HORIZONTAL_V_Max * self.HORIZONTAL_FREQ
        self._S_bench_v, self._V_bench_v, self._a_bench_v = 0.0, 0.0, 0.0
        self._S_bench_h, self._V_bench_h, self._a_bench_h = 0.0, 0.0, 0.0
        # self.r = r_initial # do we need this for the emulator?
        self._execution_interval = execution_interval # seconds
        self._force_on = 0.0  

    def setup(self):
        self._rabbitmq.connect_to_server()

        # Declare local queues for the force messages
        self.forces_queue_name = self._rabbitmq.declare_local_queue(routing_key=ROUTING_KEY_FORCES)
        #self.load_queue_name = self._rabbitmq.declare_local_queue(routing_key=ROUTING_KEY_LOADS)
        #self.displacement_queue_name = self._rabbitmq.declare_local_queue(routing_key=ROUTING_KEY_DISPLACEMENTS)

        self._l.info(f"PTEmulatorService setup complete.")

    def _read_forces(self):
        # Read the forces from the RabbitMQ queue
        #self._l.debug("Reading forces from RabbitMQ.")
        msg = self._rabbitmq.get_message(self.forces_queue_name)
        #self._l.debug(f"Message received: {msg}")
        if msg is not None:
            return msg
        else:
            return None
    
    def check_control_commands(self):
        # Check if there are control commands
        force_cmd = self._read_forces()
        #self._l.debug(f"Control command: {force_cmd}")
        if force_cmd is not None:
            if 'forces' in force_cmd and force_cmd['forces'] is not None:
                self._l.info("Force command: %s", force_cmd["forces"])
                self._force_on = 1.0 if force_cmd['forces'] else 0.0

            if "horizontal_force" in force_cmd and force_cmd["horizontal_force"] is not None:
                self._l.info(f"Horizontal force command: {force_cmd['horizontal_force']}")
                self.lh_wanted = force_cmd["horizontal_force"]

            if "vertical_displacement" in force_cmd and force_cmd["vertical_displacement"] is not None:
                self._l.info(f"Vertical force command: {force_cmd['vertical_displacement']}")
                self.uv_wanted = force_cmd["vertical_displacement"]
                
            if "vertical_frequency" in force_cmd and force_cmd["vertical_frequency"] is not None:
                self._l.info(f"Vertical frequency command: {force_cmd['vertical_frequency']}")
                self.set_vertical_frequency(force_cmd["vertical_frequency"])
                
            if "horizontal_frequency" in force_cmd and force_cmd["horizontal_frequency"] is not None:
                self._l.info(f"Horizontal frequency command: {force_cmd['horizontal_frequency']}")
                self.set_horizontal_frequency(force_cmd["horizontal_frequency"])


    def emulate_pt(self):
        # Emulate the PT behavior based on the control commands
        #self._l.info("Emulating.")
        state = [self._force_on, self.lh, self.lv, self.uh, self.uv] # Current state of the PTEmulator
        #self._l.info(f"Current state: {state}")

        # Additional logic for the emulator can go here
        # the if statement is just hardcoded emulator behaviour for now! 
        # _uh, _uv, _lh, _lv, and _r need to be extracted from the simulation results (u, lf, r)
        if self._force_on == 1.0:
            #self._l.info("Force is on, setting displacements and forces from simulation results.")
            # Horizontal displacement

            #Run the ODE solver for the horizontal and vertical motion
            try:
                self.run_ODE()
            except Exception as e:
                self._l.error("ODE solver failed: %s", e, exc_info=True)
                raise

            #self._l.info("Running simulation...")
            try:
                [u, lf, r] = self.PT_Model.run_simulation()
            except Exception as e:
                self._l.error("Simulation failed: %s", e, exc_info=True)
                raise
            #self._l.info(f"Simulation completed. u = {u.shape}, lf = {lf.shape}, r = {r.shape}")

            node10_index = 10
            
            # Horizontal displacement
            try:
                self._uh = float(self.PT_Model.get_displacement(node10_index, fx)[0])
            except IndexError as e:
                self._l.error(f"Error retrieving horizontal displacement from u({node10_index},1): %s", e, exc_info=True)

            # Vertical displacement
            try:
                self._uv = float(self.PT_Model.get_displacement(node10_index, fz)[0])
            except IndexError as e:
                self._l.error(f"Error retrieving vertical displacement from u({node10_index},1): %s", e, exc_info=True)
            
            #Forces
            try:
                # Vertical force
                self._lh = float(self.PT_Model.get_load(node10_index, fx)[0])
                # Horizontal force
                self._lv = float(self.PT_Model.get_load(node10_index, fz)[0])
            except Exception as e:
                self._l.error(f"Error retrieving forces from PT_Model.get_loads(): %s", e, exc_info=True)
                self._l.error(f"Forces not set: lh = {self._lh}, lv = {self._lv}")

            
            #self._l.info(f"Displacements: uh={self._uh}, uv={self._uv}")
            #self._l.info(f"Forces: lh={self._lh}, lv={self._lv}")
            # Restoring force
            
            # self._r = r[something] # in case we need this for the emulator, we can put it here
        else:
            #self._l.info("Force is off, setting displacements and forces to zero.")
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
                # "restoring_force": self._r,
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
        except KeyboardInterrupt:
            self._l.info("Emulation loop interrupted by user.")
        except Exception as e:
            self._l.error("Emulation loop failed: %s", e, exc_info=True)

    def run_ODE(self):
        #self._l.info(f"Current state vertical: {state_v}")
        state_h = [self._S_bench_h, self._V_bench_h] # Current state of the PTEmulator
        state_v = [self._S_bench_v, self._V_bench_v] # Current state of the PTEmulator

        try:
            sol_h = solve_ivp(
                lambda t, y: bench_ODE(t, y, self.lh_wanted, self.HORIZONTAL_FREQ, self.HORIZONTAL_V_Max, self.HORIZONTAL_A_Max),
                [0.0, self._execution_interval], state_h, t_eval=np.linspace(0.0, self._execution_interval, 2))
            sol_v = solve_ivp(
                lambda t, y: bench_ODE(t, y, self.uv_wanted, self.VERTICAL_FREQ, self.VERTICAL_V_Max, self.VERTICAL_A_Max),
                [0.0, self._execution_interval], state_v, t_eval=np.linspace(0.0, self._execution_interval, 2))
        except Exception as e:
            self._l.error("ODE solver failed: %s", e, exc_info=True)
            raise

        # Update the state variables
        self._S_bench_h = sol_h.y[0, 1]
        self._V_bench_h = sol_h.y[1, 1]
        self._S_bench_v = sol_v.y[0, 1]
        self._V_bench_v = sol_v.y[1, 1]

        self._l.debug(f"Setting loads and displacements in PTModel. Sv: {np.round(self._S_bench_v,2)}, Sh: {np.round(self._S_bench_h,2)}")
        self._l.debug(f"Setting loads and displacements in PTModel. Vv: {np.round(self._V_bench_v,2)}, Vh: {np.round(self._V_bench_h,2)}")

        try:
            self.PT_Model.set_loads_between_nodes(1, self._S_bench_h, [9,10])
            self.PT_Model.set_displacements_between_nodes(1, self._S_bench_v,[5,10])
        except Exception as e:
            self._l.error("Failed to set load in PTModel: %s", e, exc_info=True)
            raise

    def set_horizontal_frequency(self, frequency):
        # Set the horizontal frequency for the emulator
        #self._l.info(f"Setting horizontal frequency to {frequency}.")
        self.HORIZONTAL_FREQ = (2*pi / 60) / frequency
        self.HORIZONTAL_V_Max = self.lh_wanted * self.HORIZONTAL_FREQ * 1.1
        self.HORIZONTAL_A_Max = self.HORIZONTAL_V_Max * self.HORIZONTAL_FREQ * 1.1
        self._l.info(f"Horizontal frequency set to {self.HORIZONTAL_FREQ}, V_Max: {self.HORIZONTAL_V_Max}, A_Max: {self.HORIZONTAL_A_Max}.")
        
    def set_vertical_frequency(self, frequency):
        # Set the vertical frequency for the emulator
        #self._l.info(f"Setting vertical frequency to {frequency}.")
        self.VERTICAL_FREQ = (2*pi / 60) / frequency
        self.VERTICAL_V_Max = self.uv_wanted * self.VERTICAL_FREQ * 1.1
        self.VERTICAL_A_Max = self.VERTICAL_V_Max * self.VERTICAL_FREQ * 1.1 
        self._l.info(f"Vertical frequency set to {self.VERTICAL_FREQ}, V_Max: {self.VERTICAL_V_Max}, A_Max: {self.VERTICAL_A_Max}.")
    
    
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
        max_vertical_displacement = 70.0,
        execution_interval = 3.0,
        rabbitmq_config=config["rabbitmq"])

    service.setup()
    
    # Start the PTEmulatorService
    service.start_emulation()
