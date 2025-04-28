
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

    # Clip acceleration to a_max
    v = np.clip(v, -v_max, v_max)
    a = np.clip(a_target, -a_max, a_max)

    return [v, a]


class PTEmulatorService:
    
    def __init__(self, uh_initial, uv_initial, lh_initial, lv_initial, execution_interval, rabbitmq_config):

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
        self.max_vertical_displacement = 5.0
        self.lh_wanted = 100
        self.lv_wanted = 100
        self.VERTICAL_FREQ = 4 / (pi*60)
        self.HORIZONTAL_FREQ = 2 / (pi*60)
        self.VERTICAL_V_Max = self.lv_wanted * self.VERTICAL_FREQ
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
        # Emulate the PT behavior based on the control commands
        self._l.info("Emulating.")
        state = [self._force_on, self.lh, self.lv, self.uh, self.uv] # Current state of the PTEmulator
        #self._l.info(f"Current state: {state}")

        # Additional logic for the emulator can go here
        # the if statement is just hardcoded emulator behaviour for now! 
        # _uh, _uv, _lh, _lv, and _r need to be extracted from the simulation results (u, lf, r)
        if self._force_on == 1.0:
            self._l.info("Force is on, setting displacements and forces from simulation results.")
            # Horizontal displacement

            #Run the ODE solver for the horizontal and vertical motion
            try:
                self.run_ODE()
            except Exception as e:
                self._l.error("ODE solver failed: %s", e, exc_info=True)
                raise

            self._l.info("Running simulation...")
            try:
                [u, lf, r] = self.PT_Model.run_simulation()
            except Exception as e:
                self._l.error("Simulation failed: %s", e, exc_info=True)
                raise
            #self._l.info(f"Simulation completed. u = {u.shape}, lf = {lf.shape}, r = {r.shape}")

            node10_index = 10

            #self._l.debug(f"Finding dof for node: {node10_index}")
            dof10_horizontal = self.PT_Model.model.find_dofs([[node10_index, 1]]).squeeze()
            dof10_vertical = self.PT_Model.model.find_dofs([[node10_index, 3]]).squeeze()
            #self._l.debug(f"Node 10 dof: {dof10_horizontal}, {dof10_vertical}")

            try:
                self._uh = float(u[dof10_horizontal, 1])
            except IndexError as e:
                self._l.error(f"Error retrieving horizontal displacement from u({dof10_horizontal},1): %s", e, exc_info=True)

            # Vertical displacement
            try:
                self._uv = float(u[dof10_vertical, 1])
            except IndexError as e:
                self._l.error(f"Error retrieving vertical displacement from u({dof10_vertical},1): %s", e, exc_info=True)
            
            #Forces
            try:
                [lv, lh] = self.PT_Model.get_load() 
                # Horizontal force
                self._lv = float(lv) 
                # Vertical force
                self._lh = float(lh)
            except Exception as e:
                self._l.error(f"Error retrieving forces from PT_Model.get_loads(): %s", e, exc_info=True)
                self._l.error(f"Forces not set: lh = {self._lh}, lv = {self._lv}")

            
            #self._l.info(f"Displacements: uh={self._uh}, uv={self._uv}")
            #self._l.info(f"Forces: lh={self._lh}, lv={self._lv}")
            # Restoring force
            
            # self._r = r[something] # in case we need this for the emulator, we can put it here
        else:
            self._l.info("Force is off, setting displacements and forces to zero.")
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

        self._l.info("PT script executed successfully.")
        
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
        self._l.debug(f"Message sent to {ROUTING_KEY_STATE}.")
        self._l.debug(message)
    
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
        state_v = [self._S_bench_v, self._V_bench_v] # Current state of the PTEmulator
        #self._l.info(f"Current state vertical: {state_v}")

        try:
            sol_v = solve_ivp(
                lambda t, y: bench_ODE(t, y, self.lv_wanted, self.VERTICAL_FREQ, self.VERTICAL_V_Max, self.VERTICAL_A_Max),
                [0.0, self._execution_interval], state_v, t_eval=np.linspace(0.0, self._execution_interval, 2))
        except Exception as e:
            self._l.error("ODE solver failed: %s", e, exc_info=True)
            raise
        #self._l.debug(f"ODE solution vertical: {sol_v}")

        # Update the state variables
        self._S_bench_v = sol_v.y[0, 1]
        self._V_bench_v = sol_v.y[1, 1]
        #self._a_bench_v = sol_v.y[2, -1]
        #self._l.debug(f"ODE solution vertical: {self._S_bench_v}, {self._V_bench_v}")
        
        state_h = [self._S_bench_h, self._V_bench_h] # Current state of the PTEmulator
        #elf._l.info(f"Current state horizontal: {state_h}")

        try:
            sol_h = solve_ivp(
                lambda t, y: bench_ODE(t, y, self.lh_wanted, self.HORIZONTAL_FREQ, self.HORIZONTAL_V_Max, self.HORIZONTAL_A_Max),
                [0.0, self._execution_interval], state_h, t_eval=np.linspace(0.0, self._execution_interval, 2))
        except Exception as e:
            self._l.error("ODE solver failed: %s", e, exc_info=True)
            raise
        #self._l.debug(f"ODE solution horizontal: {sol_h.y}")

        # Update the state variables
        self._S_bench_h = sol_h.y[0, 1]
        self._V_bench_h = sol_h.y[1, 1]
        #self._a_bench_h = sol_h.y[2, -1]

        self._l.debug(f"Setting load: {1,self._S_bench_v, self._S_bench_h}")
        try:
            self.PT_Model.set_loads(1,self._S_bench_v, self._S_bench_h)
        except Exception as e:
            self._l.error("Failed to set load in PTModel: %s", e, exc_info=True)
            raise

    
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
