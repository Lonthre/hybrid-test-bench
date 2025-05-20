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

class ActuatorController:
    def __init__(self, lh_wanted, uv_wanted, vertical_frequency, horizontal_frequency, execution_interval):
        self._l = logging.getLogger("ActuatorController")
        self._l.info("Initializing ActuatorController.")

        self._lh_wanted = lh_wanted
        self._uv_wanted = uv_wanted
        self._vertical_frequency = vertical_frequency
        self._horizontal_frequency = horizontal_frequency

        self.VERTICAL_FREQ = (2*pi/60) / 4
        self.HORIZONTAL_FREQ = (2*pi/60) / 2
        self.VERTICAL_V_Max = self._uv_wanted * self.VERTICAL_FREQ
        self.HORIZONTAL_V_Max = self._lh_wanted * self.HORIZONTAL_FREQ
        self.VERTICAL_A_Max = self.VERTICAL_V_Max * self.VERTICAL_FREQ
        self.HORIZONTAL_A_Max = self.HORIZONTAL_V_Max * self.HORIZONTAL_FREQ
        self._S_bench_v, self._V_bench_v, self._a_bench_v = 0.0, 0.0, 0.0
        self._S_bench_h, self._V_bench_h, self._a_bench_h = 0.0, 0.0, 0.0
        self._execution_interval = execution_interval # seconds

        self._l.info(f"ActuatorController initialized")

    def step_simulation(self, PT_Model):
        # Log the values received.
        # self._l.info(f"Received state sample: {body_json}")

        #Run the ODE solver for the horizontal and vertical motion
        try:
            self.run_ODE(PT_Model)
        except Exception as e:
            self._l.error("ODE solver failed: %s", e, exc_info=True)
            raise

        #self._l.info("Running simulation...")
        try:
            [u, lf, r] = PT_Model.run_simulation()
        except Exception as e:
            self._l.error("Simulation failed: %s", e, exc_info=True)
            raise
        #self._l.info(f"Simulation completed. u = {u.shape}, lf = {lf.shape}, r = {r.shape}")

        node10_index = 10
        
        # Horizontal displacement
        try:
            self._uh = float(PT_Model.get_displacement(node10_index, fx)[0])
        except IndexError as e:
            self._l.error(f"Error retrieving horizontal displacement from u({node10_index},1): %s", e, exc_info=True)

        # Vertical displacement
        try:
            self._uv = float(PT_Model.get_displacement(node10_index, fz)[0])
        except IndexError as e:
            self._l.error(f"Error retrieving vertical displacement from u({node10_index},1): %s", e, exc_info=True)
        
        #Forces
        try:
            # Vertical force
            self._lh = float(PT_Model.get_load(node10_index, fx)[0])
            # Horizontal force
            self._lv = float(PT_Model.get_load(node10_index, fz)[0])
        except Exception as e:
            self._l.error(f"Error retrieving forces from PT_Model.get_loads(): %s", e, exc_info=True)
            self._l.error(f"Forces not set: lh = {self._lh}, lv = {self._lv}")
        
        return self._uh, self._uv, self._lh, self._lv


    def run_ODE(self, PT_Model):
        #self._l.info(f"Current state vertical: {state_v}")
        state_h = [self._S_bench_h, self._V_bench_h] # Current state of the PTEmulator
        state_v = [self._S_bench_v, self._V_bench_v] # Current state of the PTEmulator

        try:
            sol_h = solve_ivp(
                lambda t, y: bench_ODE(t, y, self._lh_wanted, self.HORIZONTAL_FREQ, self.HORIZONTAL_V_Max, self.HORIZONTAL_A_Max),
                [0.0, self._execution_interval], state_h, t_eval=np.linspace(0.0, self._execution_interval, 2))
            sol_v = solve_ivp(
                lambda t, y: bench_ODE(t, y, self._uv_wanted, self.VERTICAL_FREQ, self.VERTICAL_V_Max, self.VERTICAL_A_Max),
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
            PT_Model.set_loads_between_nodes(1, self._S_bench_h, [9,10])
            PT_Model.set_displacements_between_nodes(1, self._S_bench_v,[5,10])
        except Exception as e:
            self._l.error("Failed to set load in PTModel: %s", e, exc_info=True)
            raise

    def set_horizontal_frequency(self, frequency):
        # Set the horizontal frequency for the emulator
        #self._l.info(f"Setting horizontal frequency to {frequency}.")
        self.HORIZONTAL_FREQ = (2*pi / 60) / frequency
        self.HORIZONTAL_V_Max = self._lh_wanted * self.HORIZONTAL_FREQ * 1.1
        self.HORIZONTAL_A_Max = self.HORIZONTAL_V_Max * self.HORIZONTAL_FREQ * 1.1
        self._l.info(f"Horizontal frequency set to {self.HORIZONTAL_FREQ}, V_Max: {self.HORIZONTAL_V_Max}, A_Max: {self.HORIZONTAL_A_Max}.")
        
    def set_vertical_frequency(self, frequency):
        # Set the vertical frequency for the emulator
        #self._l.info(f"Setting vertical frequency to {frequency}.")
        self.VERTICAL_FREQ = (2*pi / 60) / frequency
        self.VERTICAL_V_Max = self._uv_wanted * self.VERTICAL_FREQ * 1.1
        self.VERTICAL_A_Max = self.VERTICAL_V_Max * self.VERTICAL_FREQ * 1.1 
        self._l.info(f"Vertical frequency set to {self.VERTICAL_FREQ}, V_Max: {self.VERTICAL_V_Max}, A_Max: {self.VERTICAL_A_Max}.")