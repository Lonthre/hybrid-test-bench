# Configure python path to load hybrid test bench modules
import sys
import os
import logging
import logging.config
import time
import numpy as np
from math import *

from scipy.integrate import solve_ivp

# Configure logging from the logging.conf file
logging.config.fileConfig('logging.conf')

# Get the current working directory. Should be hybrid-test-bench
current_dir = os.getcwd()

assert os.path.basename(current_dir) == 'hybrid-test-bench', 'Current directory is not hybrid-test-bench'

# Get the parent directory. Should be the root of the repository
parent_dir = current_dir

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
    def __init__(self, AMP, Period, execution_interval):
        # Initialize the actuator controller with the given parameters.
        #AMP: Amplitude of the actuator [kN/mm]
        #Period: Period of the actuator [minutes]
        #execution_interval: Execution interval for the ODE solver [seconds]

        # Set up logging
        self._l = logging.getLogger("Actuator")
        self._l.info("Initializing ActuatorController.")

        self._S, self._V, self._a_bench = 0.0, 0.0, 0.0


        self.AMP = AMP
        self.set_period(Period)

        self._execution_interval = execution_interval # seconds

        self._l.info(f"ActuatorController initialized")

    def step_simulation(self):
        #Run the ODE solver for the horizontal and vertical motion
        try:
            self.run_ODE()
        except Exception as e:
            self._l.error("ODE solver failed: %s", e, exc_info=True)
            raise

        return self._S

    def run_ODE(self):
        #self._l.info(f"Current state vertical: {state_v}")
        state = [self._S, self._V] # Current state of the PTEmulator

        try:
            sol = solve_ivp(
                lambda t, y: bench_ODE(t, y, self.AMP, self.FREQ, self.V_Max, self.A_Max),
                [0.0, self._execution_interval], state, t_eval=np.linspace(0.0, self._execution_interval, 2))
        except Exception as e:
            self._l.error("ODE solver failed: %s", e, exc_info=True)
            raise

        # Update the state variables
        self._S = sol.y[0, 1]
        self._V = sol.y[1, 1]

        self._l.debug(f"Setting loads and displacements in PTModel. Sv: {np.round(self._S,2)}, Vh: {np.round(self._V,2)}")

    def set_amplitude(self, amp):
        # Set the amplitude for the actuator [kN/mm]
        #self._l.info(f"Setting amplitude to {amplitude}.")
        self.AMP = amp
        self.V_Max = self.AMP * self.FREQ * 1.1
        self.A_Max = self.V_Max * self.FREQ * 1.1 
        self._l.info(f"Amplitude set to {self.AMP}, V_Max: {self.V_Max}, A_Max: {self.A_Max}.")
  
    def set_frequency(self, freq):
        # Set the frequency for the actuator [RPM]
        #self._l.info(f"Setting frequency to {frequency}.")
        self.FREQ = freq/60
        self.V_Max = self.AMP * self.FREQ * 1.1
        self.A_Max = self.V_Max * self.FREQ * 1.1 
        self._l.info(f"Frequency set to {self.FREQ}, V_Max: {self.V_Max}, A_Max: {self.A_Max}.")

    def set_period(self, period):
        # Set the period for the actuator [minutes]
        #self._l.info(f"Setting frequency to {frequency}.")
        self.T = period
        self.FREQ = (2*pi / 60) / self.T
        self.V_Max = self.AMP * self.FREQ * 1.1
        self.A_Max = self.V_Max * self.FREQ * 1.1 
        self._l.info(f"Period set to {self.T}, V_Max: {self.V_Max}, A_Max: {self.A_Max}.")

    def calibrate(self, calibration_data):
        # Calibrate the actuator with the given data
        #self._l.info(f"Calibrating actuator with data: {calibration_data}.")
        self._S = calibration_data

        omega = self.FREQ
        s0 = self.AMP
        v0 = s0 * omega

        s = self._S
        v = self._V

        ts = asin(s/(s0))/omega if abs(s / s0) <= 1 else 0 # time scale for the target motion
        ts = ts if v >= 0 else pi/omega - ts

        self._V = v0 * omega * cos(omega * ts)

        self._l.info(f"Calibration data set to {calibration_data}.")