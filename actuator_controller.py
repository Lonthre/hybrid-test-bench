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

        self.step = 0
        self._S, self._V, self._a_bench = 0.0, 0.0, 0.0


        self.AMP = AMP
        self.set_period(Period)

        self._execution_interval = execution_interval # seconds

        self._l.info(f"ActuatorController initialized")

    def get_state(self):
        # Get the current state of the actuator
        return self._S

    def step_simulation(self):
        #Run the ODE solver for the horizontal and vertical motion
        self.step += 1
        #self._l.info(f"Running step {self.step} of the actuator simulation.")
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

        #self._l.debug(f"Setting loads and displacements in PTModel. Sv: {np.round(self._S,2)}, Vh: {np.round(self._V,2)}")

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
        self.step = 0
        self.particles = []
        self.weights = []
        self.last_step = 0

        self.results = []
        self.uncertainty_estimate = []


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

    def pf_state(self, r_state, num_particles=100, obs_noise = 1, S_noise_std=0.001, V_noise_std=0.001):
        # Inputs to particle filter
        self._l.info("Running particle filter state estimation...")
        self._l.debug("Current state: S = %s, V = %s", self._S, self._V)
        self._l.debug("Current observation: %s", r_state)
        
        if not hasattr(self, 'particles') or self.particles is None or len(self.particles) == 0:
            # Initial particles. Each particle represents a possible system state.
            self.particles = np.zeros((num_particles, 2))  # Two state variables: S, V
            self.particles[:,0] = self._S # Initial actuator position
            self.particles[:,1] = self._V # Initial actuator velocity
            self.weights = np.ones(num_particles) / num_particles  # Uniform weights
            self.last_step = self.step

            self.results = np.zeros(2)
            self.uncertainty_estimate = np.zeros(2)

        self.observation_noise_std = obs_noise # Can be tuned

        self.process_S_noise_std = self.AMP * S_noise_std # Can be tuned
        self.process_V_noise_std = self.V_Max * V_noise_std # Can be tuned

        # When was the last step?:
        self._l.debug("Last step: %s, Current step: %s", self.last_step, self.step)
        d_step = self.step - self.last_step
        self.last_step = self.step

        res, fault = self.pf_step(r_state, d_step)
        S, V = res

        self._S = S
        self._V = V

        self._l.debug("Updated state: S = %s, V = %s", S, V)
        return S, fault

    def pf_step(self, r_state, d_step=1):
        # Perform a single step of the particle filter
        self._l.debug("Performing particle filter step... steps: %s", d_step)

        num_particles = self.particles.shape[0]


        for particle in self.particles:
            S_noise = np.random.normal(0, self.process_S_noise_std, 1)[0]
            V_noise = np.random.normal(0, self.process_V_noise_std, 1)[0]

            state = [particle[0]+S_noise, particle[1]+V_noise] # Current state of the Particle

            try:
                sol = solve_ivp(
                    lambda t, y: bench_ODE(t, y, self.AMP, self.FREQ, self.V_Max, self.A_Max),
                    [0.0, self._execution_interval*(1+d_step)], state, t_eval=np.linspace(0.0, self._execution_interval * d_step, d_step +1))
                

                particle[0] = sol.y[0, d_step]  # Update particle position
                particle[1] = sol.y[1, d_step]  # Update particle velocity

            except Exception as e:
                self._l.error("ODE solver failed: %s", e, exc_info=True)
                raise
        self._l.debug("Solution: %s", sol.y)

        observation = r_state

        # Update weights based on observation likelihood
        self.weights = np.exp(-0.5 * ((self.particles[:, 0] - observation) / self.observation_noise_std) ** 2)
        self.weights += 1e-300  # Avoid zero weights, just for numerical stability
        self.weights /= np.sum(self.weights)  # Normalize: turn the weights into a probability distribution

        # Resample particles
        indices = np.random.choice(num_particles, num_particles, p=self.weights)
        self.particles = self.particles[indices, :]
        self.weights = np.ones(num_particles) / num_particles  # Reset weights
        
        self.results[0] = np.mean(self.particles[:, 0]) # actuator position
        self.results[1] = np.mean(self.particles[:, 1]) # actuator velocity
        self.uncertainty_estimate = np.std(self.particles[:, 0])

        self._l.debug("Particle filter step completed. Results: %s, Uncertainty estimate: %s", self.results, self.uncertainty_estimate)

        return self.results, self.uncertainty_estimate

