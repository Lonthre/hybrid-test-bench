from scipy.optimize import least_squares

import logging
import logging.config

# Configure logging from the logging.conf file
logging.config.fileConfig('logging.conf')

import numpy as np
import dt_model as dt_model

# Define the global variables for the model
fx, fy, fz, mx, my, mz = 1, 2, 3, 4, 5, 6 # force and moment indices

class CalibrationService:
    def __init__(self, model):
        self._l = logging.getLogger('CalibrationService')
        self._l.debug("Initialising CalibrationService...")

        self.calibration_data = {
            'state': None,
            'boundaries': ([0, 0], [250e3,1])
        }

        self.DT_Model = model

    def get_calibration_data(self):
        return self.calibration_data

    def set_calibration_state(self, state):
        self._l.debug("Setting calibration state...")
        self.calibration_data['state'] = state
        return "Calibration data updated successfully."
    
    def get_DT_Model(self):
        return self.DT_Model
    
    def set_DT_Model(self, new_model):
        self._l.debug("Setting DT_Model...")
        self.DT_Model = new_model
        return "Decision Tree model updated successfully."
    
    def get_calibration_results(self):
        # Placeholder for actual calibration results
        return {
           'cost': self.accuracy,
           'results': self.res
        }       
    
    def set_boundaries(self, new_boundaries):
        self._l.debug("Setting calibration boundaries...")
        self.calibration_data['boundaries'] = new_boundaries
        return "Boundaries updated successfully."
    
    def calibrate_model(self, model=None):
        self._l.info("Starting calibration...")
        if model is not None:
            self._l.debug("Using provided model for calibration.")
            self.DT_Model = model
        else:
            self._l.debug("Using existing DT_Model for calibration.")
        E = self.DT_Model.get_beampars(16).E  # Get the current value of E from the DT model
        Ec = 0.5  # Set a default value for Ec

        initial_guess = [E, Ec]

        self.DT_Model.run_simulation()

        state = np.array([  self.DT_Model.get_displacement_between_nodes(9, 10), 
                            self.DT_Model.get_displacement_between_nodes(5, 10),
                            self.DT_Model.get_load(10, fx),
                            self.DT_Model.get_load(10, fz)])
        
        self._l.info(f"Digital Twin state: {state}")
        self._l.info("Recieved state: %s", self.calibration_data['state'])


        if self.calibration_data['boundaries'] is None:
            self._l.debug("No boundaries set for calibration. Using default boundaries.")
            res = least_squares(self.cost, initial_guess)
        else:
            self._l.debug(f"Using boundaries: {self.calibration_data['boundaries']}")
            res = least_squares(self.cost, initial_guess, bounds=self.calibration_data['boundaries'])
        self._l.info(f"Calibration result: {res}")
        self.accuracy = res.cost
        self.res = res.x[0]  # Extract the optimized value of E

        self.DT_Model.set_beampars(16, 'E', self.res)  # Set the optimized value of E in the DT model
        self._l.info(f"Calibration completed. Optimized E: {self.res}")
        return self.DT_Model


    def cost(self, P_guess):
        # noise added to test calibration before DT is done
        # noise = 30  # Add noise to the guess
        self._l.debug("")
        self._l.debug(f"Cost function called with P_guess: {P_guess}")
        E, Ec = P_guess

        self.DT_Model.set_beampars(16, 'E', E) # Set the beam parameters for the DT model

        try:
            diff = 1
            while diff > 1e-10:
                F, U0, U = self.DT_Model.update_loads_from_displacements_between_nodes()
                self._l.debug("Force needed to reach U: %s is F: %s, Current U: %s     - Diff: %s", U0, F, U, diff)
                diff = U - U0

            self.DT_Model.run_simulation()
        except:
            self._l.debug(f"Cost for {P_guess}: Simulation failed")
            return 1e6  # Return a high cost to avoid this solution
        
        state = np.array([  self.DT_Model.get_displacement_between_nodes(9, 10), 
                            self.DT_Model.get_displacement_between_nodes(5, 10),
                            self.DT_Model.get_load(10, fx),
                            self.DT_Model.get_load(10, fz)])
        recieved_state = self.calibration_data['state']
        differences = recieved_state - state
        self._l.debug(f"State: {state}")
        #self._l.info(f"Received displacements: {recieved_displacements}")
        #self._l.info(f"Differences: {differences}")
        sum_sq_dff = sum(differences**2)
        self._l.debug(f"Cost for {P_guess}: {sum_sq_dff}")
        self._l.debug("")
        #self._l.info(f"Getting beam parameters: {self.DT_Model.get_beampars(16).E}")
        return sum_sq_dff