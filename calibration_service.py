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
            'displacements': None,
            'boundaries': None
        }

        self.DT_Model = model

    def get_calibration_data(self):
        return self.calibration_data

    def set_calibration_state(self, displacements):
        self._l.debug("Setting calibration displacements...")
        self.calibration_data['displacements'] = displacements
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
    
    def calibrate_model(self):
        self._l.debug("Starting calibration...")
        E = self.DT_Model.get_beampars(16).E  # Get the current value of E from the DT model

        initial_guess = [E]

        if self.calibration_data['boundaries'] is None:
            res = least_squares(self.cost, initial_guess, diff_step=.1)
        else:
            res = least_squares(self.cost, initial_guess, bounds=self.calibration_data['boundaries'])
        self._l.debug(f"Calibration result: {res}")
        self.accuracy = res.cost
        print("self.res: ", res)
        self.res = res.x[0]  # Extract the optimized value of E

        self.DT_Model.set_beampars(16, 'E', self.res)  # Set the optimized value of E in the DT model
        self._l.debug(f"Calibration completed. Optimized E: {self.res}")


    def cost(self, P_guess):
        # noise added to test calibration before DT is done
        # noise = 30  # Add noise to the guess
        E = P_guess
        self._l.info(f"Cost function called with P_guess: {P_guess}")

        self.DT_Model.set_beampars(16, 'E', E) # Set the beam parameters for the DT model

        self._l.info(f"Setting beam parameters to E: {self.DT_Model.get_beampars(16).E}")

        try:
            self.DT_Model.run_simulation()
        except:
            self._l.info(f"Cost for {P_guess}: Simulation failed")
            return 1e6  # Return a high cost to avoid this solution
        
        displacements = np.array([self.DT_Model.get_displacement_between_nodes(9, 10)[2], 
                                  self.DT_Model.get_displacement_between_nodes(5, 10)[2],
                                  self.DT_Model.get_load(10, fx)[0],
                                  self.DT_Model.get_load(10, fz)[0]])
        recieved_displacements = self.calibration_data['displacements']
        differences = recieved_displacements - displacements
        self._l.info(f"Displacements: {displacements}")
        self._l.info(f"Received displacements: {recieved_displacements}")
        self._l.info(f"Differences: {differences}")
        sum_sq_dff = sum(differences**2)
        self._l.info(f"Cost for {P_guess}: {sum_sq_dff}")
        return sum_sq_dff