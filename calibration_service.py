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
    
    def calibrate_model(self):
        self._l.debug("Starting calibration...")
        E = self.DT_Model.get_beampars(16).E  # Get the current value of E from the DT model
        Ec = 0.5  # Set a default value for Ec

        initial_guess = [E, Ec]

        state = np.array([  self.DT_Model.get_displacement_between_nodes(9, 10)[2], 
                            self.DT_Model.get_displacement_between_nodes(5, 10)[2],
                            self.DT_Model.get_load(10, fx)[0],
                            self.DT_Model.get_load(10, fz)[0]])
        
        self._l.debug(f"Digital Twin state: {state}")
        self._l.debug(f"Recieved state: {self.calibration_data['state']}")


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
        self._l.info(f"Cost function called with P_guess: {P_guess}")
        E, Ec = P_guess

        self.DT_Model.set_beampars(16, 'E', E) # Set the beam parameters for the DT model

        try:
            #self._l.info(f"Setting displacements between nodes 5 and 10: {self.calibration_data['state'].tolist()[1]}")
            self.DT_Model.set_displacements_between_nodes(self.calibration_data['state'].tolist()[1],[5,10])
            self._l.info(f"Setting loads on node 10: {self.calibration_data['state'].tolist()}")
            #self._l.info(f"Running simulation with E: {E}")
            self.DT_Model.run_simulation()
            #self._l.info("Simulation completed successfully.")
        except:
            self._l.info(f"Cost for {P_guess}: Simulation failed")
            return 1e6  # Return a high cost to avoid this solution
        
        state = np.array([  self.DT_Model.get_displacement_between_nodes(9, 10)[2], 
                            self.DT_Model.get_displacement_between_nodes(5, 10)[2],
                            self.DT_Model.get_load(10, fx)[0],
                            self.DT_Model.get_load(10, fz)[0]])
        recieved_state = self.calibration_data['state']
        differences = recieved_state - state
        self._l.info(f"Displacements: {state}")
        #self._l.info(f"Received displacements: {recieved_displacements}")
        #self._l.info(f"Differences: {differences}")
        sum_sq_dff = sum(differences**2)
        self._l.info(f"Cost for {P_guess}: {sum_sq_dff}")
        #self._l.info(f"Getting beam parameters: {self.DT_Model.get_beampars(16).E}")
        return sum_sq_dff