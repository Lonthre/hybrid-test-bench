from scipy.optimize import least_squares

import logging
import logging.config

# Configure logging from the logging.conf file
logging.config.fileConfig('logging.conf')

import numpy as np
import dt_model as dt_model

class CalibrationService:
    def __init__(self, DT_Model):
        self._l = logging.getLogger('CalibrationService')
        self._l.debug("Initialising CalibrationService...")

        self.calibration_data = {
            'displacements': None,
            'boundaries': None
        }

        try:
            self.DT_Model = dt_model.DtModel()
        except Exception as e:
            self._l.error(f"Failed to initialize DT_Model: {e}")
            raise

    def get_calibration_data(self):
        return self.calibration_data

    def set_calibration_displacement(self, displacements):
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
        E = self.DT_Model.get_beampars(16).E  # Get the current value of E from the PT model

        initial_guess = [E]

        if self.calibration_data['boundaries'] is None:
            res = least_squares(self.cost, initial_guess)
        else:
            res = least_squares(self.cost, initial_guess, bounds=self.calibration_data['boundaries'])
        self._l.debug(f"Calibration result: {res}")
        self.accuracy = res.cost
        self.res = res.x[0]  # Extract the optimized value of E

        self.DT_Model.set_beampars(16, 'E', self.res)  # Set the optimized value of E in the PT model
        self._l.debug(f"Calibration completed. Optimized E: {self.res}")


    def cost(self, P_guess):
        #noise added to test calibration before DT is done
        noise = 30  # Add noise to the guess
        E = P_guess + noise
        
        self.DT_Model.set_beampars(16, 'E', E) # Set the beam parameters for the PT model
        #PT_Model.set_beampars(16, 'E', E) # Set the beam parameters for the PT model  

        try:
            self.DT_Model.run_simulation()
        except:
            print(f"Cost for {P_guess}: Simulation failed")
            return 1e6  # Return a high cost to avoid this solution
        
        displacements = self.DT_Model.get_displacements()
        recieved_displacements = self.calibration_data['displacements']
        differences = recieved_displacements - displacements
        sum_sq_dff = sum(differences**2)
        print(f"Cost for {P_guess}: {sum_sq_dff}")
        return sum_sq_dff