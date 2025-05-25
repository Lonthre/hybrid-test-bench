#!/usr/bin/env python
# coding: utf-8

from math import *
import numpy as np


from yafem.elem import *
from yafem.simulation import *

import logging
import logging.config

# Configure logging from the logging.conf file
logging.config.fileConfig('logging.conf')

# Define the global variables for the model
fx, fy, fz, mx, my, mz = 1, 2, 3, 4, 5, 6


_lb1 = 917.0  # length of the short beam [mm]
_lb2 = 1786.0  # length of the long beam [mm]
_lb3 = 603.0  # length of the aluminium beam [mm]

_lc1 = 996.0  # length of the short column [mm]
_lc2 = 1262.0 + 380.0  # length of the long column [mm]

_ls1 = 266.0  # length of the support beam [mm]
_ls2 = 266.0  # length of the support beam [mm]
_hs1 = 321.0  # length of the support beam [mm]
_hs2 = 206.5  # length of the support beam [mm]

_Es = 210e3  # Young's modulus (Steel) [N/mm^2]
_Ea = 70e3  # Young's modulus (~Aluminium) [N/mm^2]

# red - steel
# blue - steel
# green - aluminium

# cross-section parameters (frame)
_b1 = 150.0  # cross section width [mm]
_h1 = 100.0  # cross section height [mm]
_s1 = 5.0  # cross section thickness [mm]
_A1 = _b1 * _h1 - (_b1 - 2 * _s1) * (_h1 - 2 * _s1)  # cross-section area [mm2]
_Ixx1 = _b1 * _h1 ** 3 / 12 - (_b1 - 2 * _s1) * (_h1 - 2 * _s1) ** 3 / 12  # moment of inertia about x-axis [mm4]
_Iyy1 = _h1 * _b1 ** 3 / 12 - (_h1 - 2 * _s1) * (_b1 - 2 * _s1) ** 3 / 12  # moment of inertia about y-axis [mm4]

# cross-section parameters (supports)
_b2 = 40.0  # cross section width [mm]
_h2 = 60.0  # cross section height [mm]
_s2 = 5.0  # cross section thickness [mm]
_A2 = _b2 * _h2 - (_b2 - 2 * _s2) * (_h2 - 2 * _s2)  # cross-section area [mm2]
_Ixx2 = _b2 * _h2 ** 3 / 12 - (_b2 - 2 * _s2) * (_h2 - 2 * _s2) ** 3 / 12  # moment of inertia about x-axis [mm4]
_Iyy2 = _h2 * _b2 ** 3 / 12 - (_h2 - 2 * _s2) * (_b2 - 2 * _s2) ** 3 / 12  # moment of inertia about y-axis [mm4]

# cross-section parameters (aluminium)
_b3 = 80.0  # cross section width [mm]
_h3 = 5.0  # cross section height [mm]
_A3 = _b3 * _h3  # cross-section area [mm2]
_Ixx3 = _b3 * _h3 ** 3 / 12  # moment of inertia about x-axis [mm4]
_Iyy3 = _h3 * _b3 ** 3 / 12  # moment of inertia about y-axis [mm4]


class PtModel:
    def __init__(self):
        self._l = logging.getLogger('PTModel')
        self._l.debug("Initialising PT model...")

        self.nodes = None
        self.elements = []
        self.model = None
        self.model_pars = dict()
        
        self._c = [[1, fx], [1, fy], [1, fz], [1, mx],
                            [3, fy], [3, fz], [3, mx]]

        self._f = []
        self._fn = [[]]
        self._fs = [[]]
        self._u = []
        self._un = [[]]
        self._us = [[]]

        
        self._setup_nodes()
        self._setup_elements()
        self._setup_model()
        self.run_simulation()
        

    # nodal parameters (x, y, z)
    def _setup_nodes(self):
        self._l.debug("Setting up nodes.")

        nodes_pars = {
            'nodal_data': np.array([
                [1, 0.0, 0.0, 0.0],
                [2, _lb1, 0.0, 0.0],
                [3, _lb2, 0.0, 0.0],
                [4, _lb1, 0.0, _lc1 - _hs1],
                [5, _lb1 + _ls1, 0.0, _lc1 - _hs1],
                [6, 0.0, 0.0, _lc1],
                [7, _lb1 - _hs2, 0.0, _lc1],
                [8, _lb1, 0.0, _lc1],
                [9, _lb1 - _hs2, 0.0, _lc1 + _ls2],
                [10, _lb2 - _lb3, 0.0, _lc1 + _ls2],
                [11, _lb2, 0.0, _lc1 + _ls2],
                [12, 0.0, 0.0, _lc2],
                [13, _lb2, 0.0, _lc2]
            ])
        }

        self.nodes = nodes(nodes_pars)

    # Step 4: create the Finite Element Method elements
    def _setup_elements(self):
        self._l.debug("Setting up elements.")
        # parameters of the element 1
        beam3d_1_pars = {}
        beam3d_1_pars['shape'] = 'generic'
        beam3d_1_pars['A'] = _A1
        beam3d_1_pars['Ixx'] = _Ixx1
        beam3d_1_pars['Iyy'] = _Iyy1
        beam3d_1_pars['E'] = _Es
        beam3d_1_pars['Jv'] = beam3d_1_pars['Ixx'] + beam3d_1_pars['Iyy']
        beam3d_1_pars['nodal_labels'] = [1, 2]

        # parameters of the element 2
        beam3d_2_pars = {}
        beam3d_2_pars['shape'] = 'generic'
        beam3d_2_pars['A'] = _A1
        beam3d_2_pars['Ixx'] = _Ixx1
        beam3d_2_pars['Iyy'] = _Iyy1
        beam3d_2_pars['E'] = _Es
        beam3d_2_pars['Jv'] = beam3d_2_pars['Ixx'] + beam3d_2_pars['Iyy']
        beam3d_2_pars['nodal_labels'] = [2, 3]

        # parameters of the element 3
        beam3d_3_pars = {}
        beam3d_3_pars['shape'] = 'generic'
        beam3d_3_pars['A'] = _A1
        beam3d_3_pars['Ixx'] = _Ixx1
        beam3d_3_pars['Iyy'] = _Iyy1
        beam3d_3_pars['E'] = _Es
        beam3d_3_pars['Jv'] = beam3d_3_pars['Ixx'] + beam3d_3_pars['Iyy']
        beam3d_3_pars['nodal_labels'] = [1, 6]

        # parameters of the element 5
        beam3d_4_pars = {}
        beam3d_4_pars['shape'] = 'generic'
        beam3d_4_pars['A'] = _A1
        beam3d_4_pars['Ixx'] = _Ixx1
        beam3d_4_pars['Iyy'] = _Iyy1
        beam3d_4_pars['E'] = _Es
        beam3d_4_pars['Jv'] = beam3d_4_pars['Ixx'] + beam3d_4_pars['Iyy']
        beam3d_4_pars['nodal_labels'] = [6, 12]

        # parameters of the element 6
        beam3d_5_pars = {}
        beam3d_5_pars['shape'] = 'generic'
        beam3d_5_pars['A'] = _A1
        beam3d_5_pars['Ixx'] = _Ixx1
        beam3d_5_pars['Iyy'] = _Iyy1
        beam3d_5_pars['E'] = _Es
        beam3d_5_pars['Jv'] = beam3d_5_pars['Ixx'] + beam3d_5_pars['Iyy']
        beam3d_5_pars['nodal_labels'] = [2, 4]

        # parameters of the element 7
        beam3d_6_pars = {}
        beam3d_6_pars['shape'] = 'generic'
        beam3d_6_pars['A'] = _A1
        beam3d_6_pars['Ixx'] = _Ixx1
        beam3d_6_pars['Iyy'] = _Iyy1
        beam3d_6_pars['E'] = _Es
        beam3d_6_pars['Jv'] = beam3d_6_pars['Ixx'] + beam3d_6_pars['Iyy']
        beam3d_6_pars['nodal_labels'] = [4, 8]

        # parameters of the element 8
        beam3d_7_pars = {}
        beam3d_7_pars['shape'] = 'generic'
        beam3d_7_pars['A'] = _A1
        beam3d_7_pars['Ixx'] = _Ixx1
        beam3d_7_pars['Iyy'] = _Iyy1
        beam3d_7_pars['E'] = _Es
        beam3d_7_pars['Jv'] = beam3d_7_pars['Ixx'] + beam3d_7_pars['Iyy']
        beam3d_7_pars['nodal_labels'] = [3, 11]

        # parameters of the element 9
        beam3d_8_pars = {}
        beam3d_8_pars['shape'] = 'generic'
        beam3d_8_pars['A'] = _A1
        beam3d_8_pars['Ixx'] = _Ixx1
        beam3d_8_pars['Iyy'] = _Iyy1
        beam3d_8_pars['E'] = _Es
        beam3d_8_pars['Jv'] = beam3d_8_pars['Ixx'] + beam3d_8_pars['Iyy']
        beam3d_8_pars['nodal_labels'] = [11, 13]

        # parameters of the element 10
        beam3d_9_pars = {}
        beam3d_9_pars['shape'] = 'generic'
        beam3d_9_pars['A'] = _A1
        beam3d_9_pars['Ixx'] = _Ixx1
        beam3d_9_pars['Iyy'] = _Iyy1
        beam3d_9_pars['E'] = _Es
        beam3d_9_pars['Jv'] = beam3d_9_pars['Ixx'] + beam3d_9_pars['Iyy']
        beam3d_9_pars['nodal_labels'] = [6, 7]

        # parameters of the element 11
        beam3d_10_pars = {}
        beam3d_10_pars['shape'] = 'generic'
        beam3d_10_pars['A'] = _A1
        beam3d_10_pars['Ixx'] = _Ixx1
        beam3d_10_pars['Iyy'] = _Iyy1
        beam3d_10_pars['E'] = _Es
        beam3d_10_pars['Jv'] = beam3d_10_pars['Ixx'] + beam3d_10_pars['Iyy']
        beam3d_10_pars['nodal_labels'] = [7, 8]

        # parameters of the element 12
        beam3d_11_pars = {}
        beam3d_11_pars['shape'] = 'generic'
        beam3d_11_pars['A'] = _A1
        beam3d_11_pars['Ixx'] = _Ixx1
        beam3d_11_pars['Iyy'] = _Iyy1
        beam3d_11_pars['E'] = _Es
        beam3d_11_pars['Jv'] = beam3d_11_pars['Ixx'] + beam3d_11_pars['Iyy']
        beam3d_11_pars['nodal_labels'] = [12, 13]

        # parameters of the element 13
        beam3d_12_pars = {}
        beam3d_12_pars['shape'] = 'generic'
        beam3d_12_pars['A'] = _A2
        beam3d_12_pars['Ixx'] = _Ixx2
        beam3d_12_pars['Iyy'] = _Iyy2
        beam3d_12_pars['E'] = _Es
        beam3d_12_pars['Jv'] = beam3d_12_pars['Ixx'] + beam3d_12_pars['Iyy']
        beam3d_12_pars['nodal_labels'] = [7, 9]

        # parameters of the element 14
        beam3d_13_pars = {}
        beam3d_13_pars['shape'] = 'generic'
        beam3d_13_pars['A'] = _A2
        beam3d_13_pars['Ixx'] = _Ixx2
        beam3d_13_pars['Iyy'] = _Iyy2
        beam3d_13_pars['E'] = _Es
        beam3d_13_pars['Jv'] = beam3d_13_pars['Ixx'] + beam3d_13_pars['Iyy']
        beam3d_13_pars['nodal_labels'] = [9, 8]

        # parameters of the element 15
        beam3d_14_pars = {}
        beam3d_14_pars['shape'] = 'generic'
        beam3d_14_pars['A'] = _A2
        beam3d_14_pars['Ixx'] = _Ixx2
        beam3d_14_pars['Iyy'] = _Iyy2
        beam3d_14_pars['E'] = _Es
        beam3d_14_pars['Jv'] = beam3d_14_pars['Ixx'] + beam3d_14_pars['Iyy']
        beam3d_14_pars['nodal_labels'] = [4, 5]

        # parameters of the element 16
        beam3d_15_pars = {}
        beam3d_15_pars['shape'] = 'generic'
        beam3d_15_pars['A'] = _A2
        beam3d_15_pars['Ixx'] = _Ixx2
        beam3d_15_pars['Iyy'] = _Iyy2
        beam3d_15_pars['E'] = _Es
        beam3d_15_pars['Jv'] = beam3d_15_pars['Ixx'] + beam3d_15_pars['Iyy']
        beam3d_15_pars['nodal_labels'] = [5, 8]

        # parameters of the element 17 - this is the beam - we need to plot the force and displacement on this one
        beam3d_16_pars = {}
        beam3d_16_pars['shape'] = 'generic'
        beam3d_16_pars['A'] = _A3
        beam3d_16_pars['Ixx'] = _Ixx3
        beam3d_16_pars['Iyy'] = _Iyy3
        beam3d_16_pars['E'] = _Ea
        beam3d_16_pars['Jv'] = beam3d_16_pars['Ixx'] + beam3d_16_pars['Iyy']
        beam3d_16_pars['nodal_labels'] = [10, 11]

        # add one beam3d element to the list
        self.elements.append(beam3d(self.nodes, beam3d_1_pars))
        self.elements.append(beam3d(self.nodes, beam3d_2_pars))
        self.elements.append(beam3d(self.nodes, beam3d_3_pars))
        self.elements.append(beam3d(self.nodes, beam3d_4_pars))
        self.elements.append(beam3d(self.nodes, beam3d_5_pars))
        self.elements.append(beam3d(self.nodes, beam3d_6_pars))
        self.elements.append(beam3d(self.nodes, beam3d_7_pars))
        self.elements.append(beam3d(self.nodes, beam3d_8_pars))
        self.elements.append(beam3d(self.nodes, beam3d_9_pars))
        self.elements.append(beam3d(self.nodes, beam3d_10_pars))
        self.elements.append(beam3d(self.nodes, beam3d_11_pars))
        self.elements.append(beam3d(self.nodes, beam3d_12_pars))
        self.elements.append(beam3d(self.nodes, beam3d_13_pars))
        self.elements.append(beam3d(self.nodes, beam3d_14_pars))
        self.elements.append(beam3d(self.nodes, beam3d_15_pars))
        self.elements.append(beam3d(self.nodes, beam3d_16_pars))

    # Step 5: Create the Finite Element Method model
    def _setup_model(self):
        #self._l.debug("Setting up model.")
        assert self.nodes is not None, "Nodes are not initialized."
        assert self.elements, "Elements are not initialized."

        # Model parameters
        # x - 1 (right)
        # y - 2 (inside)
        # z - 3 (up)
        # rotation around x axis - >4

        # dofs_c - Constrained degrees of freedom
        # dofs_f - Degrees of freedom subjected to force history
        # dofs_u - Degrees of freedom subjected to displacement history (vertical dofs of node 2)
        # g_f - Force history (21 steps) - Ensure g_f is a 2D array with dimensions (2, 21)
        # g_u - Displacement history (21 steps) - Ensure  is a 2D array with dimensions (2, 21)
        if self._c == [[]]:
            self._l.error("Constraints are not set.")
            raise ValueError("Constraints are not set.")
        else:
            self.model_pars['dofs_c'] = np.array(self._c) 
        
        if self._fn == [[]]:
            self._l.info("Forces are not set.")
            #raise ValueError("Forces are not set.")
        else:
            self.model_pars['dofs_f'] = np.array(self._fn)
            self.model_pars['g_f'] = np.array(self._fs)
        
        if self._un == [[]]:
            self._l.info("Displacements are not set.")
            #raise ValueError("Displacements are not set.")
        else:
            self.model_pars['dofs_u'] = np.array(self._un)
            self.model_pars['g_u'] = np.array(self._us)

        # Create the model
        self.model = model(self.nodes, self.elements, self.model_pars)

    def get_beampars(self, element):
        #self._l.debug("Getting beam parameters. Beam(%s)", element)
        # Get the beam parameters for the model
        # beampars - beam parameters [mm]
        # E - Young's modulus [N/mm2]
        # A - cross-section area [mm2]
        # Ixx - moment of inertia about x-axis [mm4]
        # Iyy - moment of inertia about y-axis [mm4]
        # Jv - polar moment of inertia [mm4]
        return self.elements[element-1]


    def set_beampars(self, element, beampars, values):
        self._l.debug("Setting beam parameters. Beam(%s): %s = %s", element, beampars, values)
        # Set the beam parameters for the model
        # beampars - beam parameters [mm]
        beam = self.elements[element-1]

         # parameters of the element
        beam3d_pars = {}
        beam3d_pars['shape'] = 'generic'
        beam3d_pars['A'] = beam.A
        beam3d_pars['Ixx'] = beam.Ixx
        beam3d_pars['Iyy'] = beam.Iyy
        beam3d_pars['E'] = beam.E
        beam3d_pars['Jv'] = beam.Jv
        beam3d_pars['nodal_labels'] = beam.nodal_labels
        if not isinstance(beampars, list):
            beampars = [beampars]
            values = [values]

        if len(beampars) == len(values):
            for idx, par in enumerate(beampars):
                #self._l.debug("Setting beam parameter. %s, Values(%s) = %s", par, idx, values[idx]) 
                match par:
                    # Set the beam parameters for the model
                    case 'A':
                        beam3d_pars['A'] = (values[idx])
                    case 'Ixx':
                        beam3d_pars['Ixx'] = (values[idx])
                    case 'Iyy':
                        beam3d_pars['Iyy'] = (values[idx])
                    case 'E':
                        beam3d_pars['E'] = (values[idx])
                    case 'Jv':
                        beam3d_pars['Jv'] = (values[idx])
                    case _:
                        self._l.error("Beam parameters not set. %s", par)
                        raise ValueError("Beam parameters not set. %s" % par)
                    
            self._l.debug("Beam parameters set. %s", beam3d_pars)
            self.elements[element-1] = (beam3d(self.nodes, beam3d_pars))

        else:
            self._l.error("Beam parameters and values shape mismatch. Beam parameters shape: %s, Values shape: %s", np.shape(beampars), np.shape(values))
            raise ValueError("Beam parameters and values shape mismatch. Beam parameters shape: %s, Values shape: %s" % (np.shape(beampars), np.shape(values)))


    def clear_constraints(self):
        self._l.debug("Clearing constraints.")
        # Clear the constraints for the model
        # Set the constraints for the model
        # t - time [s]
        # u - displacement [mm]
        mp = model.extract_pars()
        self._l.debug("Extracting model parameters. %s", mp)
        self.model.dofs_c = 0
        self._c = np.zeros((self.nodes.n_nodes, 3))

    def set_constraints(self, t, nodes, direction):
        self._l.debug("Setting constraints. t: %s, nodes: %s, direction: %s", t, nodes, direction)
        # Set the constraints for the model
        # t - time [s]
        # u - displacement [mm]
        i, n = np.shape(nodes)
        for _i in range(i):
            self._c[nodes, direction] = 0.0

        #self._setup_model()

    def get_constraints(self, nodes, direction):
        self._l.debug("Getting constraints. nodes: %s, direction: %s", nodes, direction)
        # Get the constraints for the model
        # t - time [s]
        # u - displacement [mm]
        return self._c[nodes, direction]
    
    def calculate_fatigue(self, cycles):
        self._l.debug("Calculating fatigue...")
        D = 0.0
        # Calculate the fatigue for the model
        # cycles - [stress, cycles]
        for cycle in cycles:
            #self._l.debug("Cycle: %s", cycle)
            # Calculate the fatigue for the model
            # stress - [N/mm2]
            # cycles - [cycles]
            # fatigue - [N/mm2]
            gamma_Mf = 1.0
            N_Ed = cycle[1]
            N_L = 10e8 #number of stress cycles associated with the variable amplitude fatigue limit - EUC3-1-9 Figure 8.1a
            N_C = 2*10e6 #number of stress cycles associated with the characteristic reference value of fatigue resistance - EUC3-1-9 Figure 8.1a
            N_D = N_C #number of cycles associated with the characteristic constant amplitude fatigue limit - EUC3-1-9 Figure 8.1a
            delta_sigma_D = 125 #constant amplitude fatigue limit [N/mm2] - EUC3-1-9 Table A.1
            delta_sigma_C = delta_sigma_D #reference value at 2*10e6 cycles [N/mm2] - EUC3-1-9 Table A.1
            delta_sigma_L = 0.647 * delta_sigma_C #variable amplitude fatigue limit [N/mm2] - EUC3-1-9 Figure 8.1a
            delta_sigma_Ed = cycle[0] * _lb3 * _h3/2/_Ixx3  #M = F * a, sigma = M/W, W = I/y, [N/mm2] 
            m1 = 5 #first slope parameter of the fatigue resistance curve - EUC3-1-9 Figure 8.1a
            m2 = 9 #slope parameter of the extende fatigue resistance curve - EUC3-1-9 Figure 8.1a
            if delta_sigma_Ed >= delta_sigma_D/gamma_Mf:
                N_Rd = 2*10e6*(delta_sigma_C*gamma_Mf/delta_sigma_Ed)**m1
            elif delta_sigma_L/gamma_Mf <= delta_sigma_D <= delta_sigma_D/gamma_Mf:
                N_Rd = N_D*(delta_sigma_D*gamma_Mf/delta_sigma_Ed)**m1
            else:
                N_Rd = 0
            D = D + N_Ed/N_Rd #Damage - EUROCODE 3-1-9 (A.6)
        E = self.get_beampars(16).E * (1-D)
        E = 70e3 * (1-D) # Young's modulus [N/mm2]
        self.set_beampars(16, 'E', E)
        self._l.debug("Fatigue damage: %s, E-Module %s", D, E)
        return [D,E]
    
    def clear_displacement(self, node, direction):
        #Clear the load for the model
        self._l.debug("Clearing displacement. node: %s, direction: %s", node, direction)
        if direction == 0:
            for d in range(3):
                U_idx = np.where((node == np.array(self._un)[:, 0]) & (d+1 == np.array(self._un)[:, 1]))[0] 

                self._u = np.delete(self._u, U_idx[0])
                self._un = np.delete(self._un, U_idx[0])
                self._us = np.delete(self._us, U_idx[0])
        else:
            U_idx = np.where((node == np.array(self._un)[:, 0]) & (direction == np.array(self._un)[:, 1]))[0] 
            self._u = np.delete(self._u, U_idx[0])
            self._un = np.delete(self._un, U_idx[0])
            self._us = np.delete(self._us, U_idx[0])

    def clear_displacements(self):
        self._l.debug("Clearing displacements.")
        # Clear the displacements for the model
        
        self._u = []
        self._un = [[]]
        self._us = [[]]

        #self._setup_model()

    def set_displacements(self, u, nodes, direction):
        #self._l.debug("Setting displacements. t: %s, u: %s", t, u)
        i = np.shape(nodes)[0]

        # Set the displacements for the model
        
        if np.shape(u) == np.shape(nodes) == np.shape(direction):
            #self._l.debug("Setting displacements. t: %s, l: %s, node: %s, direction: %s", t, u, nodes, direction)
            for _i in range(i):
                node = [nodes[_i], direction[_i]]
                if np.array_equal(self._un, [[0, 0]]):
                    self._un[0] = node
                U_idx = np.where((node[0] == np.array(self._un)[:, 0]) & (node[1] == np.array(self._un)[:, 1]))[0] 
                #self._l.debug("Finding idx. %s, %s", U_idx, len([U_idx]))

                if len(U_idx) == 0:
                    self._u.append(u[_i])
                    self._un.append(node)
                    self._us.append([0, u[_i]])
                else:
                    self._u[U_idx[0]] = u[_i]
                    self._us[U_idx[0]] = [0, self._u[U_idx[0]]]
        else:
            self._l.error("Displacement, node and direction shape mismatch. Displacement shape: %s, Node shape: %s, Direction shape: %s", np.shape(u), np.shape(nodes), np.shape(direction))
            raise ValueError("Displacement, node and direction shape mismatch. Displacement shape: %s, Node shape: %s, Direction shape: %s" % (np.shape(u), np.shape(nodes), np.shape(direction)))

        #self._setup_model()

    def set_displacements_between_nodes(self, U, nodes):
        self._l.debug("Setting displacements between nodes. u: %s, nodes: %s", U, nodes)
        self.run_simulation()  # Ensure the model is up to date before setting displacements
        # Set the displacements for the model
        # t - time [s]
        # u - displacement [mm]

        if len(np.shape(nodes)) == 1:
            U = [U]
            nodes = [nodes]

        i, n = np.shape(nodes)

        if np.shape(U)[0] == np.shape(nodes)[0] and n == 2:
        
            for _i in range(i):
                node = nodes[_i]
                node1 = node[0]
                node2 = node[1]
                BTW_idx = np.where((node1 == np.array(self.BTW)[:, 0]) & (node2 == np.array(self.BTW)[:, 1]))[0]
                if len(BTW_idx) == 0:
                    F = 1 # default force [N]
                else:
                    F = self.BTW_f[BTW_idx[0]] # force [N]
                    L0, L1, delta_l = self.get_displacement_between_nodes(node1, node2) # length [mm]
                    F = 1 if delta_l == 0 else np.multiply(F, np.divide(U,delta_l)) # scale force [N]
                    if isnan(F):
                        self._l.info("Force is NaN. %s", F)
                        F = 1 # default force [N]
                    #self._l.debug("Force. %s", F)
                try:
                    self.set_loads_between_nodes(F, nodes[_i])
                except Exception as e:
                    self._l.error("Error setting loads between nodes: %s", e)
                    raise

        else:
            self._l.error("Displacement and node shape mismatch. Displacement shape: %s, Node shape: %s", np.shape(U), np.shape(nodes))
            raise ValueError("Displacement and node shape mismatch. Displacement shape: %s, Node shape: %s" % (np.shape(U), np.shape(nodes)))
        
        #self._l.debug("Displacement between nodes. %s", nodes)

    def get_displacement(self, nodes, direction):
        #self._l.debug("Getting displacements. nodes: %s, direction: %s", nodes, direction)
        # Get the displacements for the model
        
        if isinstance(nodes, int):
            nodes = [nodes]
            direction = [direction]

        us = []
        i = np.shape(nodes)[0]

        if np.shape(nodes) == np.shape(direction):
            for _i in range(i):
                node = nodes[_i]
                d = direction[_i]
                dof = self.model.find_dofs([[node, d]]).squeeze()
                #self._l.debug("Finding dof. %s, %s", dof, self.u[dof,1])
                us.append(self.u[dof, 1]) # local displacement [mm]
        else:
            self._l.error("Displacement and node shape mismatch. Displacement shape: %s, Node shape: %s", np.shape(nodes), np.shape(direction))
            raise ValueError("Displacement and node shape mismatch. Displacement shape: %s, Node shape: %s" % (np.shape(nodes), np.shape(direction)))
        
        self._l.debug("Displacement: %s", us)
        return us
    
    def get_displacement_between_nodes(self, node1, node2):
        #self._l.debug("Getting displacements between nodes. nodes: %s & %s", node1, node2)
        # Get the displacements for the model
        
        ulok = [0,0,0]

        xyz1 = self.model.my_nodes.nodal_coords[node1-1]
        xyz2 = self.model.my_nodes.nodal_coords[node2-1]
        L0 = sqrt((xyz1[0] - xyz2[0])**2 + (xyz1[1] - xyz2[1])**2 + (xyz1[2] - xyz2[2])**2) # length [mm]
        for d in range(3):
            dof1 = self.model.find_dofs([[node1, d+1]]).squeeze()
            dof2 = self.model.find_dofs([[node2, d+1]]).squeeze()
            ulok[d] = self.u[dof1, 1] - self.u[dof2, 1] # local displacement [mm]
        L1 = sqrt((xyz1[0] - xyz2[0] + ulok[0])**2 + (xyz1[1] - xyz2[1] + ulok[1])**2 + (xyz1[2] - xyz2[2] + ulok[2])**2) # length [mm]
        delta_l = L1 - L0 # deltaL [mm]
        
        self._l.debug("L0: %s, L1: %s, DeltaL: %s", L0, L1, delta_l)
        return L0, L1, delta_l

    
    def get_displacements(self):
        # Get the displacements for the model
        self._l.debug("Getting all displacements.")
        return self.u[:, 1]
    
    def clear_load(self, node, direction):
        #Clear the load for the model
        self._l.debug("Clearing load. node: %s, direction: %s", node, direction)
        if direction == 0:
            for d in range(3):
                F_idx = np.where((node == np.array(self._fn)[:, 0]) & (d+1 == np.array(self._fn)[:, 1]))[0] 

                self._f = np.delete(self._f, F_idx[0])
                self._fn = np.delete(self._fn, F_idx[0])
                self._fs = np.delete(self._fs, F_idx[0])
        else:
            F_idx = np.where((node == np.array(self._fn)[:, 0]) & (direction == np.array(self._fn)[:, 1]))[0] 
            self._f = np.delete(self._f, F_idx[0])
            self._fn = np.delete(self._fn, F_idx[0])
            self._fs = np.delete(self._fs, F_idx[0])
    
    def clear_loads(self):
        # Clear the loads for the model
        self._l.debug("Clearing loads.")
        
        self._f = []
        self._fn = [[]]
        self._fs = [[]]

        #self._setup_model()

    def set_loads(self, f, nodes, direction):
        #self._l.debug("Setting loads. t: %s, f: %s, node: %s, direction: %s", t, f, nodes, direction)
        i = np.shape(nodes)[0]

        F_idx = []

        # Set the loads for the model
        
        if np.shape(f) == np.shape(nodes) == np.shape(direction):
            for _i in range(i):
                if not f[_i] == 0:
                    #self._l.debug("Setting loads. %s, %s, %s", f[_i], nodes[_i], direction[_i])
                    node = [nodes[_i], direction[_i]]
                    if self._fn == [[]]:
                        #self._l.debug("Setting first load. %s = %s", node, self._fn)
                        self._f.append(f[_i])
                        self._fn[0] = node
                        self._fs[0] = [0, f[_i]]
                    #else:
                        #self._l.debug("Finding idx. Node[0]: %s = %s & Node[1]: %s = %s", node[0], self._fn, node[1], self._fs)
                    F_idx = np.where((node[0] == np.array(self._fn)[:, 0]) & (node[1] == np.array(self._fn)[:, 1]))[0] 
                    #self._l.debug("Finding idx. %s, %s", F_idx, len([F_idx]))

                    if len(F_idx) == 0:
                        self._f.append(f[_i])
                        self._fn.append(node)
                        self._fs.append([0, f[_i]])
                        #self._l.debug("Setting new load. %s = %s", len(self._f), f[_i])
                        #self._l.debug("Existing load [f]. %s - %s", np.shape(self._f), self._f)
                        #self._l.debug("Existing load [fn]. %s - %s", np.shape(self._fn), self._fn)
                        #self._l.debug("Existing load [fs]. %s - %s", np.shape(self._fn), self._fs)
                    else:
                        self._f[F_idx[0]] = f[_i]
                        self._fs[F_idx[0]] = [0, self._f[F_idx[0]]]
                        #self._l.debug("Setting existing load. %s = %s", F_idx[0], self._f[_i])
                        #self._l.debug("Existing load [f]. %s - %s", np.shape(self._f), self._f)
                        #self._l.debug("Existing load [fn]. %s - %s", np.shape(self._fn), self._fn)
                        #self._l.debug("Existing load [fs]. %s - %s", np.shape(self._fs), self._fs)
                #else:
                    #self._l.debug("Skipping load. %s, %s, %s", f[_i], nodes[_i], direction[_i])

                
        else:
            self._l.error("Load, node and direction shape mismatch. Load shape: %s, Node shape: %s, Direction shape: %s", np.shape(f), np.shape(nodes), np.shape(direction))
            raise ValueError("Load, node and direction shape mismatch. Load shape: %s, Node shape: %s, Direction shape: %s" % (np.shape(f), np.shape(nodes), np.shape(direction)))

        #self._setup_model()

    def set_loads_between_nodes(self, F, nodes):
        #self._l.debug("Setting loads between nodes. t: %s, F: %s, node: %s", t, F, nodes)
        # Set the loads for the model
        # t - time [s]
        # F - force [N]
        # nodes - nodes [1, 2] - node 1 and node 2

        if not hasattr(self, 'BTW'):
            # Initialize the lists if they don't exist
            self.BTW = []
            self.BTW_f = []
            BTW_idx = []

        if isinstance(F, list):
            # If F is a list, convert it to a numpy array
            self._l.debug("F is a list. %s", F)

        else:
            # If F is an int, convert it to a numpy array
            #self._l.debug("F is not a list. %s", F)
            F = [F]
            nodes = [nodes]


        i, n = np.shape(nodes)
        llok = [0,0,0]
        flok = [0,0,0]

        #Set the loads for the model...

        if np.shape(F)[0] == np.shape(nodes)[0] and n == 2:
        
            for _i in range(i):
                node = nodes[_i]
                # Check if the node is already in the list of nodes
                if not len(self.BTW) == 0: 
                    node1 = node[0]
                    node2 = node[1]
                    BTW_idx = np.where((node1 == np.array(self.BTW)[:, 0]) & (node2 == np.array(self.BTW)[:, 1]))[0]
                if len(BTW_idx) == 0:
                    self.BTW.append(node)
                    self.BTW_f.append(F[_i])
                else:
                    self.BTW[BTW_idx[0]] = node
                    self.BTW_f[BTW_idx[0]] = F[_i]

                xyz1 = self.model.my_nodes.nodal_coords[node[0]-1]
                xyz2 = self.model.my_nodes.nodal_coords[node[1]-1]
                
                for d in range(3):
                    try:
                        dof1 = self.model.find_dofs([[node[0], 1 + d]]).squeeze()
                        dof2 = self.model.find_dofs([[node[1], 1 + d]]).squeeze()
                        
                    except Exception as e:
                        self._l.error("Error finding dof: %s", e)
                        raise
                    llok[d] = (xyz1[d] + self.u[dof1,0]) - (xyz2[d] + self.u[dof2,0]) # deltaL [mm]
                    
                l_f = sqrt(llok[0]**2 + llok[1]**2 + llok[2]**2) # displacement [mm]

                for d in range(3):
                    # Set Loads for the model
                    flok[d] = float(np.multiply(F, np.divide(llok[d] , l_f))) # load [N]
                    self.set_loads([flok[d],-flok[d]], nodes[_i], [d+1,d+1])
                    
        else:
            self._l.error("Loads and node shape mismatch. Load shape: %s, Node shape: %s", np.shape(F), np.shape(nodes))
            raise ValueError("Loads and node shape mismatch. Load shape: %s, Node shape: %s" % (np.shape(F), np.shape(nodes)))

        #self._l.debug("Loads between nodes: %s", self.BTW)
        #self._setup_model()

    def get_load(self, nodes, direction):
        self._l.debug("Getting loads. nodes: %s, direction: %s", nodes, direction)
        # Get the load for the model]
        if isinstance(nodes, int):
            nodes = [nodes]
            direction = [direction]
        fs = []
        i = np.shape(nodes)[0]

        if np.shape(nodes) == np.shape(direction):
            for _i in range(i):
                node = [nodes[_i], direction[_i]]
                try:
                    F_idx = np.where((node[0] == np.array(self._fn)[:, 0]) & (node[1] == np.array(self._fn)[:, 1]))[0] 
                    if self._f[F_idx[0]] is None:
                        #self._l.error("Load is not set.")
                        fs.append(0.0)
                        #raise ValueError("Load is not set.")
                    else:
                        #self._l.debug("Load is set. %s", self._f[nodes[_i], direction[_i]])
                        fs.append(self._f[F_idx[0]])
                except Exception as e:
                    self._l.error("Error finding load index: %s", e)
                    fs = [0]
        else:
            self._l.error("Load and node shape mismatch. Load shape: %s, Node shape: %s", np.shape(nodes), np.shape(direction))
            raise ValueError("Load and node shape mismatch. Load shape: %s, Node shape: %s" % (np.shape(nodes), np.shape(direction)))
        self._l.debug("Loads: %s", fs)
        return fs

    def get_loads(self):
        self._l.debug("Getting loads: %s", self._l)
        return self._f


    # Step 6: create and execute the simulation
    def run_simulation(self):
        #self._l.debug("Running simulation.")

        self._setup_model()
        
        simulation_pars = {}

        sim = simulation(self.model, simulation_pars)
        #self._l.debug("Simulation parameters: %s, %s", self.model, simulation_pars)
        try:
            # [u, v, a, r] = simulation.dynamic_analysis()
            # perform static analysis (u: displacements, l: applied forces, r: restoring force)
            #self._l.debug("Performing static analysis.")

            self.u, self.l, self.r = sim.static_analysis()
            #self._l.debug("Static analysis completed. %s, %s, %s", self.u, self.l, self.r)

        except Exception as e:
            self._l.error("Simulation failed: %s", e)
            raise
        #self._l.debug("Simulation completed.")
        return self.u, self.l, self.r