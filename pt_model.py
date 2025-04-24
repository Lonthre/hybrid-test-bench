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

        
        self._setup_nodes()
        self._setup_elements()
        self.set_loads(10, 200, 20)
        #self._setup_model()
        

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
        beam3d_3_pars['nodal_labels'] = [2, 3]

        # parameters of the element 4
        beam3d_4_pars = {}
        beam3d_4_pars['shape'] = 'generic'
        beam3d_4_pars['A'] = _A1
        beam3d_4_pars['Ixx'] = _Ixx1
        beam3d_4_pars['Iyy'] = _Iyy1
        beam3d_4_pars['E'] = _Es
        beam3d_4_pars['Jv'] = beam3d_4_pars['Ixx'] + beam3d_4_pars['Iyy']
        beam3d_4_pars['nodal_labels'] = [1, 6]

        # parameters of the element 5
        beam3d_5_pars = {}
        beam3d_5_pars['shape'] = 'generic'
        beam3d_5_pars['A'] = _A1
        beam3d_5_pars['Ixx'] = _Ixx1
        beam3d_5_pars['Iyy'] = _Iyy1
        beam3d_5_pars['E'] = _Es
        beam3d_5_pars['Jv'] = beam3d_5_pars['Ixx'] + beam3d_5_pars['Iyy']
        beam3d_5_pars['nodal_labels'] = [6, 12]

        # parameters of the element
        beam3d_6_pars = {}
        beam3d_6_pars['shape'] = 'generic'
        beam3d_6_pars['A'] = _A1
        beam3d_6_pars['Ixx'] = _Ixx1
        beam3d_6_pars['Iyy'] = _Iyy1
        beam3d_6_pars['E'] = _Es
        beam3d_6_pars['Jv'] = beam3d_6_pars['Ixx'] + beam3d_6_pars['Iyy']
        beam3d_6_pars['nodal_labels'] = [2, 4]

        # parameters of the element 7
        beam3d_7_pars = {}
        beam3d_7_pars['shape'] = 'generic'
        beam3d_7_pars['A'] = _A1
        beam3d_7_pars['Ixx'] = _Ixx1
        beam3d_7_pars['Iyy'] = _Iyy1
        beam3d_7_pars['E'] = _Es
        beam3d_7_pars['Jv'] = beam3d_7_pars['Ixx'] + beam3d_7_pars['Iyy']
        beam3d_7_pars['nodal_labels'] = [4, 8]

        # parameters of the element 8
        beam3d_8_pars = {}
        beam3d_8_pars['shape'] = 'generic'
        beam3d_8_pars['A'] = _A1
        beam3d_8_pars['Ixx'] = _Ixx1
        beam3d_8_pars['Iyy'] = _Iyy1
        beam3d_8_pars['E'] = _Es
        beam3d_8_pars['Jv'] = beam3d_8_pars['Ixx'] + beam3d_8_pars['Iyy']
        beam3d_8_pars['nodal_labels'] = [3, 11]

        # parameters of the element 9
        beam3d_9_pars = {}
        beam3d_9_pars['shape'] = 'generic'
        beam3d_9_pars['A'] = _A1
        beam3d_9_pars['Ixx'] = _Ixx1
        beam3d_9_pars['Iyy'] = _Iyy1
        beam3d_9_pars['E'] = _Es
        beam3d_9_pars['Jv'] = beam3d_9_pars['Ixx'] + beam3d_9_pars['Iyy']
        beam3d_9_pars['nodal_labels'] = [11, 13]

        # parameters of the element 10
        beam3d_10_pars = {}
        beam3d_10_pars['shape'] = 'generic'
        beam3d_10_pars['A'] = _A1
        beam3d_10_pars['Ixx'] = _Ixx1
        beam3d_10_pars['Iyy'] = _Iyy1
        beam3d_10_pars['E'] = _Es
        beam3d_10_pars['Jv'] = beam3d_10_pars['Ixx'] + beam3d_10_pars['Iyy']
        beam3d_10_pars['nodal_labels'] = [6, 7]

        # parameters of the element 11
        beam3d_11_pars = {}
        beam3d_11_pars['shape'] = 'generic'
        beam3d_11_pars['A'] = _A1
        beam3d_11_pars['Ixx'] = _Ixx1
        beam3d_11_pars['Iyy'] = _Iyy1
        beam3d_11_pars['E'] = _Es
        beam3d_11_pars['Jv'] = beam3d_11_pars['Ixx'] + beam3d_11_pars['Iyy']
        beam3d_11_pars['nodal_labels'] = [7, 8]

        # parameters of the element 12
        beam3d_12_pars = {}
        beam3d_12_pars['shape'] = 'generic'
        beam3d_12_pars['A'] = _A1
        beam3d_12_pars['Ixx'] = _Ixx1
        beam3d_12_pars['Iyy'] = _Iyy1
        beam3d_12_pars['E'] = _Es
        beam3d_12_pars['Jv'] = beam3d_12_pars['Ixx'] + beam3d_12_pars['Iyy']
        beam3d_12_pars['nodal_labels'] = [12, 13]

        # parameters of the element 13
        beam3d_13_pars = {}
        beam3d_13_pars['shape'] = 'generic'
        beam3d_13_pars['A'] = _A2
        beam3d_13_pars['Ixx'] = _Ixx2
        beam3d_13_pars['Iyy'] = _Iyy2
        beam3d_13_pars['E'] = _Es
        beam3d_13_pars['Jv'] = beam3d_13_pars['Ixx'] + beam3d_13_pars['Iyy']
        beam3d_13_pars['nodal_labels'] = [7, 9]

        # parameters of the element 14
        beam3d_14_pars = {}
        beam3d_14_pars['shape'] = 'generic'
        beam3d_14_pars['A'] = _A2
        beam3d_14_pars['Ixx'] = _Ixx2
        beam3d_14_pars['Iyy'] = _Iyy2
        beam3d_14_pars['E'] = _Es
        beam3d_14_pars['Jv'] = beam3d_14_pars['Ixx'] + beam3d_14_pars['Iyy']
        beam3d_14_pars['nodal_labels'] = [9, 8]

        # parameters of the element 15
        beam3d_15_pars = {}
        beam3d_15_pars['shape'] = 'generic'
        beam3d_15_pars['A'] = _A2
        beam3d_15_pars['Ixx'] = _Ixx2
        beam3d_15_pars['Iyy'] = _Iyy2
        beam3d_15_pars['E'] = _Es
        beam3d_15_pars['Jv'] = beam3d_15_pars['Ixx'] + beam3d_15_pars['Iyy']
        beam3d_15_pars['nodal_labels'] = [4, 5]

        # parameters of the element 16
        beam3d_16_pars = {}
        beam3d_16_pars['shape'] = 'generic'
        beam3d_16_pars['A'] = _A2
        beam3d_16_pars['Ixx'] = _Ixx2
        beam3d_16_pars['Iyy'] = _Iyy2
        beam3d_16_pars['E'] = _Es
        beam3d_16_pars['Jv'] = beam3d_16_pars['Ixx'] + beam3d_16_pars['Iyy']
        beam3d_16_pars['nodal_labels'] = [5, 8]

        # parameters of the element 17 - this is the beam - we need to plot the force and displacement on this one
        beam3d_17_pars = {}
        beam3d_17_pars['shape'] = 'generic'
        beam3d_17_pars['A'] = _A3
        beam3d_17_pars['Ixx'] = _Ixx3
        beam3d_17_pars['Iyy'] = _Iyy3
        beam3d_17_pars['E'] = _Ea
        beam3d_17_pars['Jv'] = beam3d_17_pars['Ixx'] + beam3d_17_pars['Iyy']
        beam3d_17_pars['nodal_labels'] = [10, 11]

        # add one beam3d element to the list
        self.elements.append(beam3d(self.nodes, beam3d_1_pars))
        self.elements.append(beam3d(self.nodes, beam3d_2_pars))
        self.elements.append(beam3d(self.nodes, beam3d_3_pars))
        self.elements.append(beam3d(self.nodes, beam3d_4_pars))
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
        self.elements.append(beam3d(self.nodes, beam3d_17_pars))

    # Step 5: Create the Finite Element Method model
    def _setup_model(self):
        self._l.debug("Setting up model.")
        assert self.nodes is not None, "Nodes are not initialized."
        assert self.elements, "Elements are not initialized."

        # Model parameters
        # x - 1 (right)
        # y - 2 (inside)
        # z - 3 (up)
        # rotation around x axis - >4
        uvs = self.uvs
        fhs = self.fhs

        # dofs_c - Constrained degrees of freedom
        # dofs_f - Degrees of freedom subjected to force history
        # dofs_u - Degrees of freedom subjected to displacement history (vertical dofs of node 2)
        # g_f - Force history (21 steps) - Ensure g_f is a 2D array with dimensions (2, 21)
        # g_u - Displacement history (21 steps) - Ensure  is a 2D array with dimensions (2, 21)
        model_pars = {
            'dofs_c': np.array([[1, 1], [1, 2], [1, 3], [1, 4],
                                [3, 2], [3, 3], [3, 4]]),
            'dofs_f': np.array([[9, 1], [10, 1]]),
            'dofs_u': np.array([[5, 3], [10, 3]]),
            'g_f': np.array([np.multiply(fhs, -1), np.multiply(fhs, 1)]),
            'g_u': np.array([np.multiply(uvs, -0.1), np.multiply(uvs, 0.9)])
        }

        # Create the model
        self.model = model(self.nodes, self.elements, model_pars)



    def set_loads(self, t, lv, lh):
        self._l.debug("Setting loads. lv: %s, lh: %s", lv, lh)
        # Set the loads for the model
        # t - time [s]
        # lv - vertical load [N]
        # lh - horizontal load [N]

        self._t = t # time [s]
        
        self._uv = lv
        # _uv * sin(4/pi * _t) # vertical displacement applied [mm]
        self._fh = lh
        # fh * sin(2/pi * _t) # horizontal force applied [N]

        ts = np.linspace(0, self._t, 21)

        uvs = []
        fhs = []

        VERTICAL_FREQ = 4 / pi
        HORIZONTAL_FREQ = 2 / pi

        for time in ts:
            uvs.append(self._uv * sin(VERTICAL_FREQ * time))
            fhs.append(self._fh * sin(HORIZONTAL_FREQ * time))
        self.uvs = uvs
        self.fhs = fhs

        self._setup_model()


    def get_load(self):
        self._l.debug("Getting load. lv: %s, lh: %s", self._uv, self._fh)
        return self._uv, self._fh

    def get_loads(self):
        self._l.debug("Getting loads. uvs: %s, fhs: %s", self.uvs, self.fhs)
        return self.uvs, self.fhs


    # Step 6: create and execute the simulation
    def run_simulation(self):
        self._l.debug("Running simulation.")
        
        simulation_pars = {}

        sim = simulation(self.model, simulation_pars)
        try:
            # [u, v, a, r] = simulation.dynamic_analysis()
            # perform static analysis (u: displacements, l: applied forces, r: restoring force)
            self._l.debug("Performing static analysis.")

            u, l, r = sim.static_analysis()
        except Exception as e:
            self._l.error("Simulation failed: %s", e)
            raise
        self._l.debug("Simulation completed.")
        return u, l, r