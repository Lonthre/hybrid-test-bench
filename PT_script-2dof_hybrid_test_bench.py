#!/usr/bin/env python
# coding: utf-8

from yafem.nodes import *
from yafem.elem import *
from yafem.model import *
from yafem.simulation import *
from math import *

_lb1 = 917.0 # length of the short beam [mm]
_lb2 = 1786.0 # length of the long beam [mm]
_lb3 = 603.0 # length of the aluminium beam [mm]

_lc1 = 996.0 # length of the short column [mm]
_lc2 = 1262.0 + 380.0 # length of the long column [mm]

_ls1 = 266.0 # length of the support beam [mm]
_ls2 = 266.0 # length of the support beam [mm]
_hs1 = 321.0 # length of the support beam [mm]
_hs2 = 206.5 # length of the support beam [mm]

_Es = 210e3 # Young's modulus (Steel) [N/mm^2]
_Ea = 70e3 # Young's modulus (~Aluminium) [N/mm^2]

# red - steel
# blue - steel
# green - aluminium

# cross-section parameters (frame)
_b1 = 150.0 # cross section width [mm]
_h1 = 100.0 # cross section height [mm]
_s1 = 5.0 # cross section thickness [mm]
_A1 = _b1*_h1 - (_b1-2*_s1) * (_h1-2*_s1) # cross-section area [mm2]
_Ixx1 = _b1*_h1**3/12 - (_b1-2*_s1)*(_h1-2*_s1)**3/12 # moment of inertia about x-axis [mm4]
_Iyy1 = _h1*_b1**3/12 - (_h1-2*_s1)*(_b1-2*_s1)**3/12 # moment of inertia about y-axis [mm4]

# cross-section parameters (supports)
_b2 = 40.0 # cross section width [mm]
_h2 = 60.0 # cross section height [mm]
_s2 = 5.0 # cross section thickness [mm]
_A2 = _b2*_h2 - (_b2-2*_s2) * (_h2-2*_s2) # cross-section area [mm2]
_Ixx2 = _b2*_h2**3/12 - (_b2-2*_s2)*(_h2-2*_s2)**3/12 # moment of inertia about x-axis [mm4]
_Iyy2 = _h2*_b2**3/12 - (_h2-2*_s2)*(_b2-2*_s2)**3/12 # moment of inertia about y-axis [mm4]

# cross-section parameters (aluminium)
_b3 = 80.0 # cross section width [mm]
_h3 = 5.0 # cross section height [mm]
_A3 = _b3*_h3 # cross-section area [mm2]
_Ixx3 = _b3*_h3**3/12 # moment of inertia about x-axis [mm4]
_Iyy3 = _h3*_b3**3/12 # moment of inertia about y-axis [mm4]

# parameters of the applied boundary conditions (displacements and forces)

_t = 10 # We need to define t; for now it's 10. We're not sure what unit it uses, we're assuming that it's in seconds.
   # The problem is that 10 seconds is very little for displacement - it should be in the hundreds/thousands at least.

_uv = 20.0 
# _uv * sin(4/pi * _t) # vertical displacement applied [mm]
_fh = 100.0 
# fh * sin(2/pi * _t) # horizontal force applied [N]

ts = np.linspace(0, _t, 21)

uvs = []
for t in ts: 

   uvs.append(_uv * sin(4/pi * t))

fhs = []
for t in ts: 

   fhs.append(_fh * sin(2/pi * t))


# Step 3: Create the Finite Element Method nodes


# nodal parameters (x, y, z)
my_nodes_pars = {}
my_nodes_pars['nodal_data'] = np.array([
                                        [1,0.0,0.0,0.0],
                                        [2,_lb1,0.0,0.0],
                                        [3,_lb2,0.0,0.0],
                                        [4,_lb1,0.0, _lc1-_hs1],
                                        [5,_lb1+_ls1,0.0,_lc1-_hs1],
                                        [6,0.0,0.0,_lc1],
                                        [7,_lb1-_hs2,0.0,_lc1],
                                        [8,_lb1,0.0,_lc1],
                                        [9,_lb1-_hs2,0.0,_lc1+_ls2],
                                        [10,_lb2-_lb3,0.0,_lc1+_ls2],
                                        [11,_lb2,0.0,_lc1+_ls2],
                                        [12,0.0,0.0,_lc2],
                                        [13,_lb2,0.0,_lc2]])
# create the nodes
my_nodes = nodes(my_nodes_pars)


# Step 4: create the Finite Element Method elements



# parameters of the element 1
my_beam3d_1_pars = {}
my_beam3d_1_pars['shape'] = 'generic'
my_beam3d_1_pars['A'] = _A1
my_beam3d_1_pars['Ixx'] = _Ixx1
my_beam3d_1_pars['Iyy'] = _Iyy1
my_beam3d_1_pars['E'] = _Es
my_beam3d_1_pars['Jv'] = my_beam3d_1_pars['Ixx'] + my_beam3d_1_pars['Iyy']
my_beam3d_1_pars['nodal_labels'] = [1,2]

# parameters of the element 2
my_beam3d_2_pars = {}
my_beam3d_2_pars['shape'] = 'generic'
my_beam3d_2_pars['A'] = _A1
my_beam3d_2_pars['Ixx'] = _Ixx1
my_beam3d_2_pars['Iyy'] = _Iyy1
my_beam3d_2_pars['E'] = _Es
my_beam3d_2_pars['Jv'] = my_beam3d_2_pars['Ixx'] +  my_beam3d_2_pars['Iyy']
my_beam3d_2_pars['nodal_labels'] = [2,3]

# parameters of the element 3
my_beam3d_3_pars = {}
my_beam3d_3_pars['shape'] = 'generic'
my_beam3d_3_pars['A'] = _A1
my_beam3d_3_pars['Ixx'] = _Ixx1
my_beam3d_3_pars['Iyy'] = _Iyy1
my_beam3d_3_pars['E'] = _Es
my_beam3d_3_pars['Jv'] = my_beam3d_3_pars['Ixx'] +  my_beam3d_3_pars['Iyy']
my_beam3d_3_pars['nodal_labels'] = [2,3]

# parameters of the element 4
my_beam3d_4_pars = {}
my_beam3d_4_pars['shape'] = 'generic'
my_beam3d_4_pars['A'] = _A1
my_beam3d_4_pars['Ixx'] = _Ixx1
my_beam3d_4_pars['Iyy'] = _Iyy1
my_beam3d_4_pars['E'] = _Es
my_beam3d_4_pars['Jv'] = my_beam3d_4_pars['Ixx'] +  my_beam3d_4_pars['Iyy']
my_beam3d_4_pars['nodal_labels'] = [1,6]

# parameters of the element 5
my_beam3d_5_pars = {}
my_beam3d_5_pars['shape'] = 'generic'
my_beam3d_5_pars['A'] = _A1
my_beam3d_5_pars['Ixx'] = _Ixx1
my_beam3d_5_pars['Iyy'] = _Iyy1
my_beam3d_5_pars['E'] = _Es
my_beam3d_5_pars['Jv'] = my_beam3d_5_pars['Ixx'] +  my_beam3d_5_pars['Iyy']
my_beam3d_5_pars['nodal_labels'] = [6,12]

# parameters of the element
my_beam3d_6_pars = {}
my_beam3d_6_pars['shape'] = 'generic'
my_beam3d_6_pars['A'] = _A1
my_beam3d_6_pars['Ixx'] = _Ixx1
my_beam3d_6_pars['Iyy'] = _Iyy1
my_beam3d_6_pars['E'] = _Es
my_beam3d_6_pars['Jv'] = my_beam3d_6_pars['Ixx'] +  my_beam3d_6_pars['Iyy']
my_beam3d_6_pars['nodal_labels'] = [2,4]

# parameters of the element 7
my_beam3d_7_pars = {}
my_beam3d_7_pars['shape'] = 'generic'
my_beam3d_7_pars['A'] = _A1
my_beam3d_7_pars['Ixx'] = _Ixx1
my_beam3d_7_pars['Iyy'] = _Iyy1
my_beam3d_7_pars['E'] = _Es
my_beam3d_7_pars['Jv'] = my_beam3d_7_pars['Ixx'] +  my_beam3d_7_pars['Iyy']
my_beam3d_7_pars['nodal_labels'] = [4,8]

# parameters of the element 8
my_beam3d_8_pars = {}
my_beam3d_8_pars['shape'] = 'generic'
my_beam3d_8_pars['A'] = _A1
my_beam3d_8_pars['Ixx'] = _Ixx1
my_beam3d_8_pars['Iyy'] = _Iyy1
my_beam3d_8_pars['E'] = _Es
my_beam3d_8_pars['Jv'] = my_beam3d_8_pars['Ixx'] + my_beam3d_8_pars['Iyy']
my_beam3d_8_pars['nodal_labels'] = [3,11]

# parameters of the element 9
my_beam3d_9_pars = {}
my_beam3d_9_pars['shape'] = 'generic'
my_beam3d_9_pars['A'] = _A1
my_beam3d_9_pars['Ixx'] = _Ixx1
my_beam3d_9_pars['Iyy'] = _Iyy1
my_beam3d_9_pars['E'] = _Es
my_beam3d_9_pars['Jv'] = my_beam3d_9_pars['Ixx'] +  my_beam3d_9_pars['Iyy']
my_beam3d_9_pars['nodal_labels'] = [11,13]

# parameters of the element 10
my_beam3d_10_pars = {}
my_beam3d_10_pars['shape'] = 'generic'
my_beam3d_10_pars['A'] = _A1
my_beam3d_10_pars['Ixx'] = _Ixx1
my_beam3d_10_pars['Iyy'] = _Iyy1
my_beam3d_10_pars['E'] = _Es
my_beam3d_10_pars['Jv'] = my_beam3d_10_pars['Ixx'] +  my_beam3d_10_pars['Iyy']
my_beam3d_10_pars['nodal_labels'] = [6,7]

# parameters of the element 11
my_beam3d_11_pars = {}
my_beam3d_11_pars['shape'] = 'generic'
my_beam3d_11_pars['A'] = _A1
my_beam3d_11_pars['Ixx'] = _Ixx1
my_beam3d_11_pars['Iyy'] = _Iyy1
my_beam3d_11_pars['E'] = _Es
my_beam3d_11_pars['Jv'] = my_beam3d_11_pars['Ixx'] + my_beam3d_11_pars['Iyy']
my_beam3d_11_pars['nodal_labels'] = [7,8]

# parameters of the element 12
my_beam3d_12_pars = {}
my_beam3d_12_pars['shape'] = 'generic'
my_beam3d_12_pars['A'] = _A1
my_beam3d_12_pars['Ixx'] = _Ixx1
my_beam3d_12_pars['Iyy'] = _Iyy1
my_beam3d_12_pars['E'] = _Es
my_beam3d_12_pars['Jv'] = my_beam3d_12_pars['Ixx'] + my_beam3d_12_pars['Iyy']
my_beam3d_12_pars['nodal_labels'] = [12,13]

# parameters of the element 13
my_beam3d_13_pars = {}
my_beam3d_13_pars['shape'] = 'generic'
my_beam3d_13_pars['A'] = _A2
my_beam3d_13_pars['Ixx'] = _Ixx2
my_beam3d_13_pars['Iyy'] = _Iyy2
my_beam3d_13_pars['E'] = _Es
my_beam3d_13_pars['Jv'] = my_beam3d_13_pars['Ixx'] + my_beam3d_13_pars['Iyy']
my_beam3d_13_pars['nodal_labels'] = [7,9]

# parameters of the element 14
my_beam3d_14_pars = {}
my_beam3d_14_pars['shape'] = 'generic'
my_beam3d_14_pars['A'] = _A2
my_beam3d_14_pars['Ixx'] = _Ixx2
my_beam3d_14_pars['Iyy'] = _Iyy2
my_beam3d_14_pars['E'] = _Es
my_beam3d_14_pars['Jv'] = my_beam3d_14_pars['Ixx'] + my_beam3d_14_pars['Iyy']
my_beam3d_14_pars['nodal_labels'] = [9,8]

# parameters of the element 15
my_beam3d_15_pars = {}
my_beam3d_15_pars['shape'] = 'generic'
my_beam3d_15_pars['A'] = _A2
my_beam3d_15_pars['Ixx'] = _Ixx2
my_beam3d_15_pars['Iyy'] = _Iyy2
my_beam3d_15_pars['E'] = _Es
my_beam3d_15_pars['Jv'] = my_beam3d_15_pars['Ixx'] + my_beam3d_15_pars['Iyy']
my_beam3d_15_pars['nodal_labels'] = [4,5]

# parameters of the element 16
my_beam3d_16_pars = {}
my_beam3d_16_pars['shape'] = 'generic'
my_beam3d_16_pars['A'] = _A2
my_beam3d_16_pars['Ixx'] = _Ixx2
my_beam3d_16_pars['Iyy'] = _Iyy2
my_beam3d_16_pars['E'] = _Es
my_beam3d_16_pars['Jv'] = my_beam3d_16_pars['Ixx'] + my_beam3d_16_pars['Iyy']
my_beam3d_16_pars['nodal_labels'] = [5,8]

# parameters of the element 17 - this is the beam - we need to plot the force and displacement on this one
my_beam3d_17_pars = {}
my_beam3d_17_pars['shape'] = 'generic'
my_beam3d_17_pars['A'] = _A3
my_beam3d_17_pars['Ixx'] = _Ixx3
my_beam3d_17_pars['Iyy'] = _Iyy3
my_beam3d_17_pars['E'] = _Ea
my_beam3d_17_pars['Jv'] = my_beam3d_17_pars['Ixx'] + my_beam3d_17_pars['Iyy']
my_beam3d_17_pars['nodal_labels'] = [10,11]

# initialization of the element list
my_elements = []

# add one beam3d element to the list
my_elements.append(beam3d(my_nodes,my_beam3d_1_pars))
my_elements.append(beam3d(my_nodes,my_beam3d_2_pars))
my_elements.append(beam3d(my_nodes,my_beam3d_3_pars))
my_elements.append(beam3d(my_nodes,my_beam3d_4_pars))
my_elements.append(beam3d(my_nodes,my_beam3d_5_pars))
my_elements.append(beam3d(my_nodes,my_beam3d_6_pars))
my_elements.append(beam3d(my_nodes,my_beam3d_7_pars))
my_elements.append(beam3d(my_nodes,my_beam3d_8_pars))
my_elements.append(beam3d(my_nodes,my_beam3d_9_pars))
my_elements.append(beam3d(my_nodes,my_beam3d_10_pars))
my_elements.append(beam3d(my_nodes,my_beam3d_11_pars))
my_elements.append(beam3d(my_nodes,my_beam3d_12_pars))
my_elements.append(beam3d(my_nodes,my_beam3d_13_pars))
my_elements.append(beam3d(my_nodes,my_beam3d_14_pars))
my_elements.append(beam3d(my_nodes,my_beam3d_15_pars))
my_elements.append(beam3d(my_nodes,my_beam3d_16_pars))
my_elements.append(beam3d(my_nodes,my_beam3d_17_pars))


# Step 5: Create the Finite Element Method model

# In[97]:


# Model parameters
my_model_pars = {}

# x - 1 (right)
# y - 2 (inside)
# z - 3 (up)
# rotation around x axis - >4

# Constrained degrees of freedom
my_model_pars['dofs_c'] = np.array([[1, 1], [1, 2], [1, 3], [1, 4],
                                    [3, 2], [3, 3], [3, 4]])

# Degrees of freedom subjected to force history 
my_model_pars['dofs_f'] = np.array([[9, 1], [10, 1]])

# Degrees of freedom subjected to displacement history (vertical dofs of node 2)
my_model_pars['dofs_u'] = np.array([[5, 3], [10, 3]])

# Force history (21 steps) - Ensure g_f is a 2D array with dimensions (2, 21)
my_model_pars['g_f'] = np.array([np.multiply(fhs,-1), np.multiply(fhs,1)])

# Displacement history (21 steps) - Ensure  is a 2D array with dimensions (2, 21)
my_model_pars['g_u'] = np.array([np.multiply(uvs,-0.1), np.multiply(uvs,0.9)])

# Create the model
my_model = model(my_nodes, my_elements, my_model_pars)

# list of model dofs (left column is node number, right column is the dof number)
[np.multiply(fhs,-1), np.multiply(fhs,1)]


# Step 6: create and execute the simulation
# 
# We need to fix the error when running the static_analysis() method in my_simulation (YaFEM). It occurs because the sizes of matrices aren't matching so the multiplication can't work.

# In[98]:

# simulation parameters
simulation_pars = {}

# create the simulation
my_simulation = simulation(my_model,simulation_pars)

# perform static analysis (u: displacements, l: applied forces, r: restoring force)
[u,l,r] = my_simulation.static_analysis()
# [u, v, a, r] = my_simulation.dynamic_analysis()