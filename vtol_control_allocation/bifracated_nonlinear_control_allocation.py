#implements a modified version of the nonlinear control allocation piece, 
#which performs the control allocation in two steps.


from curses.ascii import ctrl
import re
from socket import AI_ALL
import sys
sys.path.append('..')
import numpy as np
from scipy.optimize import linprog, minimize, NonlinearConstraint

import parameters.quadplane_parameters as VTOL
import parameters.control_allocation_parameters as CA
from message_types.msg_delta import MsgDelta
from tools.rotations import Quaternion2Euler
from message_types.msg_state import MsgState

CA_ROTOR_FRONT_RIGHT = 0
CA_ROTOR_FRONT_LEFT = 1
CA_ROTOR_BACK_RIGHT = 2
CA_ROTOR_BACK_LEFT = 3
CA_ROTOR_PULLER = 4
CA_ELEVATOR = 5
CA_AILERON = 6
CA_RUDDER = 7


class BifracatedNonlinearControlAllocation():

    def __init__(self):

        #creates the previous solution to store as the whole previous solution
        self.whole_previous_solution = CA.init_actuators
        #sets the max iter
        self.max_iter = CA.max_iter

        #sets the actuator bounds
        self.actuator_bounds = [(0.0,  1.0),
                                (0.0,  1.0),
                                (0.0,  1.0),
                                (0.0,  1.0),
                                (0.0,  1.0),
                                (-1.0, 1.0),
                                (-1.0, 1.0),
                                (-1.0, 1.0)]
        
        #sets the delta_r bounds
        self.delta_r_bounds = self.actuator_bounds[0:4]
        #sets the delta_c bounds
        self.delta_c_bounds = self.actuator_bounds[4:8]

    #creates the update function
    #arguments:
    #1. thrust a length = 2 array of F_x and F_z
    #2. torques, a length = 3 array of M_x, M_y, and M_z
    #3. state, a length = 13 array of the actual state array, not the MsgState class
    #4. airspeed, a float for the airspeed

    #returns:
    #1: delta: a MsgDelta class for the control surfaces to use
    def update(self, thrust: np.ndarray,
                     torques: np.ndarray, 
                     state: np.ndarray,
                     airspeed: float):
        #gets the whole thrust and torque desired array
        thrust_torque_desired = np.concatenate([thrust, torques], axis=0).reshape(-1)

        #gets the body frame velocity (u, v, w)
        v_body = state[3:6]

        #gets the actuator commands
        actuator_commands = self._compute_nonlinear_optimization

    #creates the compute nonlinear optimization method for the system
    def _compute_nonlinear_optimization(self, thrust_torque_desired: np.ndarray, v_body: np.ndarray, airspeed: float)->np.ndarray:

        x0 = self.whole_previous_solution
        