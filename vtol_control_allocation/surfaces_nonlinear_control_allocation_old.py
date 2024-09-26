#this file implements the nonlinear control allocation
#for the control surfaces and the forward thruster.


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


CA_ROTOR_PULLER = 0
CA_ELEVATOR = 1
CA_AILERON = 2
CA_RUDDER = 3


class SurfacesNonlinearControlAllocation():

    #creates the init class
    def __init__(self):
        self.previous_solution = CA.init_actuators_surfaces

        self.max_iter = CA.max_iter

        #creates the actuator boundaries 
        self.actuator_bounds = [(0.0,  1.0),
                                (-1.0, 1.0),
                                (-1.0, 1.0),
                                (-1.0, 1.0)]
        
    #crates the update function
    def update(self, thrust: np.ndarray, 
                     torques: np.ndarray,
                     state: np.ndarray,
                     airspeed: float)->MsgDelta:
        
        thrust_torque_desired = np.concatenate([thrust, torques], axis=0).reshape(-1)
        v_body = state[3:6]

        actuator_commands = self._compute_nonlinear_optimization(thrust_torque_desired, v_body, airspeed)

        return self._formulate_ctrl_msg(actuator_commands)
    

    #creates the computational optimization function
    def _compute_nonlinear_optimization(self, thrust_torque_desired: np.ndarray, v_body: np.ndarray, airspeed: float)->np.ndarray:
        
        x0 = self.previous_solution
        
        # Non linear optimizer gets optimization output and gradient from nonlinear_ctrl_optimization output
        res = minimize(
            nonlinear_ctrl_optimization, 
            x0,
            args=(thrust_torque_desired, v_body, airspeed, x0),
            bounds=self.actuator_bounds,
            jac=True,
            options={'maxiter': self.max_iter})
        
        #stores the previous solution as the current solution. Helps significantly with computational efficiency
        #and also with accuracy of the results
        self.previous_solution = res.x
        return res.x
    

    #defines a function that formulates a new control message from an array of 
    def _formulate_ctrl_msg(self, actuator_commands: np.ndarray) -> MsgDelta:
        ctrl_msg = MsgDelta()
        #converts the throttle commands to part of the mssage
        ctrl_msg.throttle_0 = 0.0
        ctrl_msg.throttle_1 = 0.0
        ctrl_msg.throttle_2 = 0.0
        ctrl_msg.throttle_3 = 0.0
        ctrl_msg.throttle_4 = actuator_commands[CA_ROTOR_PULLER]

        ctrl_msg.elevator = actuator_commands[CA_ELEVATOR]
        ctrl_msg.aileron = actuator_commands[CA_AILERON]
        ctrl_msg.rudder = actuator_commands[CA_RUDDER]
        return ctrl_msg
    
    #defines the nonlinear control optimization

#Arguments:
#1. x: array of control inputs: delta_t_forward, delta_e, delta_a, delta_r
def nonlinear_ctrl_optimization(delta: np.ndarray, 
                                thrust_torque_desired: np.ndarray, 
                                v_body: np.ndarray,
                                airspeed: float,
                                prev_solution: np.ndarray)->tuple[float, np.ndarray]:
    
    K_Tau = CA.K_Tau
    K_delta = CA.K_delta(airspeed)
    x_des = CA.actuators_desired
    # compute the thrust/torque and its derivative with respect to the change of throttle
    Va_lift = v_body.item(2)
    Va_pull = v_body.item(0)
    
    #creates a vector with all of the thruster control inputs
    delta_throttles = np.array([0.0,
                                0.0,
                                0.0,
                                0.0,
                                delta[0]])
    thrust, torque, thrust_der, torque_der = rotor_thrust_torque_der(delta_throttles,
                                                                     [Va_lift, Va_lift, Va_lift, Va_lift, Va_pull],
                                                                     VTOL.prop_dirs.tolist())
    #gets the elevator force coefficients
    elevator_force_coefs = calc_elevator_force(v_body, airspeed)
    #gets the actual output Thrust and Moment from the system
    thrust_torque_achieved = _calc_thrust_torque_achieved(x=delta, 
                                                          thrust=thrust, 
                                                          torque=torque,
                                                          elevator_force_coefs=elevator_force_coefs,
                                                          airspeed = airspeed,
                                                          v_body=v_body)
    #gets the difference between the thrust and torque desired and the actual thrust and torque achieved
    thrust_torque_error = thrust_torque_desired - thrust_torque_achieved
    #gets the objective function
    objective_function = 0.5* thrust_torque_error.T @ K_Tau @ thrust_torque_error
    #gets the gradient vector of the objective function
    thrust_torque_Jacobian = calc_thrust_torque_achieved_gradient(x=delta,
                                                                  thrust=thrust,
                                                                  torque=torque,
                                                                  thrust_der=thrust_der,
                                                                  torque_der=torque_der,
                                                                  elevator_force_coef=elevator_force_coefs,
                                                                  airspeed=airspeed)
    
    #gets the diff norml of the derivative
    gradient_derivative = -thrust_torque_Jacobian @ K_Tau @ thrust_torque_error
    #returns the objective function and its derivative
    return objective_function, gradient_derivative
        


# Returns the thrust and torque achieved by certain deltas
# x is the proposed setpoint for each of the actuators
# thrust is a three-dimensional vector of thrust achieved by each of the three rotors
# torque is a three-dimensional vector representing the torque caused by each of the 
# rotors
def _calc_thrust_torque_achieved(x, thrust, torque, elevator_force_coefs, airspeed, v_body):

    Gamma = .5 * VTOL.rho * airspeed**2 * VTOL.S_wing

    T_x = thrust[4] + elevator_force_coefs[0] * (x[CA_ELEVATOR])
    T_z = -(thrust[0] + thrust[1] + thrust[2] + thrust[3]) \
        + elevator_force_coefs[0] * (x[CA_ELEVATOR])
    Tau_x = torque[4] + Gamma * VTOL.b * VTOL.C_ell_delta_a * x[CA_AILERON] \
        + Gamma * VTOL.b * VTOL.C_ell_delta_r * x[CA_RUDDER]
    for i in range(4):
        Tau_x -= VTOL.rotor_qs[i].item(1) * thrust[i]
    Tau_y = Gamma * VTOL.c * VTOL.C_m_delta_e * x[CA_ELEVATOR]
    for i in range(4):
        Tau_y += VTOL.rotor_qs[i].item(0) * thrust[i]
    Tau_z = torque[0] + torque[1] + torque[2] + torque[3] \
        + Gamma * VTOL.b * \
        (VTOL.C_n_delta_a*x[CA_AILERON] + VTOL.C_n_delta_r*x[CA_RUDDER])

    return np.array([T_x, T_z, Tau_x, Tau_y, Tau_z]).T

# Calculates the gradient of the thrust and torque achieved
def calc_thrust_torque_achieved_gradient(x: np.ndarray, 
                                         thrust: np.ndarray, 
                                         torque: np.ndarray, 
                                         thrust_der: np.ndarray, 
                                         torque_der: np.ndarray, 
                                         elevator_force_coef, 
                                         airspeed):

    Gamma = .5 * VTOL.rho * airspeed**2 * VTOL.S_wing


    T_x_der =  [thrust_der[4],
                elevator_force_coef[0],
                0.,
                0.]
    T_z_der =  [0.,
                elevator_force_coef[1],
                0.,
                0.]
    Tau_x_der = [torque_der[4],
                0.0,
                Gamma * VTOL.b * VTOL.C_ell_delta_a,
                Gamma * VTOL.b * VTOL.C_ell_delta_r]
    Tau_y_der = [0.0,
                Gamma * VTOL.c * VTOL.C_m_delta_e,
                0.0,
                0.0]
    Tau_z_der = [0.0,
                0.0,
                Gamma * VTOL.b * VTOL.C_n_delta_a,
                Gamma * VTOL.b * VTOL.C_n_delta_r]
    returnArray = np.array([T_x_der, T_z_der, Tau_x_der, Tau_y_der, Tau_z_der]).T
    
    return returnArray

def rotor_thrust_torque_der(delta, Va, dir):
    thrust = list()
    torque = list()
    thrust_der = list()
    torque_der = list()
    for i in range(5):
        # compute thrust and torque due to propeller  (See addendum by McLain)
        # grab motor/prop params
        C_Q0 = VTOL.C_Q0
        C_Q1 = VTOL.C_Q1
        C_T0 = VTOL.C_T0
        C_Q2 = VTOL.C_Q2
        C_T1 = VTOL.C_T1
        C_T2 = VTOL.C_T2
        D_prop = VTOL.D_prop
        KQ = VTOL.KQ
        R_motor = VTOL.R_motor
        i0 = VTOL.i0
        # map delta_t throttle command(0 to 1) into motor input voltage
        V_in = VTOL.V_max * delta[i]
        V_in_der = VTOL.V_max
        # Quadratic formula to solve for motor speed
        a = C_Q0 * VTOL.rho * np.power(D_prop, 5) \
            / ((2.*np.pi)**2)
        b = (C_Q1 * VTOL.rho * np.power(D_prop, 4)
            / (2.*np.pi)) * Va[i] + KQ**2/R_motor
        c = C_Q2 * VTOL.rho * np.power(D_prop, 3) \
            * (Va[i])**2 - (KQ / R_motor) * V_in + KQ * i0
        c_der = (KQ / R_motor) * V_in_der
        # Consider only positive root
        Omega_op = (-b + np.sqrt(b**2 - 4*a*c)) / (2.*a)
        Omega_op_der = c_der / np.sqrt(b**2 - 4*a*c)
        # compute advance ratio
        J_op = 2 * np.pi * Va[i] / (Omega_op * D_prop)
        J_op_der = -2 * np.pi * Va[i] * Omega_op_der / (Omega_op**2 * D_prop)
        # compute non-dimensionalized coefficients of thrust and torque
        C_T = C_T2 * J_op**2 + C_T1 * J_op + C_T0
        C_Q = C_Q2 * J_op**2 + C_Q1 * J_op + C_Q0
        C_T_der = 2 * C_T2 * J_op * J_op_der + C_T1 * J_op_der
        C_Q_der = 2 * C_Q2 * J_op * J_op_der + C_Q1 * J_op_der
        # add thrust and torque due to propeller
        n = Omega_op / (2 * np.pi)
        T_p = VTOL.rho * n**2 * np.power(D_prop, 4) * C_T
        Q_p = VTOL.rho * n**2 * np.power(D_prop, 5) * C_Q
        T_p_der = VTOL.rho * Omega_op * Omega_op_der * np.power(D_prop, 4) * C_T / (2 * np.pi**2) + \
            VTOL.rho * Omega_op**2 * np.power(D_prop, 4) * C_T_der / (2 * np.pi)**2
        Q_p_der = VTOL.rho * Omega_op * Omega_op_der * np.power(D_prop, 5) * C_Q / (2 * np.pi**2) + \
            VTOL.rho * Omega_op**2 * np.power(D_prop, 5) * C_Q_der / (2 * np.pi)**2
        # Flip moment sign for certain rotors
        Q_p *= -dir[i]
        Q_p_der *= -dir[i]

        thrust.append(T_p)
        torque.append(Q_p)
        thrust_der.append(T_p_der)
        torque_der.append(Q_p_der)

    return thrust, torque, thrust_der, torque_der

def calc_elevator_force(v_body, airspeed):
    Gamma = .5 * VTOL.rho * airspeed**2 * VTOL.S_wing
    elevator_lift_coef = Gamma * VTOL.C_L_delta_e
    elevator_drag_coef = Gamma * VTOL.C_D_delta_e
    if v_body[0] != 0:
        alpha = np.arctan2(v_body[2], v_body[0]).item(0)
    else:
        alpha = 0
    elevator_force_coefs = [-np.cos(alpha) * elevator_drag_coef + np.sin(alpha) * elevator_lift_coef,
                          -np.sin(alpha) * elevator_drag_coef - np.cos(alpha) * elevator_lift_coef]
    return elevator_force_coefs

def calc_thrust_torque_achieved(x, v_body, airspeed):

    Va_lift = v_body.item(2)
    Va_pull = v_body.item(0)

    thrust, torque, thrust_der, torque_der = \
        rotor_thrust_torque_der(x[CA_ROTOR_FRONT_RIGHT:CA_ROTOR_PULLER + 1], 
        [Va_lift, Va_lift, Va_lift, Va_lift, Va_pull],
        VTOL.prop_dirs.tolist())

    elevator_force_coefs = calc_elevator_force(v_body, airspeed)

    return _calc_thrust_torque_achieved(x, thrust, torque, elevator_force_coefs, airspeed, v_body)
