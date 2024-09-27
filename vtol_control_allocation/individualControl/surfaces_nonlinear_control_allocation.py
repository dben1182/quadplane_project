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
from tools.rotations import Quaternion2Euler, Quaternion2Rotation, Euler2Rotation, Euler2Quaternion
from message_types.msg_state import MsgState

from vtol_dynamics.vtol_dynamics import VTOLDynamics

import parameters.simulation_parameters as SIM
import copy




CA_ROTOR_PULLER = 0
CA_ELEVATOR = 1
CA_AILERON = 2
CA_RUDDER = 3



class SurfacesNonlinearControlAllocation():

    #creates the init class
    def __init__(self):
        self.previous_solution = CA.init_actuators_surfaces

        self.max_iter = CA.max_iter

        #creates variable to store the state
        self.state = np.ndarray

        #vector to store the wind velocity data
        self.wind = np.ndarray

        #vector to store the body frame velocity of the aircraft relative to the airmass
        self.velocity_air = np.ndarray

        #creates variable to store the other important information for the state
        self.Va = 0.0
        self.alpha = 0.0
        self.beta = 0.0

        #creates a copy of the VTOL Dynamics class and hope here to be able 
        #to store a copy so we can use the forces and moments functions which
        #are built into the system

        #the actuators we control in this part are:
        #1. forward throttle
        #2. elevator
        #3. aileron
        #4. rudder

        #creates the actuator boundaries 
        self.actuator_bounds = [(0.0,  1.0), #bounds for forward throttle
                                (-1.0, 1.0), #bounds for elevator
                                (-1.0, 1.0), #bounds for aileron
                                (-1.0, 1.0)] #bounds for rudder
        

    #crates the update function
    def update(self, thrust: np.ndarray, 
                     torques: np.ndarray,
                     state: np.ndarray,
                     airspeed: float,
                     wind: np.ndarray)->MsgDelta:

        #stores the state array, which we use for calculating the wrench and the optimization algorithm
        self.state = state

        #stores the wind vector
        self.wind = wind

        #calls the function to update the velocity data, the Va, alpha, and beta, so it's called every iteration
        self.update_velocity_data(wind=self.wind)



        wrench_desired = np.concatenate([thrust, torques], axis=0).reshape(-1)
        v_body = state[3:6]

        #computes the nonlinear optimization command
        actuator_commands = self.compute_nonlinear_optimization(wrench_desired, v_body, airspeed)

        return self.formulate_control_message(actuator_commands)
    

    #creates the computational nonlinear 
    def compute_nonlinear_optimization(self,
                                       wrench_desired: np.ndarray,
                                       v_body: np.ndarray,
                                       airspeed: float)->np.ndarray:
        
        #gets the previous solution
        delta_0 = self.previous_solution

        #gets the whole result
        delta_result = minimize(self.nonlinear_control_optimization,
                                delta_0,
                                args=(wrench_desired, v_body, airspeed, delta_0),
                                bounds= self.actuator_bounds,
                                jac=True,
                                options={'maxiter': self.max_iter})
        
        #gets the delta output
        delta_array_output = delta_result.x

        return delta_array_output
    

    #defines a function to convert a control message. In this particular control setup, we need
    #to have the vertical rotors set to zeros
    def formulate_control_message(self, actuator_commands: np.ndarray) -> MsgDelta:

        control_message = MsgDelta()

        #converts the throttle commands into the message
        #vertical throttles
        control_message.throttle_0 = 0.0
        control_message.throttle_1 = 0.0
        control_message.throttle_2 = 0.0
        control_message.throttle_3 = 0.0
        #forward throttle
        control_message.throttle_4 = actuator_commands[CA_ROTOR_PULLER]
        #sets the control surfaces
        control_message.elevator = actuator_commands[CA_ELEVATOR]
        control_message.aileron = actuator_commands[CA_AILERON]
        control_message.rudder = actuator_commands[CA_RUDDER]

        #returns the control message
        return control_message
    

    #Possible deltas are fed into this function by the minimizer and then selected to have the 
    #smallest value for the objective function
    #this function is the objective function
    def nonlinear_control_optimization(self,
                                       delta_array: np.ndarray,
                                       wrench_desired: np.ndarray,
                                       v_body: np.ndarray,
                                       airspeed: float,
                                       previous_solution: np.ndarray) -> tuple[float, np.ndarray]:
        
        #gets the Tau mixing matrix
        K_Tau = CA.K_Tau
        #gets the delta mixing matrix
        K_delta = CA.K_delta(airspeed)
        #sets the delta_desired
        delta_desired = CA.actuators_surfaces_desired

        #gets airspeeds perpendicular to the respective propellors
        Va_pull = v_body.item(0)
        Va_lift = v_body.item(2)

        #creates an array of the delta_t for the input
        delta_t = np.array([0.0, 0.0, 0.0, 0.0, delta_array[CA_ROTOR_PULLER]])

        #gets the thrust, torque, thrust derivative, and torque derivatives
        #that is, the derivative of the individual thrust and torque produced
        #by the individual rotor
        motor_thrusts, motor_moments, motor_thrust_derivatives, motor_moment_derivatives = \
              rotor_thrust_torque_derivative(delta_t,
                                             [Va_lift, Va_lift, Va_lift, Va_lift, Va_pull],
                                             VTOL.prop_dirs.tolist())
        

        #computs the elevon forces
        elevator_force_coefficients = calculate_elevator_force_coef(v_body, airspeed)


        #calculates the thrust and torque actually achieved
        wrench_achieved = self.calculate_wrench_achieved(delta_array=delta_array,
                                                         motor_thrusts=motor_thrusts,
                                                         motor_moments=motor_moments,
                                                         elevator_force_coefficients=elevator_force_coefficients,
                                                         airspeed=airspeed,
                                                         v_body=v_body)
        
        #reshapes the wrench desired so we get the right size
        wrench_desired = np.reshape(wrench_desired, (5,1))
        #calculates wrench error
        wrenchError = wrench_desired - wrench_achieved
        #calculates the 2 norm for the objective function
        objective_norm = 0.5 * wrenchError.T @ K_Tau @ wrenchError
        
        
        #gets the wrench Jacobian with respect to the deltas
        wrench_Jacobian = calc_thrust_torque_achieved_der(delta=delta_array, 
                                                          motor_thrusts=motor_thrusts,
                                                          motor_torques=motor_moments,
                                                          thrust_der=motor_thrust_derivatives,
                                                          torque_der=motor_moment_derivatives,
                                                          elevator_force_coef=elevator_force_coefficients,
                                                          airspeed=self.Va)
        
        #gets the gradient of the objective function
        objective_gradient = -wrench_Jacobian @ K_Tau @ wrenchError

        #returns the objective norm, and the Objective gradient
        return objective_norm, objective_gradient


    #function that calculates the thrust and torque achieved
    def calculate_wrench_achieved(self,
                                  delta_array: np.ndarray, 
                                  motor_thrusts: np.ndarray,
                                  motor_moments: np.ndarray,
                                  elevator_force_coefficients,
                                  airspeed: float,
                                  v_body: np.ndarray):
        

        #gets the elevator, aileron, and rudder from the delta array
        delta_t_forward = delta_array[0]
        delta_elevator = delta_array[1]
        delta_aileron = delta_array[2]
        delta_rudder = delta_array[3]
        

        #gets the roll, pitch and yaw rates
        phi, theta, psi = Quaternion2Euler(self.state[6:10])
        p = self.state.item(10)
        q = self.state.item(11)
        r = self.state.item(12)

        # compute gravitaional forces
        #gets the state quaternion for attitude
        e0 = self.state.item(6)
        ex = self.state.item(7)
        ey = self.state.item(8)
        ez = self.state.item(9)

        #gets the force due to gravity
        f_g = VTOL.mass*VTOL.gravity * np.array([[2*(ex*ez-ey*e0)],
                                                 [2*(ey*ez+ex*e0)],
                                                 [ez**2+e0**2-ex**2-ey**2]])     

        # compute Lift and Drag coefficients
        sigma = (1 + np.exp(-VTOL.M*(self.alpha-VTOL.alpha0)) + np.exp(VTOL.M*(self.alpha+VTOL.alpha0))) / \
            ((1 + np.exp(-VTOL.M*(self.alpha-VTOL.alpha0))) * (1 + np.exp(VTOL.M*(self.alpha+VTOL.alpha0)))) 
        CL = (1-sigma)*(VTOL.C_L_0+VTOL.C_L_alpha*self.alpha) + \
            sigma*2*np.sign(self.alpha)*(np.sin(self.alpha)**2)*np.cos(self.alpha)
        CD = VTOL.C_D_p + ((VTOL.C_L_0 + VTOL.C_L_alpha*self.alpha)**2) / (np.pi*VTOL.e*VTOL.AR)


        # compute Lift and Drag Forces
        Gamma_Va = (1/2)*VTOL.rho*(self.Va**2)*VTOL.S_wing
        
        F_lift = (1/2)*VTOL.rho*(self.Va**2)*VTOL.S_wing*(CL + VTOL.C_L_q*VTOL.c/(2*self.Va)*q \
            + VTOL.C_L_delta_e*delta_elevator)
        

        F_drag = (1/2)*VTOL.rho*(self.Va**2)*VTOL.S_wing*\
            (CD + VTOL.C_D_q*VTOL.c/(2*self.Va)*q + VTOL.C_D_delta_e*delta_elevator)




        # compute longitudinal forces in body frame
        f_body = np.array([[np.cos(self.alpha), -np.sin(self.alpha)],
                           [np.sin(self.alpha), np.cos(self.alpha)]]) @ \
                               np.array([[-F_drag], [-F_lift]])
        fx = f_body.item(0) + f_g.item(0) + motor_thrusts[4]
        fz = f_body.item(1) + f_g.item(2)



        #gets the Moments produced by the Rotors, both the lever arm moments caused by thrust
        #and the aerodynamic moments caused by the propellors, and sums them all together
        
        Mx_rotors = -(motor_moments[4])
        
        Mz_rotors = motor_moments[0] + motor_moments[1] + motor_moments[2] + motor_moments[3]

        # compute logitudinal torque in body frame
        # for the aerodynamic portions
        My = (1/2)*VTOL.rho*(self.Va**2)*VTOL.S_wing*VTOL.c * \
            (VTOL.C_m_0 + VTOL.C_m_alpha*self.alpha + VTOL.C_m_q*VTOL.c/(2*self.Va)*q \
            + VTOL.C_m_delta_e*delta_elevator)

        # compute lateral torques in body frame
        #for the aerodynamic portions
        Mx = (1/2)*VTOL.rho*(self.Va**2)*VTOL.S_wing*VTOL.b * \
            (VTOL.C_ell_0 + VTOL.C_ell_beta*self.beta \
            + VTOL.C_ell_p*VTOL.b/(2*self.Va)*p + VTOL.C_ell_r*VTOL.b/(2*self.Va)*r \
            + VTOL.C_ell_delta_a*delta_aileron + VTOL.C_ell_delta_r*delta_rudder)
        
        #for the aerodynamic portions
        Mz = (.5)*VTOL.rho*(self.Va**2)*VTOL.S_wing*VTOL.b * \
            (VTOL.C_n_0 + VTOL.C_n_beta*self.beta \
            + VTOL.C_n_p*VTOL.b/(2*self.Va)*p + VTOL.C_n_r*VTOL.b/(2*self.Va)*r \
            + VTOL.C_n_delta_a*delta_aileron + VTOL.C_n_delta_r*delta_rudder)
        
        #adds the 
        Mx += Mx_rotors
        Mz += Mz_rotors

        return np.array([[fx, fz, Mx, My, Mz]]).T
    

    #function to update the velocity data
    def update_velocity_data(self, wind: np.ndarray = np.zeros((6,1))):
        #gets the steady state wind
        wind_steady_state = wind[0:3]
        #gets the gust wind
        wind_gust = wind[3:6]

        # convert wind vector from world to body frame
        wind_body_frame = Quaternion2Rotation(self.state[6:10,:]) @ wind_steady_state + wind_gust

        #velocity vector relative to the airmass
        v_ground = self.state[3:6]
        #gets the air velocity 
        self.velocity_air = v_ground - wind_body_frame

        #gets the airspeed u, v, and w in the body frame
        u_air = self.velocity_air.item(0)
        v_air = self.velocity_air.item(1)
        w_air = self.velocity_air.item(2)

        #computes the total airspeed
        # compute airspeed
        self.Va = np.sqrt(u_air**2 + v_air**2 + w_air**2)
        # compute angle of attack
        if u_air == 0:
            self.alpha = 0
        else:
            self.alpha = np.arctan(w_air/u_air)
        # compute sideslip angle
        if self.Va == 0:
            self.beta = np.inf
        else:
            self.beta = np.arcsin(v_air / np.sqrt(u_air**2+v_air**2+w_air**2))



# Calculates the gradient of the thrust and torque achieved
def calc_thrust_torque_achieved_der(delta: np.ndarray, 
                                    motor_thrusts: np.ndarray, 
                                    motor_torques: np.ndarray, 
                                    thrust_der: np.ndarray, 
                                    torque_der: np.ndarray, 
                                    elevator_force_coef: np.ndarray, 
                                    airspeed: float):

    #gets the scaling factor for the coefficients
    Gamma = .5 * VTOL.rho * airspeed**2 * VTOL.S_wing

    #Gradient of T_x
    T_x_der =  [thrust_der[4],
                elevator_force_coef[0],
                0.,
                0.]
    #Gradient of T_z
    T_z_der =  [0.,
                elevator_force_coef[1],
                0.,
                0.]
    #Gradient of Tau_x
    Tau_x_der = [torque_der[4],
                0.0,
                Gamma * VTOL.b * VTOL.C_ell_delta_a,
                Gamma * VTOL.b * VTOL.C_ell_delta_r]
    #Gradient of Tau_y
    Tau_y_der = [0.0,
                Gamma * VTOL.c * VTOL.C_m_delta_e,
                0.0,
                0.0]
    #Gradient of Tau_z
    Tau_z_der = [0.0,
                0.0,
                Gamma * VTOL.b * VTOL.C_n_delta_a,
                Gamma * VTOL.b * VTOL.C_n_delta_r]
    return np.array([T_x_der, T_z_der, Tau_x_der, Tau_y_der, Tau_z_der]).T



def calculate_elevator_force_coef(v_body, airspeed):
    #gets the forces scaling factor
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



#function that gets the rotor thrust, torques, and derivatives of those two,
#with respect to the delta inputs
#arguments:
#1. delta: MsgDelta message class
#2. Va: array for the airspeed perpendicular to the five propellors
#3. dir: array of the directions of rotation of the 5 props. For calculating moments
#returns:
#1. motor_thrusts: the Thrust output from each of the five rotors
#2. motor_moments: the aerodynamic moments from each of the five rotors
#3. motor_thrusts_derivatives: list of the derivatives of each motor thrust with respect to that delta_t
#4. motor_moment_derivatives:  list of the derivatives of each motor moment with respect to that delta_t
def rotor_thrust_torque_derivative(delta: np.ndarray, Va: np.ndarray, dir: np.ndarray)-> tuple[list, list, list, list]:
    motor_thrusts = list()
    motor_moments = list()
    motor_thrust_derivatives = list()
    motor_moment_derivatives = list()
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

        motor_thrusts.append(T_p)
        motor_moments.append(Q_p)
        motor_thrust_derivatives.append(T_p_der)
        motor_moment_derivatives.append(Q_p_der)

    return motor_thrusts, motor_moments, motor_thrust_derivatives, motor_moment_derivatives






