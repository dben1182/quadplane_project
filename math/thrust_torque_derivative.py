#%%
#in this file, we are attempting to show the thrust and torque derivatives that Mason's code 
#alread has in it, and use it in comparison to our own calculations, 

#and we shall see who is right, and who is dead

import sympy as sp

from IPython.display import display


#creates the control variable, delta
delta = sp.symbols('delta')




#defines the symbolic constants of our function
C_Q0 = sp.symbols('C_{Q0}')
C_Q1 = sp.symbols('C_{Q1}')
C_T0 = sp.symbols('C_{T0}')
C_Q2 = sp.symbols('C_{Q2}')
C_T1 = sp.symbols('C_{T1}')
C_T2 = sp.symbols('C_{T2}')

D_prop = sp.symbols('D_{prop}')
KQ = sp.symbols('KQ')
R_motor = sp.symbols('R_{motor}')
i0 = sp.symbols('i0')

V_max = sp.symbols('V_{max}')

rho = sp.symbols('rho')

Va = sp.symbols('V_a')


# map delta_t throttle command(0 to 1) into motor input voltage
V_in = V_max * delta
V_in_der = V_max
# Quadratic formula to solve for motor speed
a = C_Q0 * rho * D_prop**5 / ((2.*sp.pi)**2)

b = ((C_Q1 * rho * D_prop**4 * Va) / (2.*sp.pi)) + KQ**2/R_motor

c = (C_Q2 * rho * (D_prop**3) * (Va**2)) - (KQ*V_in / R_motor) + KQ * i0
c_der = (KQ / R_motor) * V_in_der
# Consider only positive root
Omega_op = (-b + sp.sqrt(b**2 - 4*a*c)) / (2.*a)
Omega_op_der = c_der / (sp.sqrt(b**2 - 4*a*c))
# compute advance ratio
J_op = 2 * sp.pi * Va / (Omega_op * D_prop)
J_op_der = -2 * sp.pi * Va * Omega_op_der / (Omega_op**2 * D_prop)
# compute non-dimensionalized coefficients of thrust and torque
C_T = C_T2 * J_op**2 + C_T1 * J_op + C_T0
C_Q = C_Q2 * J_op**2 + C_Q1 * J_op + C_Q0
C_T_der = 2 * C_T2 * J_op * J_op_der + C_T1 * J_op_der
C_Q_der = 2 * C_Q2 * J_op * J_op_der + C_Q1 * J_op_der
# add thrust and torque due to propeller
n = Omega_op / (2 * sp.pi)
T_p = rho * (n**2) * (D_prop**4) * C_T
Q_p = rho * (n**2) * (D_prop**5) * C_Q
T_p_der = rho * Omega_op * Omega_op_der * (D_prop**4) * C_T / (2 * (sp.pi**2)) + \
    rho * (Omega_op**2) * (D_prop**4) * C_T_der / ((2 * sp.pi)**2)
Q_p_der = ((rho * Omega_op * Omega_op_der * (D_prop**5) * C_Q) / (2 * sp.pi**2)) + \
    (rho * (Omega_op**2) * (D_prop**5) * C_Q_der) / ((2 * sp.pi)**2)
# Flip moment sign for certain rotors




T_p_der_sym = sp.diff(T_p, delta)
display(T_p_der_sym)


display(T_p_der)

print('difference: ')
display(T_p_der_sym - T_p_der)


Q_p_der_sym = sp.diff(Q_p, delta)

print('Qp difference: ')
display(Q_p_der_sym - Q_p_der)


# %%
