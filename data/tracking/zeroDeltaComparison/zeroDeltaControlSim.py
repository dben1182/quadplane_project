#this file runs a simulation using Mason's code that attempts to drive deltas to zero

#/usr/bin/python3

import os, sys
# insert parent directory at beginning of python search path
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[3]))
import numpy as np
import time
import parameters.simulation_parameters as SIM

from vtol_viewer.vtol_viewer import VTOLViewer
from chap3.data_viewer import DataViewer
from vtol_dynamics.vtol_dynamics import VTOLDynamics
from chap4.wind_simulation import WindSimulation
from message_types.msg_delta import MsgDelta
from message_types.msg_state import MsgState
from vtol_control_allocation.simultaneousControl.nonlinear_control_allocation_zeroDelta import NonlinearControlAllocation
from low_level_controller.rate_control import RateControl
from trajectory_tracker.pitch_free_trajectory_tracker import PitchFreeTrajectoryTracker
from trajectory_tracker.pitch_control import PitchControl
from trajectory_tracker.attitude_control import AttitudeControl
from tools.msg_convert import *
from tools.rotations import Quaternion2Euler, Rotation2Quaternion, Rotation2Euler, Quaternion2Rotation

#imports the positional tracker
from tools.performanceMeasures import performanceMeasures


# trajectories
import vtol_trajectory_generator.trajectories as TRAJ

import pandas as pd

def main():
    np.set_printoptions(precision=4, linewidth=200, suppress=True)
    # initialize viewers
    state_view = DataViewer()
    vtol_view = VTOLViewer()

    # initialize elements of the architecture
    wind = WindSimulation(SIM.ts_simulation)
    vtol = VTOLDynamics(SIM.ts_simulation)

    # INITIALIZE TRAJECTORIES
    traj = TRAJ.tcl
    
    ## ---------------------------------

    # draw the trajectory
    SIM.end_time = traj.end_time
    trajectory_position_points = traj.get_position_pts(.01)

    vtol_view.addTrajectory(trajectory_position_points[:3,:])

    # initialize geometric controller
    traj_tracker = PitchFreeTrajectoryTracker()
    att_ctrl = AttitudeControl()
    pitch_ctrl = PitchControl()

    #initialize low level control
    rate_control = RateControl(ts_control=SIM.ts_control)
    control_alloc = NonlinearControlAllocation()




    #creates instance of the performance measures class
    performance = performanceMeasures(Ts=SIM.ts_simulation)

    # initialize command message
    delta = MsgDelta()

    #calculate_trim
    vtol._update_true_state()

    # initialize the simulation time
    sim_time = SIM.start_time
    Ts = SIM.ts_simulation

    time_hist = []
    comp_time_hist = []


    # main simulation loop
    while sim_time < SIM.end_time:
        #-------observer-------------
        measurements = vtol.sensors()  # get sensor measurements
        estimated_state = vtol._state  # estimated state is current state

        ctrl_start_time = time.time()
        # ------ Trajectory follower
        traj_derivatives_at_t = traj.traj_msg(sim_time)

        #gets the positional errors
        actualPosition = estimated_state[0:3,:]
        #commanded position
        commandedPosition = traj_derivatives_at_t[0:3,0].reshape((3,1))
        #throws those into the metric equations
        performance.posErrorTracker.update(desiredPosition=commandedPosition,
                                           actualPosition=actualPosition)
      

        #------- High Level controller-------------
        #from the trajectory tracker, based on our estimated state, we get a Thrust desired (Fx, Fz)
        #and 
        T, R_d = traj_tracker.update(estimated_state, traj_derivatives_at_t)
        T, R_d = pitch_ctrl.update(T, R_d, estimated_state[3:6])
        T = T.reshape(-1)
        #gets the commanded roll rates vector, based on the rotation matrix
        omega_c = att_ctrl.update(Quaternion2Rotation(estimated_state[6:10]), R_d)
        omega_c = omega_c.reshape(-1)

        #------- Low Level Controller -------------
        #gets the actual roll rates vector
        omega = estimated_state[10:13,0]
        #gets the commanded tau based on the commanded and actual roll rates
        tau_c = rate_control.update(omega_c, omega)
        #gets the delta commands from the control allocation
        delta = control_alloc.update(T, tau_c, estimated_state, vtol._Va)
        ctrl_end_time = time.time()

        #gets the motor electrical vectors
        V_in, I_in, P_in = vtol.getMotorElectricals()

        #writes down the arrays in the performance matrix
        performance.energyTracker.update(V_in=V_in, I_in=I_in, P_in=P_in)


        #-------update physical system-------------
        vtol.update(delta, np.array([[0.0,0.0,0.0,0.0,0.0,0.0]]).T)  # propagate the MAV dynamics

        #-------update viewers-------------
        pd_i = traj_derivatives_at_t[0:3,0]
        va_d = np.linalg.norm(traj_derivatives_at_t[0:3,1])
        desired_state = MsgState()
        desired_state.north = pd_i.item(0)
        desired_state.east = pd_i.item(1)
        desired_state.altitude = -pd_i.item(2)
        desired_state.Va = va_d
        desired_state.phi, desired_state.theta, desired_state.chi = Rotation2Euler(R_d)

        vtol_view.update(vtol.true_state)
        state_view.update(vtol.true_state, vtol.true_state, desired_state, delta, Ts)
        time_hist.append(sim_time)
        comp_time_hist.append(ctrl_end_time - ctrl_start_time)

        #-------increment time-------------
        sim_time += Ts

    #gets the error information
    errorPosition1Norm = performance.posErrorTracker.getErrorPositionNorms(degree=1)
    errorPosition2Norm = performance.posErrorTracker.getErrorPositionNorms(degree=2)

    errorPosition1NormIntegral = performance.posErrorTracker.getErrorIntegralNorms(degree=1)
    errorPosition2NormIntegral = performance.posErrorTracker.getErrorIntegralNorms(degree=2)
    

    #writes the various forms of information to csv files
    errorPosition1Norm_df = pd.DataFrame(errorPosition1Norm)
    errorPosition1Norm_df.to_csv('/home/dben1182/Documents/quadplane_project_modified/data/tracking/zeroDeltaComparison/zeroDelta/norm1.csv', index=False, header=False)

    errorPosition2Norm_df = pd.DataFrame(errorPosition2Norm)
    errorPosition2Norm_df.to_csv('/home/dben1182/Documents/quadplane_project_modified/data/tracking/zeroDeltaComparison/zeroDelta/norm2.csv', index=False, header=False)

    #writes the integrals outpu
    errorPosition1NormIntegral_df = pd.DataFrame(np.array([[errorPosition1NormIntegral]]))
    errorPosition1NormIntegral_df.to_csv('/home/dben1182/Documents/quadplane_project_modified/data/tracking/zeroDeltaComparison/zeroDelta/norm1_integral.csv', index=False, header=False)

    errorPosition2NormIntegral_df = pd.DataFrame(np.array([[errorPosition2NormIntegral]]))
    errorPosition2NormIntegral_df.to_csv('/home/dben1182/Documents/quadplane_project_modified/data/tracking/zeroDeltaComparison/zeroDelta/norm2_integral.csv', index=False, header=False)

    #gets the complete power array
    completePower = performance.energyTracker.getCompletePower()
    normPower = performance.energyTracker.getNormPower()
    totalEnergy = performance.energyTracker.getTotalEnergy()

    completePower_df = pd.DataFrame(completePower)
    completePower_df.to_csv('/home/dben1182/Documents/quadplane_project_modified/data/tracking/zeroDeltaComparison/zeroDelta/completePower.csv', index=False, header=False)

    normPower_df = pd.DataFrame(normPower)
    normPower_df.to_csv('/home/dben1182/Documents/quadplane_project_modified/data/tracking/zeroDeltaComparison/zeroDelta/normPower.csv', index=False, header=False)


    totalEnergy_df = pd.DataFrame(totalEnergy)
    totalEnergy_df.to_csv('/home/dben1182/Documents/quadplane_project_modified/data/tracking/zeroDeltaComparison/zeroDelta/totalEnergy.csv', index=False, header=False)




    print("Done with simulation")
    while (True):
        vtol_view.app.processEvents()
        state_view.plotter.app.processEvents()
    return

main()
