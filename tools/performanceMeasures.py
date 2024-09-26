#this file creates functions to record and measure performance of the airplane through the whole trajectory.
import sys
sys.path.append('..')

import numpy as np
import parameters.simulation_parameters as SIM


#our performance measures are: 
#1. Positional Error Integral
#2. Total Energy consumption (Integral of Power Consumption)



#the performance Measures super class has several subclasses, each of which perform the individual metric
#calculation, and then puts everything together in the bigger performanceMeasures super class
class performanceMeasures:

    #creates the init function
    def __init__(self, Ts):

        #creates an instance of the error tracking class
        self.posErrorTracker = positionErrorTracker(Ts=Ts)



class positionErrorTracker:

    def __init__(self, Ts: float):

        self.Ts = Ts

        #creates array to store the positional vectors
        self.desiredPosition = np.ndarray((3,0))
        self.actualPosition = np.ndarray((3,0))
        #creates array to store the error position
        self.errorPosition = np.ndarray((3,0))

        #creates array to store the 1 and 2 norms of the positional error
        self.errorPosition1Norm = np.ndarray((1,0))
        self.errorPosition2Norm = np.ndarray((1,0))

        #sets the error integrals for 1 and 2 norms
        self.errorPosition1Norm_Integral = 0.0
        self.errorPosition2Norm_Integral = 0.0


    #creates the positional error tracking function
    def update(self, desiredPosition: np.ndarray, actualPosition: np.ndarray):
        
        #section to perform the calculations
        #reshapes the input vectors
        desiredPosition = np.reshape(desiredPosition, (3,1))
        actualPosition = np.reshape(actualPosition, (3,1))
        #gets the error position
        errorPosition = actualPosition - desiredPosition

        #calculates the 1 and 2 norms for the error position
        error_1Norm = np.linalg.norm(x=errorPosition, ord=1)
        error_2Norm = np.linalg.norm(x=errorPosition, ord=2)
        
        #section to store everything

        #updates the positional error tracker
        self.desiredPosition = np.concatenate((self.desiredPosition, desiredPosition), axis=1)
        self.actualPosition = np.concatenate((self.actualPosition, actualPosition), axis=1)
        #stores the error position
        self.errorPosition = np.concatenate((self.errorPosition, errorPosition), axis=1)

        #saves the normalizations
        self.errorPosition1Norm = np.concatenate((self.errorPosition1Norm, np.reshape(error_1Norm, (1,1))), axis=1)
        self.errorPosition2Norm = np.concatenate((self.errorPosition2Norm, np.reshape(error_2Norm, (1,1))), axis=1)


        #updates the integrals
        self.errorPosition1Norm_Integral += np.abs(error_1Norm*self.Ts)
        self.errorPosition2Norm_Integral += np.abs(error_2Norm*self.Ts)


    #gets the position norms
    def getErrorPositionNorms(self, degree: float = 2):
        #case degree is 1
        if degree == 1:
            return self.errorPosition1Norm
        elif degree == 2:
            return self.errorPosition2Norm
        else:
            return self.errorPosition2Norm

    #defines functions to get the various 

    #creates functions to reset the trackers
    def resetTracker(self):
        #creates array to store the positional vectors
        self.desiredPosition = np.ndarray
        self.actualPosition = np.ndarray
        #creates array to store the error position
        self.errorPosition = np.ndarray

        #creates array to store the 1 and 2 norms of the positional error
        self.errorPosition1Norm = np.ndarray
        self.errorPosition2Norm = np.ndarray

        #sets the error integrals for 1 and 2 norms
        self.errorPosition1Norm_Integral = 0.0
        self.errorPosition2Norm_Integral = 0.0




