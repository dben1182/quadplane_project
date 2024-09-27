
#imports the delta message class
from message_types.msg_delta import MsgDelta


#stores the trim values for the four controls of the aerosonde

throttleTrim = 0.676752
aileronTrim = 0.001836
elevatorTrim = -0.8
rudderTrim = -0.000303




#instantiates the Message delta class
trimDelta = MsgDelta(throttle_0=0.0,
                     throttle_1=0.0,
                     throttle_2=0.0,
                     throttle_3=0.0,
                     throttle_4=throttleTrim,
                     elevator=elevatorTrim,
                     aileron=aileronTrim,
                     rudder=rudderTrim)

