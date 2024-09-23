#this file implements a trajectory for a fixed wing device.
#That is that we are using a primitive version of the trajectory follower, with only a straight line segment going forward for a while

import sys
import numpy as np

sys.path.append('..')
from vtol_trajectory_generator.trajectory_generator import LineSegment, TrajectoryGenerator


tcl_fast = TrajectoryGenerator()
tcl_fast.add_line_segment(LineSegment(start_pos=np.array([[0.0, 0.0, 0.0]]).T,
                                      start_vel=10.0,
                                      end_pos=np.array([[500.0, 0.0, 0.0]]),
                                      end_vel=10.0))
