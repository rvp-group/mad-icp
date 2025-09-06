from minisam import *
from minisam.sophus import *
import numpy as np


class PGOManager:
    def __init__(self):
        graph = FactorGraph()
        initialization = True

        prior_loss = DiagonalLoss.Sigmas(np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0]))
        first_pose = Pose3()
    
    def addNewKeyFrame(self, frame_id, pose: Pose3):
        if initialization:
            graph.add(PriorFactorPose3(frame_id, pose, prior_loss))
            initialization = False
        else:
            prev_id = frame_id - 1
            odom = prev_pose.between(pose)
            odom_noise = DiagonalLoss.Sigmas(np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1]))
            graph.add(BetweenFactorPose3(prev_id, frame_id, odom, odom_noise))
        
        initial_estimate.insert(frame_id, pose)
        prev_pose = pose