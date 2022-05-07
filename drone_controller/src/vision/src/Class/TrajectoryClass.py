from trajectory_msgs.msg import MultiDOFJointTrajectory 

class Trajectory: 

    TrajectoryPoints = []

    def GetTrajectoryMessage(self):
        msgTrajectory = MultiDOFJointTrajectory()
        msgTrajectory.points = self.TrajectoryPoints

        return msgTrajectory
    
    def AddPointToTrajectory(self,point): 
        global TrajectoryPoints
        self.TrajectoryPoints.append(point.GetTrajectoryPoint())