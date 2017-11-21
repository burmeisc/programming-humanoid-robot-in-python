'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import hello


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
        #save the time when angle_interpolation is first called and keyframe motion is started
        self.kf_start_time = 0
        #current time in keyframe movement
        self.kf_current_time = 0

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # YOUR CODE HERE

        names = keyframes.names
        
        #set keys for target_joints to joint names in keyframe (so we only calculate angles for listed joints)
        target_joints = {k: 0 for k in keyframes.names}

        if self.kf_start_time == 0:
            self.kf_start_time = perception.time

        self.kf_current_time = perception.time - self.kf_start_time

        #check somehow if time is still inside keyframe time

        #for all joints get closest timepoint in keyframes
        closest_point = {k:0 for k in target_joints.keys()}
        i=0
        #iterate over rows in kf.times as one row represents one joint
        for (joint,row) in zip(target_joints.keys(),keyframes.times[i][:]):
            temp= [times-self.kf_current_time for times in row]
            closest_point[joint] = min(temp)
            i+=1

        #get second point so that current_time is in betweeen those points
        interp_points = {k:(0,0) for k in target_joints.keys()}
        for (key,value) in closest_point:
            if value-self.kf_current_time <0:
                interp_points[key] = (keyframes[])
        #interpolate Bezier curve for 2 closes points
        #get Handle2 from first point and Handle1 from second point as control points
        #interpolte and return new angle

        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
