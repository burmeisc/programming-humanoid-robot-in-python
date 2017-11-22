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
import numpy as np


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
        #save the time when angle_interpolation is first called and keyframe motion is started
        self.kf_start_time = -1
        #current time in keyframe movement
        self.kf_current_time = 0

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # YOUR CODE HERE

        names = keyframes[0]

        #set keys for target_joints to joint names in keyframe (so we only calculate angles for listed joints)
        target_joints = {k: 0 for k in names}

        #if start time is 0 set start time for the first time.
        if self.kf_start_time == -1:
            self.kf_start_time = perception.time

        #substract the start time so our current time point is in the keyframe time
        self.kf_current_time = perception.time - self.kf_start_time

        #TO DO: check somehow if time is still inside keyframe time

        #for all joints get closest timepoint in keyframes
        bezier_points = []          #each row represents one joint, saves 4 points (x,y) for interpolation
        #iterate over rows in kf.times as one row represents one joint
        for joint,row,keys in zip(names, keyframes[1],keyframes[2]):
            temp= [time-self.kf_current_time for time in row]
            points = [0,0,0,0]
            closest_point = np.min(temp)
            index_cp = np.argmin(temp)

            if not joint in perception.joint:
                break
            #if closes point is first time point get current angles from perception
            if index_cp == 0:

                points[0] = (0.0,perception.joint[joint])
                points[3] = (closest_point,keys[index_cp][0])

                # get handles, handle2 for point 1, handle1 for point 2
                handle1 = keys[index_cp][1]
                dTime = - handle1[1]
                dAngle = 0
                points[1] = (points[0][0] + dTime, points[0][1] + dAngle)

                dTime = handle1[1]
                dAngle = handle1[2]
                points[2] = (points[3][0] + dTime, points[3][1] + dAngle)
            elif closest_point-self.kf_current_time<0:
                #save first point [x,y] x=cp time, y=angle
                points[0] = (closest_point,keys[index_cp][0])
                #save second point, x=cp time + 1, y=angle
                points[3] = (row[index_cp+1],keys[index_cp+1][0])
                #get handles, handle2 for point 1, handle1 for point 2
                handle2 = keys[index_cp][2]
                dTime = handle2[1]
                dAngle = handle2[2]
                points[1] = (points[0][0]+dTime,points[0][1]+dAngle)

                handle1 =keys[index_cp+1][1]
                dTime = handle1[1]
                dAngle = handle1[2]
                points[2] = (points[3][0]+dTime, points[3][1]+dAngle)
            elif closest_point-self.kf_current_time>0:
                points[0] = (row[index_cp-1],keys[index_cp-1][0])
                points[3] = (closest_point,keys[index_cp][0])
                # get handles, handle2 for point 1, handle1 for point 2
                handle2 = keys[index_cp][2]
                dTime = handle1[1]
                dAngle = handle1[2]
                points[1] = (points[0][0] + dTime, points[0][1] + dAngle)

                handle1 = keys[index_cp + 1][1]
                dTime = handle2[1]
                dAngle = handle2[2]
                points[2] = (points[3][0] + dTime, points[3][1] + dAngle)


            bezier_points.append(points)

            #find i with x equation
            for (joint,row) in zip(names,bezier_points):
                i = self.solve_cubic_x(row,self.kf_current_time)
                target_angle = self.eval_cubic(row,i)
                target_joints[joint] = target_angle

        return target_joints

    #solve cubic with x valuesfor given points in row
    def solve_cubic_x(self,row, t):
        x0 = row[0][0]
        x1= row[1][0]
        x2 = row[2][0]
        x3 = row[3][0]
        roots = np.roots([-x0 + 3 * x1 - 3 * x2 + x3, 3 * x0 - 6 * x1 + 3 * x2, -3 * x0 + 3 * x1, x0 - t])
        r=0
        for root in roots:
            if np.isreal(root) and 0 <= np.real(root)<=1:
                r = root
        return r

    def eval_cubic(self,row,i):
        y0 = row[0][1]
        y1 = row[1][1]
        y2 = row[2][1]
        y3 = row[3][1]
        return np.polyval([-y0+3*y1-3*y2+y3,3*y0-6*y1+3*y2,-3*y0+3*y1,y0],i)



if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
