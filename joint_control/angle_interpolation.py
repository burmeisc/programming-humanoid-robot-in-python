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
        # save the time when angle_interpolation is first called and keyframe motion is started
        self.kf_start_time = -1

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}

        # YOUR CODE HERE
        # if start time is 0 set start time for the first time.
        if self.kf_start_time == -1:
            self.kf_start_time = perception.time

        # substract the start time so our current time point is in the keyframe time
        kf_current_time = perception.time - self.kf_start_time

        names = keyframes[0]
        # iterate over rows in kf.times as one row represents one joint
        for joint, time, keys in zip(names, keyframes[1], keyframes[2]):

            if not joint in perception.joint:
                break

            j = -1
            for index, t in enumerate(time):
                if (t > kf_current_time):
                    j = index
                    break

            if j == -1:
                target_joints[joint] = 0
                continue

            endHandleDTime = keys[j][1][1]
            endHandleDAngle = keys[j][1][2]

            bezierEnd = (time[j], keys[j][0])
            bezierEndHandle = np.add(bezierEnd, (endHandleDTime, endHandleDAngle))

            if (j > 0):
                startHandleDTime = keys[j - 1][2][1]
                startHandleDAngle = keys[j - 1][2][2]
                bezierStart = (time[j - 1], keys[j - 1][0])
                bezierStartHandle = np.add(bezierStart, (startHandleDTime, startHandleDAngle))
            else:
                startHandleDTime = - endHandleDTime
                startHandleDAngle = 0
                bezierStart = (0, perception.joint[joint])
                bezierStartHandle = np.add(bezierStart, (startHandleDTime, startHandleDAngle))

            root = self.get_root(bezierStart[0],
                                 bezierStartHandle[0],
                                 bezierEndHandle[0],
                                 bezierEnd[0],
                                 kf_current_time)

            target_angle = self.eval_cubic(bezierStart[1],
                                           bezierStartHandle[1],
                                           bezierEndHandle[1],
                                           bezierEnd[1],
                                           root)

            #print joint + ':' + str(target_angle)
            target_joints[joint] = target_angle

        return target_joints

    # solve cubic with x valuesfor given points in row
    def get_root(self, x0, x1, x2, x3, t):
        roots = np.roots([-x0 + 3 * x1 - 3 * x2 + x3, 3 * x0 - 6 * x1 + 3 * x2, -3 * x0 + 3 * x1, x0 - t])
        r = 0
        for root in roots:
            if np.isreal(root) and np.real(root) >= 0 <= 1:
                r = np.real(root)
        return r

    def eval_cubic(self, y0, y1, y2, y3, i):

        return np.polyval([-y0 + 3 * y1 - 3 * y2 + y3, 3 * y0 - 6 * y1 + 3 * y2, -3 * y0 + 3 * y1, y0], i)


if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
