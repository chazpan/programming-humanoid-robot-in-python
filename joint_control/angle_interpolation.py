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

import numpy as np
import time
from pid import PIDAgent, INVERSED_JOINTS
from keyframes import hello,wipe_forehead,rightBackToStand,rightBellyToStand,leftBackToStand,nicken,leftBellyToStand

class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = hello()

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)

        action = super(AngleInterpolationAgent, self).think(perception)

        return  action

    def generate_bezier(self, p):

        n = len(p)

        c = self.pascal(n-1)

        def bezier(t):
            tpowers = (t**i for i in range(n))
            upowers = reversed([(1.0-t)**i for i in range(n)])
            coeff = [bino*t_1*t for bino, t_1, t in zip(c, tpowers, upowers)]
            t = zip(*p)
            return tuple(sum([coef*p for coef, p in zip(coeff, ps)]) for ps in t)
        return bezier

    def pascal(self, n):
        v, num = 1, n
        row = []
        row.append(1)
        for i in range(1,n//2+1):
            v = v*num
            v = v/i
            row.append(v)
            num = num - 1

        if n & 1 == 0:
            row.extend(reversed(row[:-1]))
        else:
            row.extend(reversed(row))
        return row


    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # YOUR CODE HERE
        x,y,z = keyframes
        i=0

        t = self.perception.time % 20

        for i in range(0,len(x)):
            arr = []
            name = x[i]
            times = y[i]
            position = z[i]
            if name not in self.joint_names:
                continue
            if(self.perception.time %20 < times[0]  or self.perception.time % 20 > times[len(times)-1]):
                target_joints[name] = position[len(times)-1][0]+position[len(times)-1][2][2]
                continue

            indexTime = 0
            for k in range(0,len(times)-1):
                if t >= times[k]:
                    indexTime = k
                    continue
                else:
                    break
            p = []

            point = position[indexTime]
            p.append((times[indexTime],point[0]))
            p.append((times[indexTime]+point[2][1],point[0]+point[1][2]))
            point = position[indexTime+1]
            p.append((times[indexTime+1]+point[1][1],point[0]+point[2][2]))
            p.append((times[indexTime+1],point[0]))

            bezier = self.generate_bezier(p)


            scaleToOne = np.abs(t-times[indexTime])/ (times[indexTime+1] - times[indexTime])

            a,b =  bezier(scaleToOne)

            # For Simulator
            if(name in INVERSED_JOINTS):
                b = b* (-1)
            target_joints[name] = b






        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = rightBackToStand()  # CHANGE DIFFERENT KEYFRAMES
    time.sleep(10)
    agent.run()

