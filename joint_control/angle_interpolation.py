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
from keyframes import hello, leftBackToStand, wipe_forehead, rightBellyToStand


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
        self.starting_time = self.perception.time


    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}

        # YOUR CODE HERE
        names = keyframes[0]
        times = keyframes[1]
        keys = keyframes[2]
        time = self.perception.time - self.starting_time



        #find key with fitting time
        for i in range(len(names)):
                for j in range(len(times[i])):
                    if time < times[i][j]:
                        time_fracture = time/times[i][j]
                        target_joints[names[i]] = AngleInterpolationAgent.interpolate(self, time_fracture, i, j, keys)
                        break
                    else:
                        if j == len(times[i])-1:
                            if names[i] in perception.joint:
                                target_joints[names[i]] = self.perception.joint[names[i]]
                            else:
                                break

        return target_joints

    def interpolate(self, time_fracture, index_i, index_j, keys):

        x_0 = keys[index_i][index_j][0]
        x_1 = keys[index_i][index_j][1][2] + x_0
        x_2 = keys[index_i][0][0]
        x_3 = keys[index_i][index_j][2][2] + x_2


        y_0 = (1 - time_fracture)**3
        y_1 = 3 * (1 - time_fracture)**2
        y_2 = 3 * (1 - time_fracture)

        bezier = (y_0 * x_0) + (y_1 * x_1 * time_fracture) + (y_2 * x_3 * (time_fracture**2)) + (x_2 * (time_fracture**3))

        return bezier


if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = rightBellyToStand()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
