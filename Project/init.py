import os
import sys
from PIL import Image
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'software_installation'))
from spark_agent import SparkAgent

class OnePunchMan(SparkAgent):

    def think(self, perception):

        dataStream = perception.see[0]
        x = get_ball_pos(dataStream)
        print(x)
        #print(dataStream)

        return super(OnePunchMan, self).think(perception)



def get_ball_pos(stream):
    x = stream.get('B')
    return x



if '__main__' == __name__:
    agent = OnePunchMan()
    agent.run()
