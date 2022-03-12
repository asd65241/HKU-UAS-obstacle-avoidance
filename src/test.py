# Tests your package

__author__ = "you"
__email__ = "your email"

from obstacle_avoidance import Mission
import json
from matplotlib import path
import time

missionFile = "mission.plan"
f = open(missionFile)
data = json.load(f)
f.close()


# test code
mission1 = Mission(data)
new_path = mission1.obs_avoid(100, 10)

#print(new_path)

time.sleep(5)

#mission1.export()


#print(mission1)
#print(data)
