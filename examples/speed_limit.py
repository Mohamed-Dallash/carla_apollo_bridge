import jpype
# import jpype.imports

import time
import math
from jvm_setup import JVMSetup

class SpeedLimit():
    def __init__(self):
        JVMSetup.initializeJVM()
        # jpype.addClassPath('tpdejavu.jar')
        # jpype.addClassPath('CollisionRate.jar')
        # jpype.addClassPath('SafeDistance.jar')
        # jpype.addClassPath('SpeedLimit.jar')
        # jpype.startJVM(jpype.getDefaultJVMPath())
        
        self.monitor = jpype.JClass("speed_limit.TraceMonitor") 
    
    def send_data(self, ego_actor):
        jpype.attachThreadToJVM()
        # data = [["VehicleSpeedChanged", self.uid, ego_speed, self.carla_actor.get_speed_limit()]]
        my_vehicle = ego_actor
        my_velocity = my_vehicle.get_velocity()
        my_speed = 3.6 * math.sqrt(my_velocity.x**2 + my_velocity.y**2 + my_velocity.z**2)
        message = "SpeedChanged,{},{}".format(int(my_speed),int(my_vehicle.get_speed_limit()))
        print(message)
        self.monitor.eval(message)  
