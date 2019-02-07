import math
#import numpy as np

from pyfrc.physics import motor_cfgs, tankmodel
from pyfrc.physics.units import units

import sim.simComms as simComms

class PhysicsEngine(object):
    def __init__(self, controller):
        self.controller = controller
        self.position = 0

        self.DistPerPulseL = 6/12 * math.pi / 256
        self.DistPerPulseR = 6/12 * math.pi / 256

        # Change these parameters to fit your robot!
        self.drivetrain = tankmodel.TankModel.theory(
            motor_cfgs.MOTOR_CFG_MINI_CIM,  # motor configuration
            140*units.lbs,                  # robot mass
            9.44,                           # drivetrain gear ratio
            3,                              # motors per side
            (28/12)*units.feet,             # robot wheelbase
            (35/12)*units.feet,             # robot width
            (35/12)*units.feet,             # robot length
            (6/12)*units.feet               # wheel diameter
        )

        self.distance = [0.0,0.0]

        self.controller.add_device_gyro_channel('navxmxp_spi_4_angle')

        self.deadZone=0.10

    def update_sim(self, hal_data, now, timeDiff):
        # Simulate the drivetrain
        can = hal_data['CAN']

        if not 10 in can or not 20 in can:
            return

        left = -can[10]['value']
        right = can[20]['value']

        if(abs(left)<self.deadZone): left = 0
        if(abs(right)<self.deadZone): right = 0

        x,y,angle = self.drivetrain.get_distance(left, right, timeDiff)
        self.controller.distance_drive(x, y, angle)

        if(simComms.getEncoders()==True):
            self.distance = [0,0]
            simComms.resetEncodersSim()
        else:
            self.distance[0] += self.drivetrain.l_velocity*timeDiff
            self.distance[1] += self.drivetrain.r_velocity*timeDiff

        hal_data['encoder'][0]['count'] = int(self.distance[0]/self.DistPerPulseL)
        hal_data['encoder'][1]['count'] = int(self.distance[1]/self.DistPerPulseR)
