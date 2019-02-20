import math
#import numpy as np

from pyfrc.physics import motor_cfgs, tankmodel
from pyfrc.physics.units import units

import sim.simComms as simComms
import robotmap

class CountUpdater(object):
  def __init__(self, idx):
    self.idx = idx
    self.cntIdx = -1
    self.lastVal = False

  def update(self, hal_data):
    curVal = hal_data['dio'][self.idx]['value']
    if curVal and curVal != self.lastVal:
      if self.cntIdx == -1:
        for i in range(0, len(hal_data['counter'])):
          if self.idx == hal_data['counter'][i]['up_source_channel']:
            self.cntIdx = i
            break

      if self.cntIdx != -1:
        cnt = 1 + hal_data['counter'][self.cntIdx]['count']
        hal_data['counter'][self.cntIdx]['count'] = cnt
        
    self.lastVal = curVal

class PhysicsEngine(object):
    def __init__(self, controller):
        self.controller = controller
        self.position = 0
        self.counters = (
          CountUpdater(robotmap.kDioClimbBackBot),
          CountUpdater(robotmap.kDioClimbBackTop),
          CountUpdater(robotmap.kDioClimbFrontBot),
          CountUpdater(robotmap.kDioClimbFrontTop)
        )

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

    # def updateAnalog(self, hal_data, idx: int, volts: float):
    #   keys = ("analog_in", "analog_trigger")
    #   attrs = ("value", "voltage", "avg_voltage")

    #   for key in keys:
    #     for attr in attrs:
    #       hal_data[key][idx][attr] = volts
      
    def update_sim(self, hal_data, now, timeDiff):
        # Simulate the drivetrain
        can = hal_data['CAN']

        left = 0
        right = 0

        if 10 in can:
          left = -can[10]['value']
        if 20 in can:
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

        # Update counters based on rising edge changes in DIO states
        for counter in self.counters:
          counter.update(hal_data)

        # Simulate climber sensors
        # hal_data['dio'][robotmap.kDioClimbBackBot]["value"] = True
        # hal_data['dio'][robotmap.kDioClimbBackTop]["value"] = False
        # hal_data['dio'][robotmap.kDioClimbFrontBot]["value"] = True
        # hal_data['dio'][robotmap.kDioClimbFrontTop]["value"] = False

        #self.updateAnalog(hal_data, robotmap.kAiClimbGroundFront, 3.1)
        #self.updateAnalog(hal_data, robotmap.kAiClimbGroundBack, 3.2)
