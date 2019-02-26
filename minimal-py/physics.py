import math

from pyfrc.physics import motor_cfgs, tankmodel
from pyfrc.physics.units import units

class PhysicsEngine(object):
  def __init__(self, controller):
    self.controller = controller
    self.position = 0
    self.distance = [0.0,0.0]

    # Change these parameters to fit your robot!
    self.drivetrain = tankmodel.TankModel.theory(
      motor_cfgs.MOTOR_CFG_MINI_CIM,  # motor configuration
      140*units.lbs,                  # robot mass
      40, #9.44,                           # drivetrain gear ratio
      1,                              # motors per side
      (28/12)*units.feet,             # robot wheelbase
      (35/12)*units.feet,             # robot width
      (35/12)*units.feet,             # robot length
      (6/12)*units.feet               # wheel diameter
    )

  def update_sim(self, hal_data, now, timeDiff):
    # Simulate the drivetrain
    pwm = hal_data['pwm']

    if len(pwm) < 2:
      return

    left = -pwm[0]['value']
    right = pwm[1]['value']

    x,y,angle = self.drivetrain.get_distance(left, right, timeDiff)
    self.controller.distance_drive(x, y, angle)
