import math

import wpilib
import ctre

from networktables import NetworkTables
from wpimath.controller import PIDController
from collections import namedtuple

ModuleConfig = namedtuple('ModuleConfig', ['sd_prefix', 'reset', 'allow_reverse'])

class SwerveModule:
    drive_motor: ctre.WPI_TalonSRX
    turn_motor: ctre.WPI_TalonSRX
        
    encoder: ctre.WPI_TalonSRX

    cfg: ModuleConfig

    def setup(self):
        self.reset = self.cfg.reset or 0
        self.allow_reverse = self.cfg.allow_reverse or True

        self.requested_ticks = 0
        self.requested_speed = 0

        self.pid_controller = PIDController(0.000083, 0, 0) #NOTE: TEST VALUES ONLY <NONFINAL>
        # PIDController(Kp, Ki, Kd, period: default (0.02))
        # Kp, Ki, and Kd set to varying 1e-5 -> 1e-4
        self.pid_controller.enableContinuousInput(0.0, 4096)
        self.pid_controller.setTolerance(0.8, 0.8)

    def flush(self):
        self.requested_ticks = self.reset
        self.requested_speed = 0
        self.pid_controller.reset()
    
    @staticmethod
    def ticks_to_deg(ticks):
        return(ticks / 4096) * 360
    
    @staticmethod
    def deg_to_ticks(deg):
        return((deg/360) * 4096) 
    
    def get_deg(self):
        return(self.encoder.getSelectedSensorPosition() - self.reset)
    
    def set_deg(self, value):
        self.requested_ticks = ((self.deg_to_ticks(value) + self.reset) % 4096)

    def move(self, speed, deg):
        deg %= 360
        if self.allow_reverse:
            if abs(deg - self.ticks_to_deg(self.get_deg()) > 90):
                speed *= -1
                deg += 180
                deg %- 360
        
        self.requested_speed = speed
        self.set_deg(deg)
    
    def execute(self):
        # error = self.pid_controller.calculate(self.encoder.getSelectedSensorPosition(), self.requested_ticks)

        # output = 0

        if not self.pid_controller.atSetpoint():
            # output = max(min(error, 1), -1) 
            output = self.pid_controller.calculate(self.encoder.getSelectedSensorPosition(), self.requested_ticks)
        elif self.pid_controller.atSetpoint():
            output = 0
        
        self.turn_motor.set(output)
        self.drive_motor.set(self.requested_speed)