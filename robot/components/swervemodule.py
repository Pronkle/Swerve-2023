import math

import wpilib
import ctre

from networktables import NetworkTables
from wpimath.controller import PIDController
from collections import namedtuple

ModuleConfig = namedtuple('ModuleConfig', ['sd_prefix', 'zero', 'inverted', 'allow_reverse'])

class SwerveModule:
    driveMotor: ctre.WPI_TalonSRX
    rotateMotor: ctre.WPI_TalonSRX
        
    encoder: wpilib.AnalogInput

    cfg: ModuleConfig

    def setup(self):
        self.encoder_zero = self.cfg.zero or 0
        self.inverted = self.cfg.inverted or False
        self.allow_reverse = self.cfg.allow_reverse or True

        self.driveMotor.setInverted(self.inverted)

        self.requested_voltage = 0
        self.requested_speed = 0

        self.pid_controller = PIDController(1.5, 0.0, 0.0)
        self.pid_controller.enableContinuousInput(0.0, 5.0) # Will set the 0 and 5 as the same point
        self.pid_controller.setTolerance(0.05, 0.05) # Tolerance where the PID will be accpeted aligned
        
        def get_voltage(self):
            return self.encoder.getAverageVoltage() - self.encoder_zero
        
        def flush(self):
            
            self.requested_voltage = self.encoder_zero
            self.requested_speed = 0
            self.pid_controller.reset
        
        @staticmethod
        def voltage_to_degrees(voltage):
            deg = (voltage / 5) * 360

            if deg < 0:
                deg += 360
            
            return deg
        
        @staticmethod
        def degree_to_voltage(degree):
            return (degree / 360) * 5
        
        def set_deg(self, value):
            self.requested_voltage = ((self.degree_to_voltage(value) + self.encoder_zero) % 5)

        def move(self, speed, deg):
            deg %= 360

            if self.allow_reverse:

                if abs(deg - self.voltage_to_degree(self.get_voltage())) > 90:
                    speed *= -1
                    deg += 180
                    deg %= 360
                
            self.requested_speed = speed
            self.set_deg(deg)
        
        def execute(self):
            error = self.pid_controller.calculate(self.encoder.getVoltage(), self.requested_voltage)

            output = 0

            if not self.pid_controller.atSetpoint():
                output = max(min(error,1), -1)

            self.rotateMotor.set(output)
            self.driveMotor.set(self.requested_speed)
            

        
        