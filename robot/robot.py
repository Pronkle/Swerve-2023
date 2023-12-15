import wpilib

import ctre

from magicbot import MagicRobot

from networktables import NetworkTables
from networktables.util import ntproperty

from components import swervedrive, swervemodule

from collections import namedtuple

ModuleConfig = swervemodule.ModuleConfig

# Download and install stuff on the RoboRIO after imaging
'''
py -3 -m robotpy_installer download-python
py -3 -m robotpy_installer install-python
py -3 -m robotpy_installer download robotpy
py -3 -m robotpy_installer install robotpy
py -3 -m robotpy_installer download robotpy[ctre]
py -3 -m robotpy_installer install robotpy[ctre]
py -3 -m robotpy_installer download robotpy[rev]
py -3 -m robotpy_installer install robotpy[rev]
py -3 -m robotpy_installer download pynetworktables
py -3 -m robotpy_installer install pynetworktables
py -3 -m pip install -U robotpy[ctre]
py -3 -m pip install robotpy[ctre]
'''

# Push code to RoboRIO (only after imaging)
'''
python robot/robot.py deploy --skip-tests
py robot/robot.py deploy --skip-tests --no-version-check
'''


class MyRobot(MagicRobot):

    drive: swervedrive.SwerveDrive

    frontLeftModule: swervemodule.SwerveModule
    frontRightModule: swervemodule.SwerveModule
    rearLeftModule: swervemodule.SwerveModule
    rearRightModule: swervemodule.SwerveModule

    frontLeftModule_cfg = ModuleConfig(sd_prefix = 'FrontLeft_Module', reset = 0, allow_reverse=True)
    frontRightModule_cfg = ModuleConfig(sd_prefix = 'FrontRight_Module', reset = -527.0, allow_reverse=True)
    rearLeftModule_cfg = ModuleConfig(sd_prefix = 'RearLeft_Module', reset = 0, allow_reverse=True)
    rearRightModule_cfg = ModuleConfig(sd_prefix = 'RearRight_Module', reset = 0, allow_reverse=True)


    def createObjects(self):
        self.controller = wpilib.XboxController(0)

        self.frontLeftModule_drive_motor = ctre.WPI_TalonSRX(6)
        self.frontRightModule_drive_motor = ctre.WPI_TalonSRX(8)
        self.rearLeftModule_drive_motor = ctre.WPI_TalonSRX(3)
        self.rearRightModule_drive_motor = ctre.WPI_TalonSRX(1)

        self.frontLeftModule_turn_motor = ctre.WPI_TalonSRX(5)
        self.frontRightModule_turn_motor = ctre.WPI_TalonSRX(7)
        self.rearLeftModule_turn_motor = ctre.WPI_TalonSRX(4)
        self.rearRightModule_turn_motor = ctre.WPI_TalonSRX(2)

        self.frontLeftModule_encoder = self.frontLeftModule_turn_motor
        self.frontRightModule_encoder = self.frontRightModule_turn_motor
        self.rearLeftModule_encoder = self.rearLeftModule_turn_motor
        self.rearRightModule_encoder = self.rearRightModule_turn_motor

    def autonomousInit(self):
        self.drive.flush()
    
    def teleopInit(self):
        self.drive.flush()
        self.drive.squared_inputs = True
    
    def move(self, x, y, rcw):
        self.drive.move(x, y, rcw)

    def teleopPeriodic(self):
        self.move(self.controller.getLeftY(), self.controller.getLeftX(), self.controller.getRightX())

        print(self.frontLeftModule_encoder.getSelectedSensorPosition())
        print(self.frontRightModule_encoder.getSelectedSensorPosition())
        print(self.rearLeftModule_encoder.getSelectedSensorPosition())
        print(self.rearRightModule_encoder.getSelectedSensorPosition())
        print('_________________________________________________________________')


if __name__ == "__main__":
    wpilib.run(MyRobot)