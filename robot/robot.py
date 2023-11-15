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

    frontLeftModule_cfg = ModuleConfig(sd_prefix='FrontLeft_Module', zero=2.97, inverted=True, allow_reverse=True)
    frontRightModule_cfg = ModuleConfig(sd_prefix='FrontRight_Module', zero=2.69, inverted=False, allow_reverse=True)
    rearLeftModule_cfg = ModuleConfig(sd_prefix='RearLeft_Module', zero=0.18, inverted=True, allow_reverse=True)
    rearRightModule_cfg = ModuleConfig(sd_prefix='RearRight_Module', zero=4.76, inverted=False, allow_reverse=True)

    def createObjects(self):
        
        self.controller = wpilib.XboxController(0)

        self.frontLeftModule_driveMotor = ctre.WPI_TalonSRX(1)
        self.frontRightModule_driveMotor = ctre.WPI_TalonSRX(2)
        self.rearLeftModule_driveMotor = ctre.WPI_TalonSRX(3)
        self.rearRightModule_driveMotor = ctre.WPI_TalonSRX(4)

        self.frontLeftModule_rotateMotor = ctre.WPI_TalonSRX(5)
        self.frontRightModule_rotateMotor = ctre.WPI_TalonSRX(6)
        self.rearLeftModule_rotateMotor = ctre.WPI_TalonSRX(7)
        self.rearRightModule_rotateMotor = ctre.WPI_TalonSRX(8)

        self.frontLeftModule_encoder = wpilib.AnalogInput(0)
        self.frontRightModule_encoder = wpilib.AnalogInput(1)
        self.rearLeftModule_encoder = wpilib.AnalogInput(2)
        self.rearRightModule_encoder = wpilib.AnalogInput(3)

    def autonomousInit(self):
        self.drive.flush()
    
    def teleopInit(self):
        self.drive.flush()
    
    def move(self, x, y, rcw):
        self.drive.move(x, y, rcw)

    def teleopPeriodic(self):
        self.move(self.controller.getLeftY, self.controller.getLeftX, self.controller.getRightX)



if __name__ == "__main__":
    wpilib.run(MyRobot)