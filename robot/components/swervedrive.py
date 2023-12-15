from magicbot import magiccomponent
from components import swervemodule
import math

class SwerveDrive:
    # These modules will be injected from robot.py
    frontRightModule: swervemodule.SwerveModule
    frontLeftModule: swervemodule.SwerveModule
    rearRightModule: swervemodule.SwerveModule
    rearLeftModule: swervemodule.SwerveModule

    xy_multiplier = 1.0
    rotation_multiplier = 1.0

    def setup(self):
        self.modules = {
            'front_left': self.frontLeftModule,
            'front_right': self.frontRightModule,
            'rear_left': self.rearLeftModule,
            'rear_right': self.rearRightModule
        }

        # Set all inputs to zero
        self.requested_vectors = {
            'fwd': 0,
            'strafe': 0,
            'rcw': 0
        }

        self.requested_angles = {
            'front_left': 0,
            'front_right': 0,
            'rear_left': 0,
            'rear_right': 0
        }

        self.requested_speeds = {
            'front_left': 0,
            'front_right': 0,
            'rear_left': 0,
            'rear_right': 0
        }

        # Variables that allow enabling and disabling of features in code
        self.squared_inputs = True
        self.threshold_input_vectors = True

        self.width = (34.5 / 12) / 2
        self.length = (38 / 12) / 2

    @property
    def chassis_dimension(self):
        return(self.width, self.length)
    
    @staticmethod
    def square_inputs(input):
        return(math.copysign(input * input, input))
    
    @staticmethod
    def normalize(data):
        maxMagnitude = max(abs(x) for x in data)

        if maxMagnitude > 1.0:
            for i in range(len(data)):
                data[i] = data[i] / maxMagnitude
        
        return data
    
    @staticmethod
    def normalizeDictionary(data):
        """
        Get the maximum value in the data. If the max is more than 1,
        divide each data by that max.
        :param data: The dictionary with the data to be normalized
        :returns: The normalized dictionary with the data
        """
        maxMagnitude = max(abs(x) for x in data.values())

        if maxMagnitude > 1.0:
            for i in data:
                data[i] = data[i] / maxMagnitude
        
        return data
    
    def flush(self):
        self.requested_vectors = {
            'fwd': 0,
            'strafe': 0,
            'rcw': 0
        }

        self.requested_angles = {
            'front_left': 0,
            'front_right': 0,
            'rear_left': 0,
            'rear_right': 0
        }

        self.requested_speeds = {
            'front_left': 0,
            'front_right': 0,
            'rear_left': 0,
            'rear_right': 0
        }

        for modules in self.modules.values():
            modules.flush()
    
    def set_raw_fwd(self, fwd):
        """
        Sets the raw fwd value to prevent it from being passed through any filters

        :param fwd: A value from -1 to 1
        """
        self.requested_vectors['fwd'] = fwd

    def set_raw_strafe(self, strafe):
        """
        Sets the raw strafe value to prevent it from being passed through any filters

        :param strafe: A value from -1 to 1
        """
        self.requested_vectors['strafe'] = strafe
    
    def set_raw_rcw(self, rcw):
        """
        Sets the raw rcw value to prevent it from being passed through any filters

        :param rcw: A value from -1 to 1
        """
        self.requested_vectors['rcw'] = rcw

    def set_fwd(self, fwd):
        """
        Individually sets the fwd value. (passed through filters)

        :param fwd: A value from -1 to 1
        """
        if self.squared_inputs:
            fwd = self.square_inputs(fwd)

        fwd *= self.xy_multiplier

        self.requested_vectors['fwd'] = fwd

    def set_strafe(self, strafe):
        """
        Individually sets the strafe value. (passed through filters)

        :param strafe: A value from -1 to 1
        """
        if self.squared_inputs:
            strafe = self.square_inputs(strafe)

        strafe *= self.xy_multiplier

        self.requested_vectors['strafe'] = strafe

    def set_rcw(self, rcw):
        """
        Individually sets the rcw value. (passed through filters)

        :param rcw: A value from -1 to 1
        """
        if self.squared_inputs:
            rcw = self.square_inputs(rcw)

        rcw *= self.rotation_multiplier

        self.requested_vectors['rcw'] = rcw
    
    def move(self, fwd, strafe, rcw):
        """
        Calulates the speed and angle for each wheel given the requested movement

        Positive fwd value = Forward robot movement\n
        Negative fwd value = Backward robot movement\n
        Positive strafe value = Left robot movement\n
        Negative strafe value = Right robot movement

        :param fwd: the requested movement in the Y direction 2D plane
        :param strafe: the requested movement in the X direction of the 2D plane
        :param rcw: the requestest magnatude of the rotational vector of a 2D plane
        """
        self.set_fwd(fwd)
        self.set_strafe(strafe)
        self.set_rcw(rcw)

    def calculate_vectors(self):
        """
        Calculate the requested speed and angle of each modules from self._requested_vectors and store them in
        self._requested_speeds and self._requested_angles dictionaries.
        """
        self.requested_vectors['fwd'], self.requested_vectors['strafe'], self.requested_vectors['rcw'] = self.normalize([self.requested_vectors['fwd'], self.requested_vectors['strafe'], self.requested_vectors['rcw']])

        ratio = math.hypot(self.length, self.width)

        frontX = self.requested_vectors['strafe'] - (self.requested_vectors['rcw'] * (self.length / ratio))
        rearX = self.requested_vectors['strafe'] + (self.requested_vectors['rcw'] * (self.length / ratio))
        leftY = self.requested_vectors['fwd'] - (self.requested_vectors['rcw'] * (self.width / ratio))
        rightY = self.requested_vectors['fwd'] + (self.requested_vectors['rcw'] * (self.width / ratio))

        frontLeft_speed = math.hypot(frontX, rightY)
        frontLeft_angle = math.degrees(math.atan2(frontX, rightY))

        frontRight_speed = math.hypot(frontX, leftY)
        frontRight_angle = math.degrees(math.atan2(frontX, leftY))

        rearLeft_speed = math.hypot(rearX, rightY)
        rearLeft_angle = math.degrees(math.atan2(rearX, rightY))

        rearRight_speed = math.hypot(rearX, leftY)
        rearRight_angle = math.degrees(math.atan2(rearX, leftY))

        self.requested_speeds['front_left'] = frontLeft_speed
        self.requested_speeds['front_right'] = frontRight_speed
        self.requested_speeds['rear_left'] = rearLeft_speed
        self.requested_speeds['rear_right'] = rearRight_speed

        self.requested_angles['front_left'] = frontLeft_angle
        self.requested_angles['front_right'] = frontRight_angle
        self.requested_angles['rear_left'] = rearLeft_angle
        self.requested_angles['rear_right'] = rearRight_angle

        self.requested_speeds = self.normalizeDictionary(self.requested_speeds)

        # Zero request vectors for saftey reasons
        self.requested_vectors['fwd'] = 0.0
        self.requested_vectors['strafe'] = 0.0
        self.requested_vectors['rcw'] = 0.0
    
    def execute(self):
        self.calculate_vectors()

        for i in self.modules:
            self.modules[i].move(self.requested_speeds[i], self.requested_angles[i])
        
        self.requested_speeds = dict.fromkeys(self.requested_speeds, 0)
        
        for i in self.modules:
            self.modules[i].execute()
