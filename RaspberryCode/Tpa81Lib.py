import time
import math
import  smbus #smbus2 as smbus 


class Tpa81:
    """
    This is a library made to make thermal imaging easier on the
    Raspberry Pi, using the TPA81 sensor (www.robot-electronics.co.uk/htm/tpa81tech.htm).
    It allows the end-user to pass through the use of I2C and the calculations needed to
    use the servo. It started as a school project, so the PEP may not be respected entirely.
    I may brake some of the code style conventions too. Temperature is in degrees Celsius and
    Angles are in Degrees
    """

    # In this list will be kept the data sent by the sensor
    tpa81data = []

    # On the newer revisions of the RPI
    bus = smbus.SMBus(1)

    #Normally, if you just got the sensor, the adress is still 0x68
    DEVICE_ADDRESS = 0x68

    #The ambient temperature is stored in a special register
    #ambient_temp = bus.read_byte_data(DEVICE_ADDRESS, 1)

    #Displays the revision of the embedded sensor software
    #SOFT_REV = bus.read_byte_data(DEVICE_ADDRESS, 0)

    def __init__(self,address):
        try:
            #print (self.SOFT_REV)
            self.DEVICE_ADDRESS = address
        except ValueError:
            print("Can't access the TPA81 module! Check if you enabled I2C in raspi-config")
        

    def getThermalData(self):
        """
        Private function that captures the data from the thermopile array
        10-2 = 8, that makes the 8 registers where the temperature data is stored
        :return: list of temperature values
        """
        self.tpa81data.clear()
        for i in range(2, 10):
            self.tpa81temp = self.bus.read_byte_data(self.DEVICE_ADDRESS, i)
            #print((self.tpa81temp)%128)
            self.tpa81data.append((self.tpa81temp)%128)
        return self.tpa81data

    def getPanoramicData(self, angle):
        """
            Starts a panoramic capture of the thermal data around the sensor.
            angle is in degrees
            :param angle: int
            :return: list of temperature values took around with the angle parameter
            """
        step = math.floor((31 * angle) / 2)

        for z in range(0, step):
            self.bus.write_byte_data(self.DEVICE_ADDRESS, 0x00, z)
            self.__getThermalData()
            time.sleep(0.1)

        return self.tpa81data


    def getSoftwareRev(self):
        """
        Returns the software revision of the sensor
        :return: int
        """
        return self.SOFT_REV


    def getAmbientTemp(self):
        """
        Returns the ambient temperature from a sensor inside the TPA81
        :return: int temperature in degrees Celsius
        """
        return self.ambient_temp

