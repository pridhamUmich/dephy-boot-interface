import math as m
from flexsea.device import Device
import configparser
from serial.tools import list_ports
import json



# P. Pridham, May 2023 

# boot_firmware_version = "7.2.0"

# left_port = "COM1"
# right_port = "COM4"

# streaming_frequency = 500

# set data values to pull
data_of_interest = [
"statetime",
"gyro_x",
"gyro_y",
"gyro_z",
"accl_x",
"accl_y",
"accl_z",
"motor_voltage",
"motor_current",
"motor_ang",
"motor_vel",
"ank_ang",
"ank_vel",
]

# set PID values to send
pid_val_current_control = {
    "kp": 40,
    "ki": 400,
    "kd": 0,
    "k": 0,
    "b": 0,
    "ff": 0,
}
 

class Exo : 
    """ 
    A class used to interface with the dephy exo.  Sends torque commands to the exo, and reads and stores data from the exo.

    ...

    Attributes
    ----------
    boot_firmware_version : string
        Stores the boot firmware version from the config file
    streaming_frequency : int
        Stores the streaming frequency from the config file.
    _device : actpack device
        Stores the actpack device that comes from the Dephy library
    raw_data : dict
        The raw data from the boot.
    data : dict
        The data converted to "normal" units, e.g. rads, Nm, A, etc.
    torque_cmd_Nm : float
        The most recent torque requested for the exo.
    current_cmd_A : float
        The most recent current value sent to the exo.
    is_left : bool
        Left side when true
    _direction : int
        1 when torque and angle don't need directional modification, -1 when the direction needs inversion from the raw.
    _wm_wa_coeffs : dict 
        Stores the polynomial coefficients for the derivative of the motor angle with respect to the ankle angle.  These are pulled from the config file.
    _wm_wa : float
        Stores the current derivative of the motor angle with respect to the ankle angle.  This is used for calculating the required motor torque to get the desired angle torque.
    _kt : float
        Motor torque constant (Nm/A).

    Methods
    -------
    __init__(port, is_left):
        Initializes the exo connected to the port, and sets the side based on is_left.  The port and id can be found using scan_for_boots() and the side can be found using get_is_left(id).
    __del__():
        Stops the motor, stops streaming, and closes the Dephy device.
    set_parameters(parameters):
        Sets the parameters for the exo.  Currently just a place holder.
    send_torque(torque_cmd_Nm):
        Sends a current command to the motor based on the provided ankle torque command in Nm, and sets the current_torque_cmd_Nm to the value sent.
    _Nm_to_A(torque_cmd_Nm):
        Calculates the motor current needed to provide the requested ankle torque. This is based on the ankle angle and the assumption that the belt is tensioned.
    _calc_wm_wa():
        Calculates the current derivative of the motor angle with respect to the ankle angle.  
    read_data():
        Reads and returns the data from the exo.
    _process_data():
        converts the raw data to 'regular' units
        
    Static Methods
    -------
    scan_for_boots(ports)
        Scans the ports to see if any boots are attached, takes in an array of port strings, returns a dict with the port and hex boot id if they are found.
    get_is_left(id)
        Checks the config file for the id and returns true if the boot is left, false if it is not left, and None if the id is not found.
    _accl_bit_to_m_s2(raw_reading)
        Converts raw accelerometer reading to m/s^2
    _gyro_bit_to_rad_s(raw_reading)
        Converts raw gyro reading to rad/s
    _ticks_to_rad(raw_reading)
        Converts the raw encoder reading to rad, this is for the motor it looks like the ankle just needs a deg->rad conversion
    print_data(data_dict)
        prints the data you give, should work for raw or processed data, or any dict really.
    """
    def __init__(self, port, is_left) : 
        """
        Constructs all the necessary attributes for the Exo.  Unset parameters are None.

        Parameters
        ----------
        port : string 
            An string containing the port to open.  These can be found using the static method scan_for_boots
        is_left : bool
            Is set to true for the left boot.  This determines how directionality is handled.
        
        Returns
        -------
        None   
        
        """
        config = configparser.ConfigParser()
        config.read('bootConfig.ini')
        
        self.boot_firmware_version = config['Boot']['Version']
        self.streaming_frequency = config.getint('Boot','Streaming Frequency')
        
        # parameters
        self._device = Device(port=port, firmwareVersion=boot_firmware_version)
        self._device.open()  
        self._device.start_streaming(self.streaming_frequency)
        self._device.set_gains(pid_val_current_control)
        
        self.raw_data = self._device.read()
        self.data = self._process_data()
        self.torque_cmd_Nm = 0
        self.current_cmd_A = 0
        self.is_left = is_left
        self._direction = 1 if is_left else -1
        # read in the values for the fit curve of the derivitave of the motor angle with respect to the ankle angle
        self._wm_wa_coeffs = {
            'poly4' : config.getfloat(self._device.id,'poly4'), 
            'poly3' : config.getfloat(self._device.id,'poly3'), 
            'poly2' : config.getfloat(self._device.id,'poly2'), 
            'poly1' : config.getfloat(self._device.id,'poly1'),  
            'poly0' : config.getfloat(self._device.id,'poly0')
        }
        # variable to store the value of the current derivitave
		self._wm_wa = 0 
        self._kt = .048 #Nm/A
    
    def __del__(self)
        """
        Stops the motors and streaming and closes the Dephy device.

        Parameters
        ----------
        None
        
        Returns
        -------
        None   
        """
        self._device.stop_motor()
        self._device.stop_streaming()
        self._device.close()
        
    def set_parameters(self, **kwargs) :
        """
        Sets the parameters for the exo.  Currently just a place holder.

        Parameters
        ----------
        Keyword Arguments:
        None
            
        Returns
        -------
        None   
        """
        pass
        
    def send_torque(self,torque_cmd_Nm)
        '''
        Sends a current command to the motor based on the provided ankle torque command in Nm, and sets the current_torque_cmd_Nm to the value sent.

        Parameters
        ----------
        torque_cmd_Nm : float
            Desired ankle torque in Nm.
        
        Returns
        -------
        None   
        '''
        
        self._device.command_motor_current(self._direction * int(1000*self._Nm_to_A(torque_cmd_Nm)))
        self.torque_cmd_Nm = torque_cmd
        
    def _Nm_to_A(self, torque_cmd_Nm)
        '''
        Calculates the motor current needed to provide the requested ankle torque. This is based on the ankle angle and the assumption that the belt is tensioned.

        Parameters
        ----------
        torque_cmd_Nm : float
            Desired ankle torque in Nm.
        
        Returns
        -------
        current_cmd_A : float
            The required motor current needed to provide the ankle torque based on the ankle position.
        '''
        self._calc_wm_wa()
        self.current_cmd_A = (torque_cmd_Nm / self._wm_wa) / self._kt # get the current based on the torque cmd and the system state
        return self.current_cmd_A
			
    def _calc_wm_wa (self) :
		'''
        Calculates the current derivative of the motor angle with respect to the ankle angle.  

        Parameters
        ----------
        None
        
        Returns
        -------
        None   
        '''
        self._wm_wa = 5 * self.data['ankle_angle_rad'] ** 4 * self._wm_wa_coeffs["poly4"] + 4 * self.data['ankle_angle_rad'] ** 3 * self._wm_wa_coeffs["poly3"] + 3 * self.data['ankle_angle_rad'] ** 2 * self._wm_wa_coeffs["poly2"] + 2 * self.data['ankle_angle_rad'] * self._wm_wa_coeffs["poly1"] + self._wm_wa_coeffs["poly0"]
		self._wm_wa = 1 if self._wm_wa <= .5 else self._wm_wa  # safety check to keep it from getting too large.
    
    def read_data(self) : 
        """
        Reads the data from the device and performs calculations to transfer the raw data to the regular data.

        Parameters
        ----------
        None   
            
        Returns
        -------
        None
        """
        self.raw_data = self._device.read()
        self._process_data()
        
    def _process_data(self) : 
        """
        Populates the data dict from the processed raw data only pulling the relevant fields.  This should only be called internally when initializing or in self.read_data()

        Parameters
        ----------
        None   
            
        Returns
        -------
        None
        """
        # TODO : figure out units and make converters 
        self.data = {
            "state_time" : self.raw_data["state_time"],
            "accl_x" : _accl_bit_to_m_s2(self.raw_data["accelx"]),
            "accl_y" : _accl_bit_to_m_s2(self.raw_data["accely"]),
            "accl_z" : _accl_bit_to_m_s2(self.raw_data["accelz"]),
            "gyro_x" : _gyro_bit_to_rad_s(self.raw_data["gyrox"]),
            "gyro_y" : _gyro_bit_to_rad_s(self.raw_data["gyroy"]),
            "gyro_z" : _gyro_bit_to_rad_s(self.raw_data["gyroz"]),
            "motor_angle_rad" : self._direction * _ticks_to_rad(self.raw_data["mot_ang"]),
            "motor_velocity_rad_s" : self._direction * self.raw_data["mot_vel"]*m.pi/180,
            "motor_current_A" : self._direction * self.raw_data["mot_cur"]/1000,
            "ankle_angle_rad" : self._direction * self.raw_data["ank_ang"]/100*m.pi/180,
            "ankle_velocity_rad_s" : self._direction * self.raw_data["ank_vel"]/10*m.pi/180,
            "battery_voltage_V" : self.raw_data["batt_volt"]/1000,
        }
        
    def print_data(data)
        '''
        prints the data you give, should work for raw or processed data, or any dict really.

        Parameters
        ----------
        data : dict
            Description
        
        Returns
        -------
        None   
        '''
        pretty_dict = json.dumps(data, indent=4)
        print(pretty_dict)
        
    def _accl_bit_to_m_s2(raw_reading)
        """
        Converts raw accelerometer reading to m/s^2
        Values from https://dephy.com/start/#units
        
        Parameters
        ----------
        raw_reading : int
            The raw reading from the accelerometer.
        
        Returns
        -------
        acceleration_m_s2 : float
            The acceleration reading converted to m/s^2
        """
        return raw_reading/8192*9.81 # raw/(LSB/g)*((m/s^2)/g)
        
    def _gyro_bit_to_rad_s(raw_reading)
        """
        Converts raw gyro reading to rad/s
        Values from https://dephy.com/start/#units
        
        Parameters
        ----------
        raw reading : int
            The raw reading from the gyro
        
        Returns
        -------
        acceleration_m_s2 : float
            The gyro reading converted to rad/s 
        """
        return raw_reading/32.8*m.pi/180 # raw/(LSB/(deg/s))*(rad/deg)
        
    def _ticks_to_rad(raw_reading)
        """
        Converts the raw encoder reading to rad, this is for the motor it looks like the ankle just needs a deg->rad conversion
        Values from https://dephy.com/start/#units
        
        Parameters
        ----------
        raw_reading : int
            raw encoder count for the motor.
        
        Returns
        -------
        encoder_angle_rad : float
            The encoder angle in radians
            
        TODO : Check with Dephy that 0.02197 is the correct multiplier.
        """
        return raw_reading*0.02197*m.pi/180 # raw*(deg/LSB)*(rad/deg)
        
    def scan_for_boots(port = 'all'):
        """
        Scans the serial ports to see if any boots are attached.
        
        Parameters
        ----------
        port : string array
            The port(s) to scan.  Default is 'all'.
        
        Returns
        -------
        boot_info : dict
            A dictionary with key strings of port names where boots are attached, and values of strings of the boot id in hexadecimal
        """
        # has only been tested with just boots. Not sure what happens when the com port is connected to something else.
        config = configparser.ConfigParser()
        config.read('bootConfig.ini')
        
        boot_firmware_version = config['Boot']['Version']
        
        if(port == 'all') :
            # get open ports 
            open_ports = list_ports.comports()
        else :
            # just check the port given
            open_ports = port
        
        # iterate over open ports and check if they are boots, if the are boots return the port and boot id.
        num_valid_boots = 0
        boot_info = {}
        for p in open_ports:
            port_name = p.device
            test_boot = Device(port=port_name, firmwareVersion=boot_firmware_version)
            
            try:
                test_boot.open()    
            except RuntimeError :
                print('RuntimeError')
            else :
                boot_info.update({port_name : f"{test_boot.id:X}"})
                # print(boot_info)
                
        return boot_info
        
    def get_is_left(dev_id):
        """
        Checks the config file for a section with the device id and returns true if the boot is left, false if it is not left, and None if the id is not found.
        
        Parameters
        ----------
        device_id : string
            A string containing the hexadecimal id of the boot to check the side of.
        
        Returns
        -------
        Side : bool 
            True - boot id is in config file and the boot is on the left side.
            False - boot id is in config file and the boot is on the right side 
            None - boot id is not in the config or the side information doesn't exist.
        """
        # has only been tested with just boots. Not sure what happens when the com port is connected to something else.
        config = configparser.ConfigParser()
        config.read('bootConfig.ini')
        
        # check the sides return true for left, false for right, and none otherwise
        try :
            is_left = True if ('left' == config[dev_id]['Side']) else False if ('right' == config[dev_id]['Side']) else None
        # I think this might error out if the id does not exist so catch it and return None. 
        else :
            is_left = None
            
        return is_left
        
    

if __name__ == '__main__':
    import numpy as np
    import matplotlib.pyplot as plt
    
    available_boots = scan_for_boots()
    for b in available_boots
        is_left = get_is_left(available_boots[b])
        if is_left :
            left_boot = Exo(b, is_left)
        else None != is_left :
            right_boot = Exo(b, is_left)
    
    
    try:
        while True :
            #this only works if there is a left and right boot. For it to work single sided you would need to check that each boot exists.
            left_boot.read_data()
            right_boot.read_data()
            
            left_boot.send_torque(1)
            right_boot.send_torque(1)
        
    except KeyboardInterrupt:
        pass
    
    left_boot.__del__() 
    right_boot.__del__()